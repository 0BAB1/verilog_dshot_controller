// =============================================================================
// holy_dshot.sv - HOLY DSHOT: 4-channel standard (unidirectional) DShot ESC
//                 controller with an AXI4-Lite slave register interface.
//
// Sections (top to bottom):
//   1. Constants and helper functions (DShot timing, encoding, clamping)
//   2. Storage (all registers and frame-engine state)
//   3. AXI4-Lite slave: handshake, write/read decode, read mux
//   4. Control: commit, state machine, watchdog, frame scheduling
//   5. Frame engine: 16-bit shift register per channel, bit timing,
//      registered DShot outputs
//
// Register map and design decisions: see readme.md.
// =============================================================================

/* verilator lint_off UNUSEDSIGNAL */
module holy_dshot #(
    parameter int unsigned CLK_HZ = 25_000_000,  // clock used for all DShot timing
    parameter int unsigned N_CH   = 4            // fixed at 4 (register map assumes 4)
) (
    input  logic        clk,
    input  logic        rst_n,                   // synchronous, active low

    // AXI4-Lite slave (flat ports, Vivado / cocotbext-axi compatible)
    input  logic [11:0] s_axi_awaddr,
    input  logic [2:0]  s_axi_awprot,           // ignored
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [11:0] s_axi_araddr,
    input  logic [2:0]  s_axi_arprot,           // ignored
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    output logic [3:0]  dshot_o,                // DShot lines to the ESCs
    output logic        fault_irq               // level, = STATUS.NEED_RESET
);
/* verilator lint_on UNUSEDSIGNAL */

    // =========================================================================
    // 1. Constants and helper functions
    // =========================================================================

    if (N_CH != 4) begin : g_n_ch_check
        $error("holy_dshot: N_CH must be 4");
    end

    localparam int unsigned FRAME_BITS     = 16;             // bits per DShot frame
    localparam int unsigned MIN_PERIOD_BITS = FRAME_BITS + 2; // frame + 2 idle bits (REQ-12)

    localparam logic [1:0] SPD_600  = 2'd0;
    localparam logic [1:0] SPD_300  = 2'd1;
    localparam logic [1:0] SPD_150  = 2'd2;
    localparam logic [1:0] SPD_RSVD = 2'd3;

    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_SLVERR = 2'b10;

    // Controller state (STATUS.STATE encodes the same values)
    typedef enum logic [1:0] {
        S_DISABLED = 2'd0,
        S_RUNNING  = 2'd1,
        S_FAULT    = 2'd2
    } state_e;

    // Register word index = address bits [7:2]
    localparam logic [5:0] A_CTRL    = 6'h00;  // 0x00
    localparam logic [5:0] A_STATUS  = 6'h01;  // 0x04
    localparam logic [5:0] A_COMMIT  = 6'h02;  // 0x08
    localparam logic [5:0] A_FPERIOD = 6'h03;  // 0x0C
    localparam logic [5:0] A_WDG_TO  = 6'h04;  // 0x10
    localparam logic [5:0] A_FRAMES  = 6'h05;  // 0x14
    localparam logic [5:0] A_INFO    = 6'h06;  // 0x18
    localparam logic [5:0] A_THR0    = 6'h08;  // 0x20..0x2C (THROTTLE0..3)
    localparam logic [5:0] A_ACT0    = 6'h0C;  // 0x30..0x3C (ACTIVE0..3)

    // DShot bit period in clock cycles, rounded to nearest (section 3.3).
    // Reserved speed code 3 is treated as 600 by the callers (eff_spd).
    function automatic logic [15:0] t_bit(input logic [1:0] spd);
        logic [31:0] rate;
        rate = (spd == SPD_300) ? 32'd300_000 : (spd == SPD_150) ? 32'd150_000 : 32'd600_000;
        return 16'((CLK_HZ + rate / 2) / rate);
    endfunction

    // T1H = round(0.75 * T_BIT): high time of a '1' bit
    function automatic logic [15:0] t1h(input logic [1:0] spd);
        logic [31:0] tb;
        tb = 32'(t_bit(spd));
        return 16'((3 * tb + 2) / 4);
    endfunction

    // T0H = round(0.375 * T_BIT): high time of a '0' bit
    function automatic logic [15:0] t0h(input logic [1:0] spd);
        logic [31:0] tb;
        tb = 32'(t_bit(spd));
        return 16'((3 * tb + 4) / 8);
    endfunction

    // Shortest legal FRAME_PERIOD for a speed (REQ-12)
    function automatic logic [31:0] min_period(input logic [1:0] spd);
        return 32'(MIN_PERIOD_BITS) * 32'(t_bit(spd));
    endfunction

    function automatic logic [31:0] clamp_period(input logic [31:0] p, input logic [1:0] spd);
        return (p < min_period(spd)) ? min_period(spd) : p;
    endfunction

    // Speed code actually used: reserved 3 acts as 600
    function automatic logic [1:0] eff_spd(input logic [1:0] raw);
        return (raw == SPD_RSVD) ? SPD_600 : raw;
    endfunction

    // Throttle 1..47 is a special command (REQ-19)
    function automatic logic is_cmd(input logic [10:0] thr);
        return (thr != 11'd0) && (thr < 11'd48);
    endfunction

    // Register value {telemetry, throttle[10:0]} -> 16-bit DShot frame (section 3.1)
    function automatic logic [15:0] encode(input logic [11:0] r);
        logic [11:0] v;   // payload {throttle[10:0], telemetry}
        logic [3:0]  crc;
        v   = {r[10:0], r[11]};
        crc = v[3:0] ^ v[7:4] ^ v[11:8];
        return {v, crc};
    endfunction

    // =========================================================================
    // 2. Storage
    // =========================================================================

    // AXI4-Lite handshake
    logic        aw_hold_q;        // AW accepted, waiting for W
    logic        w_hold_q;         // W accepted, waiting for AW
    logic [5:0]  aw_addr_q;            // word index of the held write address
    logic [31:0] w_data_q;
    logic [3:0]  w_strb_q;
    logic        b_valid_q;
    logic [1:0]  b_resp_q;
    logic        r_valid_q;
    logic [31:0] r_data_q;

    // Control registers
    state_e      state_q;
    logic        en_q;                 // CTRL.EN
    logic [1:0]  spd_sel_q;            // CTRL.SPEED as written (3 = reserved)
    logic        mode_q;               // CTRL.MODE: 0 = TRIGGER, 1 = AUTO
    logic        wdg_en_q;             // CTRL.WDG_EN
    logic        cmd_en_q;             // CTRL.CMD_EN
    logic [31:0] frame_period_q;       // FRAME_PERIOD (clamped)
    logic [31:0] wdg_timeout_q;        // WDG_TIMEOUT
    logic [31:0] frames_q;             // FRAMES (preserved by soft reset)
    logic [11:0] staged_q [N_CH];      // THROTTLE0..3
    logic [11:0] active_q [N_CH];      // ACTIVE0..3
    logic        err_overrun_q;
    logic        err_invalid_q;
    logic        err_cfg_q;
    logic [3:0]  fault_cause_q;        // 1 = watchdog

    // Watchdog and frame scheduler
    logic [31:0] wdg_cnt_q;            // cycles since last refresh
    logic [31:0] tmr_q;                // cycles since last frame start

    // Frame engine
    logic        busy_q;               // a frame is being shifted out
    logic [3:0]  bit_q;                // index of the bit being sent (0 = first)
    logic [15:0] cyc_q;                // cycle index inside the bit
    logic [1:0]  frame_spd_q;          // speed of the frame in flight
    logic [15:0] frame_q [N_CH];       // frame shift registers, MSB = bit being sent
    logic [N_CH-1:0] dshot_q;          // registered DShot outputs

    // Frame engine next-state (combinational, see section 5)
    logic            busy_n;
    logic [3:0]      bit_n;
    logic [15:0]     cyc_n;
    logic [1:0]      spd_n;
    logic [15:0]     frame_n [N_CH];
    logic [N_CH-1:0] dshot_d;

    // =========================================================================
    // 3. AXI4-Lite slave
    //    One outstanding write (AW + W -> B) and one outstanding read (AR -> R).
    //    READY never depends on VALID. VALID is never dropped before its handshake.
    // =========================================================================

    logic        aw_fire, w_fire, ar_fire;
    logic        wr_go;                // address and data both available
    logic        wr_full;              // full-word strobe (REQ-26)
    logic        wr_ok;                // write executes a register access
    logic [31:0] wr_data;
    logic [3:0]  wr_strb;
    logic [5:0]  wr_idx;
    logic [5:0]  rd_idx;
    logic [31:0] rd_mux;

    assign aw_fire      = s_axi_awvalid && s_axi_awready;
    assign w_fire       = s_axi_wvalid  && s_axi_wready;
    assign ar_fire      = s_axi_arvalid && s_axi_arready;

    assign s_axi_awready = !aw_hold_q && !b_valid_q;
    assign s_axi_wready  = !w_hold_q  && !b_valid_q;
    assign s_axi_arready = !r_valid_q;

    // Use the held channel if it arrived first, otherwise the live one
    assign wr_go   = (aw_hold_q || aw_fire) && (w_hold_q || w_fire);
    assign wr_idx  = aw_hold_q ? aw_addr_q : s_axi_awaddr[7:2];
    assign wr_data = w_hold_q  ? w_data_q       : s_axi_wdata;
    assign wr_strb = w_hold_q  ? w_strb_q       : s_axi_wstrb;
    assign wr_full = (wr_strb == 4'hF);
    assign wr_ok   = wr_go && wr_full;
    assign rd_idx  = s_axi_araddr[7:2];

    assign s_axi_bresp = b_resp_q;
    assign s_axi_bvalid = b_valid_q;
    assign s_axi_rdata = r_data_q;
    assign s_axi_rresp = RESP_OKAY;
    assign s_axi_rvalid = r_valid_q;

    // Handshake registers. Reads have no side effects (REQ-27).
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            aw_hold_q <= 1'b0;
            w_hold_q  <= 1'b0;
            aw_addr_q <= 6'd0;
            w_data_q  <= 32'd0;
            w_strb_q  <= 4'd0;
            b_valid_q <= 1'b0;
            b_resp_q  <= RESP_OKAY;
            r_valid_q <= 1'b0;
            r_data_q  <= 32'd0;
        end else begin
            // B channel: clear on handshake
            if (b_valid_q && s_axi_bready) b_valid_q <= 1'b0;

            // Write: capture whichever channel arrives first, respond when both are in
            if (wr_go) begin
                aw_hold_q <= 1'b0;
                w_hold_q  <= 1'b0;
                b_valid_q <= 1'b1;
                b_resp_q  <= wr_full ? RESP_OKAY : RESP_SLVERR;   // partial strobe -> SLVERR (REQ-26)
            end else begin
                if (aw_fire) begin
                    aw_hold_q <= 1'b1;
                    aw_addr_q <= s_axi_awaddr[7:2];
                end
                if (w_fire) begin
                    w_hold_q <= 1'b1;
                    w_data_q <= s_axi_wdata;
                    w_strb_q <= s_axi_wstrb;
                end
            end

            // Read: sample the register mux on the address handshake
            if (ar_fire) begin
                r_valid_q <= 1'b1;
                r_data_q  <= rd_mux;
            end else if (r_valid_q && s_axi_rready) begin
                r_valid_q <= 1'b0;
            end
        end
    end

    // Read mux. Unmapped addresses read 0 (REQ-25).
    always_comb begin
        rd_mux = 32'd0;
        case (rd_idx)
            A_CTRL:    rd_mux = {25'd0, cmd_en_q, wdg_en_q, mode_q, spd_sel_q, 1'b0, en_q};
            A_STATUS:  rd_mux = {18'd0, 2'(state_q), 1'b0, err_cfg_q, err_invalid_q,
                                 err_overrun_q, fault_cause_q, 1'b0, (state_q == S_FAULT),
                                 busy_q, (state_q == S_RUNNING)};
            A_FPERIOD: rd_mux = frame_period_q;
            A_WDG_TO:  rd_mux = wdg_timeout_q;
            A_FRAMES:  rd_mux = frames_q;
            A_INFO:    rd_mux = {8'h01, 8'(N_CH), 16'(t_bit(eff_spd(spd_sel_q)))};
            default: begin
                for (int ch = 0; ch < N_CH; ch++) begin
                    if (rd_idx == A_THR0 + 6'(ch)) rd_mux = {20'd0, staged_q[ch]};
                    if (rd_idx == A_ACT0 + 6'(ch)) rd_mux = {20'd0, active_q[ch]};
                end
            end
        endcase
    end

    // Write decode (only full-word writes reach a register)
    logic soft_rst, wr_ctrl, wr_commit, wr_status, wr_fper, wr_wdgto;
    logic en_rise, en_fall;

    assign soft_rst  = wr_ok && (wr_idx == A_CTRL) && wr_data[1];    // SOFT_RESET is W1S
    assign wr_ctrl   = wr_ok && (wr_idx == A_CTRL) && !wr_data[1];
    assign wr_commit = wr_ok && (wr_idx == A_COMMIT);
    assign wr_status = wr_ok && (wr_idx == A_STATUS);
    assign wr_fper   = wr_ok && (wr_idx == A_FPERIOD);
    assign wr_wdgto  = wr_ok && (wr_idx == A_WDG_TO);
    assign en_rise   = wr_ctrl && wr_data[0] && !en_q;
    assign en_fall   = wr_ctrl && !wr_data[0] && en_q;

    // =========================================================================
    // 4. Control: commit, state machine, watchdog, frame scheduling
    // =========================================================================

    logic running, faulted;
    logic stop_req;                    // abort the frame in flight
    logic period_hit;                  // AUTO / stop-frame period elapsed
    logic start_auto, start_trig, start;
    logic [11:0] act_next [N_CH];      // active values after this cycle
    logic inv_set;                     // CMD_EN=0 rejected a command value (REQ-19)
    logic ov_set;                      // TRIGGER COMMIT while a frame is in flight
    logic cfg_err;                     // ERR_CFG set condition this cycle
    logic refresh;                     // restart the watchdog counter
    logic wdg_active, wdg_expire;
    logic [31:0] wdg_cnt_n;

    assign running    = (state_q == S_RUNNING);
    assign faulted    = (state_q == S_FAULT);
    assign stop_req   = soft_rst || en_fall;
    assign period_hit = (tmr_q >= frame_period_q - 32'd1);

    // Periodic frames: AUTO while RUNNING, and stop frames while FAULT with EN=1 (REQ-17)
    assign start_auto = (running && mode_q) || (faulted && en_q);
    // TRIGGER: one frame per COMMIT (REQ-10)
    assign start_trig = wr_commit && running && !mode_q;
    assign start      = !busy_q && !stop_req && ((start_auto && period_hit) || start_trig);

    assign ov_set  = wr_commit && running && !mode_q && busy_q;
    assign cfg_err = (wr_ctrl && ((wr_data[3:2] == SPD_RSVD) ||
                                  (clamp_period(frame_period_q, eff_spd(wr_data[3:2])) != frame_period_q))) ||
                     (wr_fper && (clamp_period(wr_data, eff_spd(spd_sel_q)) != wr_data)) ||
                     (wr_wdgto && (wr_data == 32'd0));

    // Commit: copy staged -> active for all channels at once (REQ-07).
    // With CMD_EN=0, a staged command value keeps the previous active value (REQ-19).
    always_comb begin
        inv_set = 1'b0;
        for (int ch = 0; ch < N_CH; ch++) begin
            act_next[ch] = active_q[ch];
            if (wr_commit) begin
                if (!cmd_en_q && is_cmd(staged_q[ch][10:0])) begin
                    inv_set = 1'b1;
                end else begin
                    act_next[ch] = staged_q[ch];
                end
            end
        end
    end

    // Watchdog: counts cycles since the last COMMIT or since entering RUNNING (REQ-09, REQ-15).
    // Always active in AUTO, active in TRIGGER only with WDG_EN (REQ-14).
    assign refresh    = (wr_commit && running) || en_rise;
    assign wdg_cnt_n  = refresh ? 32'd0 : ((wdg_cnt_q == '1) ? wdg_cnt_q : wdg_cnt_q + 32'd1);
    assign wdg_active = running && (mode_q || wdg_en_q);
    assign wdg_expire = wdg_active && !refresh && (wdg_cnt_n >= wdg_timeout_q);

    // State machine (section 4.5):
    //   reset / SOFT_RESET -> DISABLED (handled in the always_ff reset branch)
    //   DISABLED --EN 0->1--> RUNNING
    //   RUNNING  --EN 1->0--> DISABLED
    //   RUNNING  --watchdog-> FAULT
    //   FAULT    latched until SOFT_RESET
    always_ff @(posedge clk) begin
        if (!rst_n || soft_rst) begin
            // Hardware reset or CTRL.SOFT_RESET: everything back to reset values,
            // except FRAMES (handled in its own block) and the AXI handshake.
            state_q        <= S_DISABLED;
            en_q           <= 1'b0;
            spd_sel_q      <= SPD_600;
            mode_q         <= 1'b0;
            wdg_en_q       <= 1'b0;
            cmd_en_q       <= 1'b0;
            frame_period_q <= 32'(CLK_HZ / 1000);
            wdg_timeout_q  <= 32'(CLK_HZ / 20);
            for (int ch = 0; ch < N_CH; ch++) begin
                staged_q[ch] <= 12'd0;
                active_q[ch] <= 12'd0;
            end
            err_overrun_q <= 1'b0;
            err_invalid_q <= 1'b0;
            err_cfg_q     <= 1'b0;
            fault_cause_q <= 4'd0;
            wdg_cnt_q     <= 32'd0;
            tmr_q         <= 32'd0;
            busy_q        <= 1'b0;
            bit_q         <= 4'd0;
            cyc_q         <= 16'd0;
            frame_spd_q   <= SPD_600;
            for (int ch = 0; ch < N_CH; ch++) begin
                frame_q[ch] <= 16'd0;
            end
            dshot_q       <= '0;
        end else begin
            // ---- CTRL and config writes ----
            if (wr_ctrl) begin
                en_q      <= wr_data[0];
                spd_sel_q <= wr_data[3:2];
                mode_q    <= wr_data[4];
                wdg_en_q  <= wr_data[5];
                cmd_en_q  <= wr_data[6];
                // A speed change re-clamps FRAME_PERIOD to the new minimum
                frame_period_q <= clamp_period(frame_period_q, eff_spd(wr_data[3:2]));
            end
            if (wr_fper)  frame_period_q <= clamp_period(wr_data, eff_spd(spd_sel_q));
            if (wr_wdgto) wdg_timeout_q  <= (wr_data == 32'd0) ? 32'(CLK_HZ / 20) : wr_data;

            // ---- Staged values and active values (commit) ----
            for (int ch = 0; ch < N_CH; ch++) begin
                if (wr_ok && (wr_idx == A_THR0 + 6'(ch))) staged_q[ch] <= wr_data[11:0];
                active_q[ch] <= en_rise ? 12'd0 : act_next[ch];   // EN 0->1: active = stop
            end

            // ---- Sticky errors: W1C first, then set events win in the same cycle ----
            if (wr_status) begin
                if (wr_data[8])  err_overrun_q <= 1'b0;
                if (wr_data[9])  err_invalid_q <= 1'b0;
                if (wr_data[10]) err_cfg_q     <= 1'b0;
            end
            if (ov_set)  err_overrun_q <= 1'b1;
            if (inv_set) err_invalid_q <= 1'b1;
            if (cfg_err) err_cfg_q     <= 1'b1;

            // ---- State ----
            case (state_q)
                S_DISABLED: begin
                    if (en_rise) state_q <= S_RUNNING;
                end
                S_RUNNING: begin
                    if (en_fall) begin
                        state_q <= S_DISABLED;
                    end else if (wdg_expire) begin
                        state_q       <= S_FAULT;
                        fault_cause_q <= 4'd1;         // 1 = watchdog
                    end
                end
                default: begin
                    // S_FAULT: latched (REQ-18), only SOFT_RESET leaves it
                end
            endcase

            // ---- Watchdog and frame scheduler ----
            wdg_cnt_q <= wdg_cnt_n;
            tmr_q     <= (start || en_rise) ? 32'd0 :
                         ((tmr_q == '1) ? tmr_q : tmr_q + 32'd1);

            // ---- Frame engine registers ----
            busy_q      <= busy_n;
            bit_q       <= bit_n;
            cyc_q       <= cyc_n;
            frame_spd_q <= spd_n;
            for (int ch = 0; ch < N_CH; ch++) begin
                frame_q[ch] <= frame_n[ch];
            end
            dshot_q <= dshot_d;
        end
    end

    // FRAMES: frames started since hardware reset. Preserved by SOFT_RESET (REQ-23).
    always_ff @(posedge clk) begin
        if (!rst_n)      frames_q <= 32'd0;
        else if (start)  frames_q <= frames_q + 32'd1;
    end

    // =========================================================================
    // 5. Frame engine
    //    A frame is 16 bits, each bit lasting t_bit cycles. The line is high for
    //    t1h (bit = 1) or t0h (bit = 0) cycles at the start of each bit.
    //    All channels share the bit counters, so the four frames are aligned
    //    to the same cycle (REQ-01) by construction.
    //    The next-state logic is combinational; outputs are registered (REQ-02).
    // =========================================================================

    always_comb begin
        busy_n = busy_q;
        bit_n  = bit_q;
        cyc_n  = cyc_q;
        spd_n  = frame_spd_q;
        for (int ch = 0; ch < N_CH; ch++) begin
            frame_n[ch] = frame_q[ch];
        end

        if (start) begin
            // New frame: latch speed and the active values (stop frame in FAULT)
            busy_n = 1'b1;
            bit_n  = 4'd0;
            cyc_n  = 16'd0;
            spd_n  = eff_spd(spd_sel_q);
            for (int ch = 0; ch < N_CH; ch++) begin
                frame_n[ch] = faulted ? 16'h0000 : encode(act_next[ch]);
            end
        end else if (busy_q) begin
            if (cyc_q == t_bit(frame_spd_q) - 16'd1) begin
                // End of bit: move to the next bit (or finish the frame)
                cyc_n = 16'd0;
                bit_n = bit_q + 4'd1;
                if (bit_q == 4'd15) busy_n = 1'b0;
                for (int ch = 0; ch < N_CH; ch++) begin
                    frame_n[ch] = frame_q[ch] << 1;
                end
            end else begin
                cyc_n = cyc_q + 16'd1;
            end
        end

        // EN 1->0 or SOFT_RESET aborts the frame in flight (REQ-02, section 4.5)
        if (stop_req) busy_n = 1'b0;
    end

    // Level of each line in the next cycle
    always_comb begin
        for (int ch = 0; ch < N_CH; ch++) begin
            dshot_d[ch] = busy_n && (cyc_n < (frame_n[ch][15] ? t1h(spd_n) : t0h(spd_n)));
        end
    end

    assign dshot_o   = dshot_q;
    assign fault_irq = faulted;

endmodule
