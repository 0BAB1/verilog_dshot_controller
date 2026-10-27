// =============================================================================
// holy_dshot.sv - 4-channel DShot300 ESC output with an AXI4-Lite register slave.
//
// Every channel sends one DShot300 frame after another. The throttle of a
// frame is read from its register when the frame starts. There is no staging:
// a register write takes effect on the next frame of that channel.
//
// Safety:
//   - Only frames with throttle 0 (stop) are sent while CTRL.EN = 0.
//   - Throttle values 1..47 are DShot special commands. They are never sent,
//     the channel sends stop instead.
//   - After reset or soft reset the outputs send stop frames, so the ESCs
//     can arm and stay stopped.
//
// Register map and usage: see readme.md.
// =============================================================================

/* verilator lint_off UNUSEDSIGNAL */
module holy_dshot #(
    parameter int unsigned CLK_HZ = 25_000_000   // clock used for DShot300 timing
) (
    input  logic        clk,
    input  logic        rst_n,                   // synchronous, active low

    // AXI4-Lite slave, 16-bit address (64 KB)
    input  logic [15:0] s_axi_awaddr,
    input  logic [2:0]  s_axi_awprot,            // ignored
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [15:0] s_axi_araddr,
    input  logic [2:0]  s_axi_arprot,            // ignored
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    output logic [3:0]  dshot_o                  // DShot lines to the ESCs
);
/* verilator lint_on UNUSEDSIGNAL */

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    // DShot300 timing in clock cycles (requirements section 3.3)
    localparam logic [15:0] T_BIT = 16'((CLK_HZ + 150_000) / 300_000);
    localparam logic [15:0] T1H   = 16'((3 * T_BIT + 2) / 4);
    localparam logic [15:0] T0H   = 16'((3 * T_BIT + 4) / 8);
    localparam logic [4:0]  SLOTS = 5'd18;       // 16 data bits + 2 idle bit times

    // Register addresses (byte addresses)
    localparam logic [15:0] A_CTRL = 16'h0000;   // bit 0 EN, bit 1 SOFT_RESET (W1S)
    localparam logic [15:0] A_THR0 = 16'h0004;   // THROTTLE0..3 at 0x04, 0x08, 0x0C, 0x10

    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_SLVERR = 2'b10;

    // Frame word for one channel: stop unless enabled and a real throttle (48..2047).
    // Layout: throttle[10:0], telemetry request = 0, CRC over the 12-bit payload.
    function automatic logic [15:0] dshot_frame(input logic en, input logic [10:0] thr);
        logic [10:0] t;
        logic [11:0] v;
        logic [3:0]  crc;
        t   = (en && (thr >= 11'd48)) ? thr : 11'd0;
        v   = {t, 1'b0};
        crc = v[3:0] ^ v[7:4] ^ v[11:8];
        return {v, crc};
    endfunction

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    logic        en_q;                 // CTRL.EN
    logic [10:0] thr_q [4];            // THROTTLE0..3

    // -------------------------------------------------------------------------
    // Frame engine
    // -------------------------------------------------------------------------
    logic [15:0] cyc_q;                // clock cycle inside the current bit
    logic [4:0]  slot_q;               // bit slot 0..17 inside the frame
    logic [15:0] shift_q [4];          // frame being sent, MSB = current bit
    logic [3:0]  dshot_q;              // registered output
    logic [3:0]  dshot_d;              // next output

    always_comb begin
        for (int ch = 0; ch < 4; ch++) begin
            // Line is high for T1H (bit 1) or T0H (bit 0) at the start of the bit
            dshot_d[ch] = (slot_q < 5'd16) &&
                          (cyc_q < (shift_q[ch][15] ? T1H : T0H));
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cyc_q  <= 16'd0;
            slot_q <= 5'd0;
            for (int ch = 0; ch < 4; ch++) shift_q[ch] <= 16'd0;   // stop frame
            dshot_q <= 4'd0;
        end else begin
            dshot_q <= dshot_d;
            if (cyc_q == T_BIT - 16'd1) begin
                cyc_q <= 16'd0;
                if (slot_q == SLOTS - 5'd1) begin
                    // Frame boundary: load the next frame from the registers
                    slot_q <= 5'd0;
                    for (int ch = 0; ch < 4; ch++)
                        shift_q[ch] <= dshot_frame(en_q, thr_q[ch]);
                end else begin
                    slot_q <= slot_q + 5'd1;
                    for (int ch = 0; ch < 4; ch++)
                        shift_q[ch] <= shift_q[ch] << 1;
                end
            end else begin
                cyc_q <= cyc_q + 16'd1;
            end
        end
    end

    assign dshot_o = dshot_q;

    // -------------------------------------------------------------------------
    // AXI4-Lite slave
    //   One outstanding write and one outstanding read.
    //   Writes wait until both AW and W have arrived.
    // -------------------------------------------------------------------------
    logic        aw_hold_q, w_hold_q;
    logic [15:0] aw_addr_q;
    logic [10:0] w_data_q;
    logic [3:0]  w_strb_q;
    logic        b_valid_q;
    logic [1:0]  b_resp_q;
    logic        r_valid_q;
    logic [31:0] r_data_q;

    logic        aw_fire, w_fire, ar_fire, wr_go, wr_full;
    logic [15:0] wr_addr;
    logic [10:0] wr_data;              // only bits 10:0 are used (throttle, CTRL bits)
    logic [3:0]  wr_strb;
    logic [31:0] rd_mux;

    assign s_axi_awready = !aw_hold_q && !b_valid_q;
    assign s_axi_wready  = !w_hold_q  && !b_valid_q;
    assign s_axi_arready = !r_valid_q;

    assign aw_fire = s_axi_awvalid && s_axi_awready;
    assign w_fire  = s_axi_wvalid  && s_axi_wready;
    assign ar_fire = s_axi_arvalid && s_axi_arready;

    assign wr_go   = (aw_hold_q || aw_fire) && (w_hold_q || w_fire);
    assign wr_addr = aw_hold_q ? aw_addr_q : s_axi_awaddr;
    assign wr_data = w_hold_q  ? w_data_q : s_axi_wdata[10:0];
    assign wr_strb = w_hold_q  ? w_strb_q  : s_axi_wstrb;
    assign wr_full = (wr_strb == 4'hF);

    assign s_axi_bresp  = b_resp_q;
    assign s_axi_bvalid = b_valid_q;
    assign s_axi_rdata  = r_data_q;
    assign s_axi_rresp  = RESP_OKAY;
    assign s_axi_rvalid = r_valid_q;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            aw_hold_q <= 1'b0;
            w_hold_q  <= 1'b0;
            aw_addr_q <= 16'd0;
            w_data_q  <= 11'd0;
            w_strb_q  <= 4'd0;
            b_valid_q <= 1'b0;
            b_resp_q  <= RESP_OKAY;
            r_valid_q <= 1'b0;
            r_data_q  <= 32'd0;
        end else begin
            if (b_valid_q && s_axi_bready) b_valid_q <= 1'b0;

            if (wr_go) begin
                aw_hold_q <= 1'b0;
                w_hold_q  <= 1'b0;
                b_valid_q <= 1'b1;
                b_resp_q  <= wr_full ? RESP_OKAY : RESP_SLVERR;   // partial write: refused
            end else begin
                if (aw_fire) begin
                    aw_hold_q <= 1'b1;
                    aw_addr_q <= s_axi_awaddr;
                end
                if (w_fire) begin
                    w_hold_q <= 1'b1;
                    w_data_q <= s_axi_wdata[10:0];
                    w_strb_q <= s_axi_wstrb;
                end
            end

            if (ar_fire) begin
                r_valid_q <= 1'b1;
                r_data_q  <= rd_mux;
            end else if (r_valid_q && s_axi_rready) begin
                r_valid_q <= 1'b0;
            end
        end
    end

    // Read: no side effects. Unmapped addresses read 0.
    always_comb begin
        rd_mux = 32'd0;
        if (s_axi_araddr == A_CTRL) rd_mux = {31'd0, en_q};
        for (int ch = 0; ch < 4; ch++) begin
            if (s_axi_araddr == A_THR0 + 16'(4 * ch)) rd_mux = {21'd0, thr_q[ch]};
        end
    end

    // Write: full-word writes only. Unmapped writes are ignored.
    logic soft_rst;
    assign soft_rst = wr_go && wr_full && (wr_addr == A_CTRL) && wr_data[1];

    always_ff @(posedge clk) begin
        if (!rst_n || soft_rst) begin
            en_q <= 1'b0;
            for (int ch = 0; ch < 4; ch++) thr_q[ch] <= 11'd0;
        end else if (wr_go && wr_full) begin
            if (wr_addr == A_CTRL) begin
                en_q <= wr_data[0];
                // Enable always starts from stop. A throttle left over from before
                // the enable must not spin the motor. Software writes the
                // throttles after enabling.
                if (wr_data[0] && !en_q)
                    for (int ch = 0; ch < 4; ch++) thr_q[ch] <= 11'd0;
            end
            for (int ch = 0; ch < 4; ch++) begin
                if (wr_addr == A_THR0 + 16'(4 * ch)) thr_q[ch] <= wr_data[10:0];
            end
        end
    end

endmodule
