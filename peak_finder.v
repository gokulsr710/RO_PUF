// =============================================================================
// peak_finder: locate the dense band in a 1-D edge-count profile.
//
//   1. Smooth the profile with a trailing box window of W samples
//      (sm[i] = p[i-W+1] + ... + p[i]) and store it.
//   2. Find the first maximum (peak, index am).
//   3. Grow left/right from am while sm >= max(1, peak >> SHIFT), allowing up
//      to `gap` consecutive samples below the threshold (bridges the spaces
//      between character groups). Each profile sample is first clipped to CAP.
//   4. Map smoothed indices back to profile indices:
//        lo = lo_sm - (FW-1),  hi = hi_sm - (W-FW),  FW = W >> SHIFT
//
// The profile lives in an external RAM (1-cycle read latency) addressed by
// p_raddr. Same arithmetic as peak_grow() in tools/ref_model.py.
// =============================================================================

module peak_finder #(
    parameter L     = 240,   // profile length
    parameter W     = 8,     // smoothing window
    parameter SHIFT = 1,     // grow threshold = peak >> SHIFT
    parameter AW    = 9,     // index width
    parameter DW    = 9,     // profile sample width
    parameter SW    = 13,    // smoothed-sum width
    parameter CAP   = 511    // clip each profile sample to this value
) (
    input  wire          clk,
    input  wire          start,
    input  wire [AW-1:0] gap,       // gap tolerance while growing (runtime)
    output wire [AW-1:0] p_raddr,
    input  wire [DW-1:0] p_rdata,
    output reg           done = 1'b0,     // 1-cycle pulse
    output reg  [AW-1:0] lo   = 0,
    output reg  [AW-1:0] hi   = 0,
    output reg  [SW-1:0] peak = 0
);

    localparam FW     = W >> SHIFT;
    localparam OFF_LO = FW - 1;
    localparam OFF_HI = W - FW;

    localparam S_IDLE = 3'd0, S_SM = 3'd1, S_WL_RD = 3'd2, S_WL_CK = 3'd3,
               S_WR_RD = 3'd4, S_WR_CK = 3'd5, S_FIN = 3'd6, S_SETUP = 3'd7;

    reg [2:0]    state = S_IDLE;
    reg [AW:0]   i     = 0;        // issue index (one extra bit for i == L)
    reg [AW-1:0] i_d   = 0;
    reg          v_d   = 1'b0;
    reg [SW-1:0] sum   = 0;
    reg [DW-1:0] win [0:W-1];
    reg [AW-1:0] am    = 0;
    reg [AW-1:0] j     = 0;
    reg [AW-1:0] lo_sm = 0;
    reg [AW-1:0] hi_sm = 0;
    reg [AW-1:0] last  = 0;
    reg [AW-1:0] miss  = 0;
    reg [SW-1:0] thr   = 0;

    integer k;
    initial for (k = 0; k < W; k = k + 1) win[k] = 0;

    // smoothed-profile RAM
    wire [SW-1:0] sm_q;
    wire [DW-1:0] p_c     = (p_rdata > CAP) ? CAP[DW-1:0] : p_rdata;
    wire [SW-1:0] new_sum = sum + p_c - win[W-1];
    wire [AW-1:0] sm_raddr = (state == S_WL_RD) ? (j - 1'b1) : (j + 1'b1);

    dp_ram #(.DW(SW), .AW(AW), .DEPTH(L)) u_sm (
        .wclk(clk), .we(v_d && state == S_SM), .waddr(i_d), .wdata(new_sum),
        .rclk(clk), .raddr(sm_raddr), .rdata(sm_q)
    );

    assign p_raddr = i[AW-1:0];

    // index mapping (done in S_FIN)
    wire [AW-1:0] lo_map = (lo_sm >= OFF_LO) ? (lo_sm - OFF_LO) : {AW{1'b0}};
    wire [AW-1:0] hi_raw = (hi_sm >= OFF_HI) ? (hi_sm - OFF_HI) : {AW{1'b0}};
    wire [AW-1:0] hi_map = (hi_raw < lo_map) ? lo_map : hi_raw;

    always @(posedge clk) begin
        done <= 1'b0;
        case (state)
            S_IDLE: if (start) begin
                i     <= 0;
                v_d   <= 1'b0;
                sum   <= 0;
                peak  <= 0;
                am    <= 0;
                for (k = 0; k < W; k = k + 1) win[k] <= 0;
                state <= S_SM;
            end

            S_SM: begin
                // issue side
                if (i < L) begin
                    i   <= i + 1'b1;
                    v_d <= 1'b1;
                end else begin
                    v_d <= 1'b0;
                end
                i_d <= i[AW-1:0];
                // data side (p_rdata belongs to i_d)
                if (v_d) begin
                    sum    <= new_sum;
                    win[0] <= p_c;
                    for (k = 1; k < W; k = k + 1) win[k] <= win[k-1];
                    if (new_sum > peak) begin
                        peak <= new_sum;
                        am   <= i_d;
                    end
                    if (i_d == L - 1) state <= S_SETUP;
                end
            end

            S_SETUP: begin
                thr   <= ((peak >> SHIFT) == 0) ? {{(SW-1){1'b0}}, 1'b1} : (peak >> SHIFT);
                j     <= am;
                last  <= am;
                miss  <= 0;
                state <= S_WL_RD;
            end

            // ---- grow left: sm[j-1] is read in WL_RD, checked in WL_CK ----
            S_WL_RD: begin
                if (j == 0) begin
                    lo_sm <= last;
                    j     <= am;
                    last  <= am;
                    miss  <= 0;
                    state <= S_WR_RD;
                end else begin
                    state <= S_WL_CK;
                end
            end

            S_WL_CK: begin
                if (sm_q >= thr) begin
                    j     <= j - 1'b1;
                    last  <= j - 1'b1;
                    miss  <= 0;
                    state <= S_WL_RD;
                end else if (miss < gap) begin
                    j     <= j - 1'b1;
                    miss  <= miss + 1'b1;
                    state <= S_WL_RD;
                end else begin
                    lo_sm <= last;
                    j     <= am;
                    last  <= am;
                    miss  <= 0;
                    state <= S_WR_RD;
                end
            end

            // ---- grow right: sm[j+1] is read in WR_RD, checked in WR_CK ----
            S_WR_RD: begin
                if (j == L - 1) begin
                    hi_sm <= last;
                    state <= S_FIN;
                end else begin
                    state <= S_WR_CK;
                end
            end

            S_WR_CK: begin
                if (sm_q >= thr) begin
                    j     <= j + 1'b1;
                    last  <= j + 1'b1;
                    miss  <= 0;
                    state <= S_WR_RD;
                end else if (miss < gap) begin
                    j     <= j + 1'b1;
                    miss  <= miss + 1'b1;
                    state <= S_WR_RD;
                end else begin
                    hi_sm <= last;
                    state <= S_FIN;
                end
            end

            S_FIN: begin
                lo    <= lo_map;
                hi    <= hi_map;
                done  <= 1'b1;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end

endmodule
