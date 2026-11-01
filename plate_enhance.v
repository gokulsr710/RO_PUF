// =============================================================================
// plate_enhance: still-image number-plate localization + ROI histogram
// equalization, all on the 50 MHz clock.
//
// Data flow (all images 320x240, 8-bit grey):
//   buf_in   (written by the camera, read here)
//   buf_orig (copy of the snapshot, for the "before" view)
//   buf_out  (enhanced result, for the "after" view)
//
// Passes after a snapshot (about 16 ms in total):
//   CLRB/HIST/BWALK : brightness histogram -> bright level tb = 85th percentile.
//            At night the plate (retro-reflective, lit) is among the brightest
//            things in the frame.
//   CLRG/GRAD/THR : histogram of Sobel-X gradients at CANDIDATE positions
//            (brighter side of the edge >= tb: dark text on a bright plate);
//            ADAPTIVE threshold keeps the strongest 35% of them
//            (skipped when a manual threshold is given)
//   ROWS   : copy buf_in -> buf_orig; candidate edges -> rowp[240], total count
//   FINDY  : peak_finder on rowp  -> plate rows   (y0..y1)
//            plate found if peak >= 2x average row density
//   COLS   : column edge counts inside rows y0..y1 -> colp[320]
//            and bright-pixel counts per column inside the band -> colb[320]
//   FINDX  : peak_finder on colp (each column clipped to CAP_X, gaps up to the
//            band height bridged, so it scales with plate size) -> x span
//   BRUN   : keep only the continuous bright part (column >= 50% bright,
//            gaps <= BGAP) around the centre of that span  -> plate cols
//   HIST   : 256-bin histogram of the ROI (raw pixels)
//   CDF    : CONTRAST-LIMITED cumulative histogram: every bin clipped to
//            CLIP_K * N/256 so flat dark areas do not turn into noise
//   DIV    : scale = (255 << 20) / (N - cdf_min)      (28-cycle divider)
//   LUT    : lut[v] = round((cdf[v]-cdf_min) * scale / 2^20), clamped to 255
//   APPLY  : out = ROI ? lut[p] : (outside_gamma ? gamma(p) : p)
//
// Bit-exact with tools/ref_model.py (keep the localparams below in sync).
// =============================================================================

module plate_enhance (
    input  wire        clk,

    // control (1-cycle pulses, clk domain)
    input  wire        snap,          // take a snapshot and process it
    input  wire        go_live,       // return to live preview
    input  wire        reprocess,     // re-run processing on the held frame
    input  wire        test_mode,     // 1: no camera, buf_in holds a test image

    // camera handshake
    input  wire        frame_tog,     // from ov7670_capture_y (PCLK domain)
    output reg         cap_enable = 1'b1,

    // settings (quasi-static, from switches)
    input  wire [9:0]  edge_thr,      // 0 = adaptive (default), else fixed threshold
    input  wire        gamma_detect,  // use gamma-boosted pixels for edge detection
    input  wire        outside_gamma, // gamma-boost pixels outside the ROI
    input  wire        force_global,  // equalize the whole frame instead of the ROI

    // buf_in read port (1-cycle latency)
    output wire [16:0] in_raddr,
    input  wire [7:0]  in_rdata,

    // buf_orig / buf_out write ports
    output wire        orig_we,
    output wire        out_we,
    output wire [16:0] wr_addr,
    output wire [7:0]  orig_wdata,
    output wire [7:0]  out_wdata,

    // status
    output wire        live,
    output wire        busy,
    output wire        held,
    output reg         plate_found = 1'b0,
    output reg  [8:0]  box_x0 = 9'd0,
    output reg  [8:0]  box_x1 = 9'd0,
    output reg  [7:0]  box_y0 = 8'd0,
    output reg  [7:0]  box_y1 = 8'd0
);

    // ---- Tunable parameters (match tools/ref_model.py) ----------------------
    localparam BRIGHT_RANK = 65280;          // 85th percentile of 76800 pixels
    localparam EDGE_MUL = 179, EDGE_SH = 9;  // keep strongest 179/512 of candidates
    localparam MIN_THR = 24;
    localparam WIN_Y = 8,  SHIFT_Y = 1;
    localparam WIN_X = 16, SHIFT_X = 3;
    localparam CAP_X = 8;
    localparam BGAP = 8;
    localparam PAD_Y = 3,  PAD_X = 6;
    localparam MIN_PEAK_Y = 32;
    localparam FOUND_K = 15;
    localparam CLIP_K = 3;                   // 0 = plain HE
    localparam FRAC = 20;

    localparam IMG_W = 320, IMG_H = 240;

    // ---- States -------------------------------------------------------------
    localparam S_LIVE    = 5'd0,  S_SNAPWAIT = 5'd1,  S_DRAIN  = 5'd2,
               S_ROWS    = 5'd3,  S_FINDY    = 5'd4,  S_CLRCOL = 5'd5,
               S_COLS    = 5'd6,  S_FINDX    = 5'd7,  S_CLRH   = 5'd8,
               S_HIST    = 5'd9,  S_CDF      = 5'd10, S_DIV    = 5'd11,
               S_LUT     = 5'd12, S_APPLY    = 5'd13, S_HOLD   = 5'd14,
               S_CLRG    = 5'd15, S_GRAD     = 5'd16, S_THR    = 5'd17,
               S_CLRB    = 5'd18, S_BWALK    = 5'd19, S_BL_RD  = 5'd20,
               S_BL_CK   = 5'd21, S_BR_RD    = 5'd22, S_BR_CK  = 5'd23,
               S_BFIN    = 5'd24;

    reg [4:0] state = S_LIVE;
    reg       live_pend = 1'b0;

    assign live = (state == S_LIVE) || (state == S_SNAPWAIT);
    assign held = (state == S_HOLD);
    assign busy = !live && !held;

    // ---- Frame-done event from the camera domain ----------------------------
    reg [2:0] ft_s = 3'b000;
    always @(posedge clk) ft_s <= {ft_s[1:0], frame_tog};
    wire frame_ev = ft_s[2] ^ ft_s[1];

    // ---- Raster scanner over a rectangle -------------------------------------
    reg        scan_on = 1'b1;
    reg [8:0]  sx = 9'd0, rx0 = 9'd0, rx1 = IMG_W - 1;
    reg [7:0]  sy = 8'd0, ry0 = 8'd0, ry1 = IMG_H - 1;
    wire [16:0] scan_addr = {1'b0, sy, 8'b0} + {3'b0, sy, 6'b0} + {8'b0, sx};
    wire        scan_last = (sx == rx1) && (sy == ry1);

    assign in_raddr = scan_addr;

    // ---- Pipeline registers (1 pixel / clock passes) --------------------------
    reg        v1 = 1'b0, v2 = 1'b0;
    reg [8:0]  x1 = 9'd0, x2 = 9'd0;
    reg [7:0]  y1 = 8'd0, y2 = 8'd0;
    reg [16:0] a1 = 17'd0, a2 = 17'd0;
    wire       pipe_empty = !scan_on && !v1 && !v2;

    // ---- Gamma LUT (shared; input is always the buf_in read data) -----------
    wire [7:0] gam;
    gamma_lut u_gamma (.din(in_rdata), .dout(gam));
    wire [7:0] bpix = gamma_detect ? gam : in_rdata;

    // ---- Sobel-X line buffers -------------------------------------------------
    wire [7:0] lb1_q, lb2_q;
    wire       sobel_pass = (state == S_ROWS) || (state == S_COLS) || (state == S_GRAD);
    dp_ram #(.DW(8), .AW(9), .DEPTH(IMG_W)) u_lb1 (
        .wclk(clk), .we(v1 && sobel_pass), .waddr(x1), .wdata(bpix),
        .rclk(clk), .raddr(sx), .rdata(lb1_q));
    dp_ram #(.DW(8), .AW(9), .DEPTH(IMG_W)) u_lb2 (
        .wclk(clk), .we(v1 && sobel_pass), .waddr(x1), .wdata(lb1_q),
        .rclk(clk), .raddr(sx), .rdata(lb2_q));

    // column sum s(x,y) = b(x,y-2) + 2 b(x,y-1) + b(x,y)
    wire [9:0] s_col = {2'b00, lb2_q} + {1'b0, lb1_q, 1'b0} + {2'b00, bpix};
    reg  [9:0] sq0 = 10'd0, sq1 = 10'd0, sq2 = 10'd0;
    wire [9:0] gx   = (sq0 > sq2) ? (sq0 - sq2) : (sq2 - sq0);
    reg  [10:0] thr_used = 11'd96;  // active edge threshold (adaptive or fixed)
    reg  [7:0]  tb = 8'd0;          // bright level (detection domain)
    // centre-row samples b(x, y-1) delayed: c0 = column x, c2 = column x-2
    reg  [7:0]  c0 = 8'd0, c1 = 8'd0, c2 = 8'd0;
    wire [7:0]  side   = (c0 > c2) ? c0 : c2;           // brighter side of the edge
    wire       pos_ok   = v2 && (x2 >= 9'd2) && (y2 >= 8'd2);
    wire       cand     = pos_ok && (side >= tb);
    wire       edge_raw = cand && ({1'b0, gx} >= thr_used);
    wire [8:0] ex = x2 - 1'b1;      // edge centre column
    wire [7:0] ey = y2 - 1'b1;      // edge centre row

    // ---- Row profile RAM ------------------------------------------------------
    reg  [8:0] row_acc = 9'd0;
    reg  [16:0] e_total = 17'd0;      // total edge count (for the plate-found test)
    wire       rowp_we    = (state == S_ROWS) && v2 && (x2 == IMG_W - 1);
    wire [7:0] rowp_waddr = (y2 == 8'd0) ? (IMG_H - 1) : ey;
    wire [8:0] rowp_wdata = (y2 >= 8'd2) ? (row_acc + edge_raw) : 9'd0;
    wire [8:0] rowp_q;
    wire [8:0] pfy_raddr;
    dp_ram #(.DW(9), .AW(8), .DEPTH(IMG_H)) u_rowp (
        .wclk(clk), .we(rowp_we), .waddr(rowp_waddr), .wdata(rowp_wdata),
        .rclk(clk), .raddr(pfy_raddr[7:0]), .rdata(rowp_q));

    // ---- Column profile RAM ---------------------------------------------------
    reg  [8:0] clr_cnt = 9'd0;
    reg  [7:0] band_y0 = 8'd0, band_y1 = 8'd0;
    wire [8:0] colp_q;
    wire [8:0] pfx_raddr;
    wire       in_band   = (ey >= band_y0) && (ey <= band_y1);
    wire       colp_we   = (state == S_CLRCOL) ||
                           ((state == S_COLS) && v2 && (x2 >= 9'd2) && (y2 >= 8'd2) && in_band);
    wire [8:0] colp_waddr = (state == S_CLRCOL) ? clr_cnt : ex;
    wire [8:0] colp_wdata = (state == S_CLRCOL) ? 9'd0 : (colp_q + edge_raw);
    wire [8:0] colp_raddr = (state == S_COLS) ? (x1 - 1'b1) : pfx_raddr;
    dp_ram #(.DW(9), .AW(9), .DEPTH(IMG_W)) u_colp (
        .wclk(clk), .we(colp_we), .waddr(colp_waddr), .wdata(colp_wdata),
        .rclk(clk), .raddr(colp_raddr), .rdata(colp_q));

    // bright-pixel count per column inside the band (pixel-level, at the v1 stage)
    reg  [8:0] bj = 9'd0, blast = 9'd0, bmiss = 9'd0;   // bright-run grow
    wire [7:0] colb_q;
    wire       px_in_band = (y1 >= band_y0) && (y1 <= band_y1);
    wire       colb_we    = (state == S_CLRCOL) || ((state == S_COLS) && v1 && px_in_band);
    wire [8:0] colb_waddr = (state == S_CLRCOL) ? clr_cnt : x1;
    wire [7:0] colb_wdata = (state == S_CLRCOL) ? 8'd0 : (colb_q + (bpix >= tb));
    wire [8:0] colb_raddr = (state == S_COLS)  ? sx :
                            (state == S_BL_RD) ? (bj - 1'b1) : (bj + 1'b1);
    dp_ram #(.DW(8), .AW(9), .DEPTH(IMG_W)) u_colb (
        .wclk(clk), .we(colb_we), .waddr(colb_waddr), .wdata(colb_wdata),
        .rclk(clk), .raddr(colb_raddr), .rdata(colb_q));
    wire [8:0] band_h = {1'b0, band_y1} - {1'b0, band_y0} + 1'b1;
    wire       col_bright = ({colb_q, 1'b0} >= band_h);       // >= 50% bright

    // ---- Peak finders ---------------------------------------------------------
    reg         pfy_start = 1'b0, pfx_start = 1'b0;
    wire        pfy_done, pfx_done;
    wire [8:0]  pfy_lo, pfy_hi, pfx_lo, pfx_hi;
    wire [12:0] pfy_peak, pfx_peak;

    peak_finder #(.L(IMG_H), .W(WIN_Y), .SHIFT(SHIFT_Y), .AW(9), .DW(9), .SW(13)) u_pfy (
        .clk(clk), .start(pfy_start), .gap(9'd0), .p_raddr(pfy_raddr), .p_rdata(rowp_q),
        .done(pfy_done), .lo(pfy_lo), .hi(pfy_hi), .peak(pfy_peak));

    peak_finder #(.L(IMG_W), .W(WIN_X), .SHIFT(SHIFT_X), .AW(9), .DW(9), .SW(13),
                  .CAP(CAP_X)) u_pfx (
        .clk(clk), .start(pfx_start), .gap(band_h), .p_raddr(pfx_raddr), .p_rdata(colp_q),
        .done(pfx_done), .lo(pfx_lo), .hi(pfx_hi), .peak(pfx_peak));

    // ---- Histogram / CDF RAM (256 x 17) ----------------------------------------
    reg  [1:0]  ph = 2'd0;            // sub-phase for HIST / CDF / LUT
    reg  [8:0]  v_cnt = 9'd0;         // bin counter (9 bits so it can reach 256)
    reg  [7:0]  p_h = 8'd0;
    reg  [16:0] npix = 17'd0;
    reg  [16:0] cdf = 17'd0;
    reg  [16:0] cdf_min = 17'd0;
    reg         min_found = 1'b0;
    wire [16:0] hist_q;
    reg  [16:0] clip = 17'd1;                 // contrast limit per bin
    wire [16:0] hq_c    = (CLIP_K != 0 && hist_q > clip) ? clip : hist_q;
    wire [16:0] cdf_new = cdf + hq_c;
    reg         bmode = 1'b0;                 // HIST pass builds the brightness histogram
    reg  [16:0] n_cand = 17'd0;               // number of candidate positions
    reg  [16:0] e_tgt = 17'd0;
    wire [7:0]  gam_v;                        // gamma of the bin counter (for tb)
    gamma_lut u_gamma_tb (.din(v_cnt[7:0]), .dout(gam_v));

    // gradient histogram (GRAD pass): bin = gx >> 2, read at the v2 cycle,
    // incremented one cycle later (pixels are 3 clocks apart: no hazard)
    reg         g_wr = 1'b0;
    reg  [7:0]  gbin_d = 8'd0;
    reg  [16:0] g_acc = 17'd0;
    wire [17:0] g_sum = {1'b0, g_acc} + {1'b0, hist_q};
    wire        clr_any = (state == S_CLRH) || (state == S_CLRG) || (state == S_CLRB);

    wire        hist_we = clr_any || g_wr ||
                          ((state == S_HIST) && ph == 2'd2) ||
                          ((state == S_CDF)  && ph == 2'd1);
    wire [7:0]  hist_waddr = clr_any ? clr_cnt[7:0] :
                             g_wr ? gbin_d :
                             (state == S_HIST) ? p_h : v_cnt[7:0];
    wire [16:0] hist_wdata = clr_any ? 17'd0 :
                             (g_wr || state == S_HIST) ? (hist_q + 1'b1) : cdf_new;
    // (in S_CDF the clipped cumulative histogram is written back in place)
    wire [7:0]  hist_raddr = (state == S_HIST) ? in_rdata :
                             (state == S_GRAD) ? gx[9:2] : v_cnt[7:0];
    dp_ram #(.DW(17), .AW(8), .DEPTH(256)) u_hist (
        .wclk(clk), .we(hist_we), .waddr(hist_waddr), .wdata(hist_wdata),
        .rclk(clk), .raddr(hist_raddr), .rdata(hist_q));

    // ---- Divider: scale = (255 << FRAC) / D -----------------------------------
    reg  [16:0] dvsr = 17'd0;
    reg  [27:0] quo = 28'd0;
    reg  [17:0] rem = 18'd0;
    reg  [4:0]  div_cnt = 5'd0;
    reg         ident = 1'b0;
    wire [17:0] rem_sh = {rem[16:0], quo[27]};
    wire        rem_ge = (rem_sh >= {1'b0, dvsr});

    // ---- LUT construction / application ---------------------------------------
    reg  [16:0] diff = 17'd0;
    reg  [44:0] prod = 45'd0;
    wire [44:0] prod_r  = prod + (45'd1 << (FRAC - 1));
    wire [24:0] lut_val = prod_r[44:FRAC];
    wire [7:0]  lut_new = ident ? v_cnt[7:0] : ((lut_val > 25'd255) ? 8'd255 : lut_val[7:0]);
    wire        lut_we  = (state == S_LUT) && (ph == 2'd3);
    wire [7:0]  lut_q;
    dp_ram #(.DW(8), .AW(8), .DEPTH(256)) u_lut (
        .wclk(clk), .we(lut_we), .waddr(v_cnt[7:0]), .wdata(lut_new),
        .rclk(clk), .raddr(in_rdata), .rdata(lut_q));

    // ROI used for equalization (may be the whole frame)
    reg  [8:0] roi_x0 = 9'd0, roi_x1 = IMG_W - 1;
    reg  [7:0] roi_y0 = 8'd0, roi_y1 = IMG_H - 1;
    wire       in_roi1 = (x1 >= roi_x0) && (x1 <= roi_x1) && (y1 >= roi_y0) && (y1 <= roi_y1);
    reg        roi2 = 1'b0;
    reg  [7:0] p2 = 8'd0, g2 = 8'd0;

    // ---- Output buffer writes ----------------------------------------------------
    assign orig_we    = v1 && ((state == S_LIVE) || (state == S_SNAPWAIT) ||
                               (state == S_DRAIN) || (state == S_ROWS));
    assign out_we     = (v1 && ((state == S_LIVE) || (state == S_SNAPWAIT) || (state == S_DRAIN))) ||
                        (v2 && (state == S_APPLY));
    assign wr_addr    = (state == S_APPLY) ? a2 : a1;
    assign orig_wdata = in_rdata;
    assign out_wdata  = (state == S_APPLY) ? (roi2 ? lut_q : (outside_gamma ? g2 : p2))
                                           : in_rdata;

    // ---- Helper: start a full-frame or rectangle scan --------------------------
    task start_scan;
        input [8:0] x_a, x_b;
        input [7:0] y_a, y_b;
        begin
            rx0 <= x_a; rx1 <= x_b; ry0 <= y_a; ry1 <= y_b;
            sx  <= x_a; sy  <= y_a;
            scan_on <= 1'b1;
        end
    endtask

    // ---- Main sequential logic ----------------------------------------------------
    // Box arithmetic helpers
    wire [8:0] y0_pad = (pfy_lo >= PAD_Y) ? (pfy_lo - PAD_Y) : 9'd0;
    wire [8:0] y1_pad = (pfy_hi + PAD_Y > IMG_H - 1) ? (IMG_H - 1) : (pfy_hi + PAD_Y);
    reg  [8:0] bl_res = 9'd0, bh_res = 9'd0;
    wire [8:0] bcen   = ({1'b0, pfx_lo} + {1'b0, pfx_hi}) >> 1;
    wire [8:0] ix_lo  = (bl_res > pfx_lo) ? bl_res : pfx_lo;   // max(lx, bl)
    wire [8:0] ix_hi  = (bh_res < pfx_hi) ? bh_res : pfx_hi;   // min(hx, bh)
    wire       ix_ok  = ix_lo < ix_hi;
    wire [8:0] fx_lo  = ix_ok ? ix_lo : pfx_lo;
    wire [8:0] fx_hi  = ix_ok ? ix_hi : pfx_hi;
    wire [8:0] x0_pad = (fx_lo >= PAD_X) ? (fx_lo - PAD_X) : 9'd0;
    wire [9:0] x1_sum = fx_hi + PAD_X;
    wire [8:0] x1_pad = (x1_sum > IMG_W - 1) ? (IMG_W - 1) : x1_sum[8:0];

    always @(posedge clk) begin
        pfy_start <= 1'b0;
        pfx_start <= 1'b0;

        // ---------------- pipeline (1 px / clock passes) -----------------
        v1 <= scan_on && (state != S_HIST) && ((state != S_GRAD) || (ph == 2'd0));
        g_wr   <= (state == S_GRAD) && cand;
        if (state == S_GRAD && cand) n_cand <= n_cand + 1'b1;
        gbin_d <= gx[9:2];
        x1 <= sx; y1 <= sy; a1 <= scan_addr;
        v2 <= v1;
        x2 <= x1; y2 <= y1; a2 <= a1;
        if (v1) begin
            sq0 <= s_col; sq1 <= sq0; sq2 <= sq1;
            c0  <= lb1_q; c1 <= c0;   c2  <= c1;
            roi2 <= in_roi1;
            p2   <= in_rdata;
            g2   <= gam;
        end
        if (state == S_ROWS && v2) begin
            if (x2 == IMG_W - 1) row_acc <= 9'd0;
            else                 row_acc <= row_acc + edge_raw;
            e_total <= e_total + edge_raw;
        end

        // scanner advance (1 px / clock, except HIST and GRAD which step themselves)
        if (scan_on && state != S_HIST && state != S_GRAD) begin
            if (scan_last) begin
                if ((state == S_LIVE || state == S_SNAPWAIT) && test_mode) begin
                    sx <= rx0; sy <= ry0;          // test image: keep looping
                end else begin
                    scan_on <= 1'b0;
                end
            end else if (sx == rx1) begin
                sx <= rx0; sy <= sy + 1'b1;
            end else begin
                sx <= sx + 1'b1;
            end
        end

        // ---------------- control FSM ------------------------------------
        case (state)
            S_LIVE: begin
                cap_enable <= 1'b1;
                // Live preview: copy buf_in -> buf_out once per COMPLETE camera
                // frame (copy takes 1.5 ms and stays ahead of the camera's next
                // frame), so the screen never shows two frames mixed (no tearing).
                if (frame_ev) live_pend <= 1'b1;
                if (!scan_on && (live_pend || frame_ev) && !test_mode) begin
                    live_pend <= 1'b0;
                    start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                end
                if (snap) begin
                    if (test_mode) begin
                        scan_on <= 1'b0;
                        state   <= S_DRAIN;
                    end else begin
                        state   <= S_SNAPWAIT;
                    end
                end
            end

            S_SNAPWAIT: begin            // wait until a complete frame is in buf_in
                if (go_live) begin
                    state <= S_LIVE;
                end else if (frame_ev) begin
                    cap_enable <= 1'b0;  // freeze buf_in (takes effect at next VSYNC)
                    scan_on    <= 1'b0;
                    state      <= S_DRAIN;
                end
            end

            S_DRAIN: if (pipe_empty) begin
                clr_cnt <= 9'd0;
                state   <= S_CLRB;
            end

            // ---- bright level: brightness histogram of the whole frame ------
            S_CLRB: begin
                if (clr_cnt == 9'd255) begin
                    bmode <= 1'b1;
                    ph    <= 2'd0;
                    start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                    state <= S_HIST;
                end
                clr_cnt <= clr_cnt + 1'b1;
            end

            // walk bins 0 -> 255: tb = first value whose cumulative count > RANK
            S_BWALK: begin
                if (ph == 2'd0) begin
                    ph <= 2'd1;
                end else begin
                    ph <= 2'd0;
                    if (g_sum > BRIGHT_RANK || v_cnt == 9'd255) begin
                        tb      <= gamma_detect ? gam_v : v_cnt[7:0];
                        clr_cnt <= 9'd0;
                        state   <= S_CLRG;
                    end else begin
                        g_acc <= g_sum[16:0];
                        v_cnt <= v_cnt + 1'b1;
                    end
                end
            end

            // ---- adaptive threshold: gradient histogram -------------------
            S_CLRG: begin
                if (clr_cnt == 9'd255) begin
                    if (edge_thr != 10'd0) begin            // fixed threshold
                        thr_used <= {1'b0, edge_thr};
                        row_acc  <= 9'd0;
                        e_total  <= 17'd0;
                        start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                        state    <= S_ROWS;
                    end else begin
                        ph     <= 2'd0;
                        n_cand <= 17'd0;
                        start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                        state <= S_GRAD;
                    end
                end
                clr_cnt <= clr_cnt + 1'b1;
            end

            // 3 clocks per pixel: ph0 address, ph1 v1, ph2 v2 (histogram read)
            S_GRAD: begin
                if (scan_on) begin
                    if (ph == 2'd2) begin
                        ph <= 2'd0;
                        if (scan_last) begin
                            scan_on <= 1'b0;
                        end else if (sx == rx1) begin
                            sx <= rx0; sy <= sy + 1'b1;
                        end else begin
                            sx <= sx + 1'b1;
                        end
                    end else begin
                        ph <= ph + 1'b1;
                    end
                end else if (pipe_empty && !g_wr) begin
                    v_cnt <= 9'd255;
                    g_acc <= 17'd0;
                    ph    <= 2'd0;
                    e_tgt <= ({8'd0, n_cand} * EDGE_MUL) >> EDGE_SH;
                    state <= S_THR;
                end
            end

            // walk bins 255 -> 0; stop at the first bin that would exceed the target
            S_THR: begin
                if (ph == 2'd0) begin
                    ph <= 2'd1;
                end else begin
                    ph <= 2'd0;
                    if (g_sum > e_tgt || v_cnt == 9'd0) begin
                        // thr = max(MIN_THR, (b+1)*4); b = -1 if nothing exceeded
                        if (g_sum > e_tgt && ({v_cnt + 1'b1, 2'b00} > MIN_THR))
                            thr_used <= {v_cnt + 1'b1, 2'b00};
                        else
                            thr_used <= MIN_THR;
                        row_acc <= 9'd0;
                        e_total <= 17'd0;
                        start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                        state   <= S_ROWS;
                    end else begin
                        g_acc <= g_sum[16:0];
                        v_cnt <= v_cnt - 1'b1;
                    end
                end
            end

            S_ROWS: if (pipe_empty) begin
                pfy_start <= 1'b1;
                state     <= S_FINDY;
            end

            S_FINDY: if (pfy_done) begin
                if ((pfy_peak >= MIN_PEAK_Y) &&
                    ({4'b0, pfy_peak} * FOUND_K >= e_total)) begin
                    plate_found <= 1'b1;
                    box_y0  <= y0_pad[7:0];
                    box_y1  <= y1_pad[7:0];
                    band_y0 <= y0_pad[7:0];
                    band_y1 <= y1_pad[7:0];
                    clr_cnt <= 9'd0;
                    state   <= S_CLRCOL;
                end else begin
                    plate_found <= 1'b0;
                    box_x0 <= 9'd0; box_x1 <= IMG_W - 1;
                    box_y0 <= 8'd0; box_y1 <= IMG_H - 1;
                    clr_cnt <= 9'd0;
                    state   <= S_CLRH;
                end
            end

            S_CLRCOL: begin
                if (clr_cnt == IMG_W - 1) begin
                    start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                    state <= S_COLS;
                end
                clr_cnt <= clr_cnt + 1'b1;
            end

            S_COLS: if (pipe_empty) begin
                pfx_start <= 1'b1;
                state     <= S_FINDX;
            end

            S_FINDX: if (pfx_done) begin
                bj    <= ({1'b0, pfx_lo} + {1'b0, pfx_hi}) >> 1;   // centre of the span
                blast <= ({1'b0, pfx_lo} + {1'b0, pfx_hi}) >> 1;
                bmiss <= 9'd0;
                state <= S_BL_RD;
            end

            // ---- bright run: grow left / right over columns >= 50% bright ----
            S_BL_RD: begin
                if (bj == 9'd0) begin
                    bl_res <= blast;
                    bj    <= bcen; blast <= bcen; bmiss <= 9'd0;
                    state <= S_BR_RD;
                end else begin
                    state <= S_BL_CK;
                end
            end

            S_BL_CK: begin
                if (col_bright) begin
                    bj <= bj - 1'b1; blast <= bj - 1'b1; bmiss <= 9'd0;
                    state <= S_BL_RD;
                end else if (bmiss < BGAP) begin
                    bj <= bj - 1'b1; bmiss <= bmiss + 1'b1;
                    state <= S_BL_RD;
                end else begin
                    bl_res <= blast;
                    bj    <= bcen; blast <= bcen; bmiss <= 9'd0;
                    state <= S_BR_RD;
                end
            end

            S_BR_RD: begin
                if (bj == IMG_W - 1) begin
                    bh_res <= blast;
                    state  <= S_BFIN;
                end else begin
                    state <= S_BR_CK;
                end
            end

            S_BR_CK: begin
                if (col_bright) begin
                    bj <= bj + 1'b1; blast <= bj + 1'b1; bmiss <= 9'd0;
                    state <= S_BR_RD;
                end else if (bmiss < BGAP) begin
                    bj <= bj + 1'b1; bmiss <= bmiss + 1'b1;
                    state <= S_BR_RD;
                end else begin
                    bh_res <= blast;
                    state  <= S_BFIN;
                end
            end

            S_BFIN: begin
                box_x0  <= x0_pad;
                box_x1  <= x1_pad;
                clr_cnt <= 9'd0;
                state   <= S_CLRH;
            end

            S_CLRH: begin
                if (clr_cnt == 9'd0) begin
                    if (plate_found && !force_global) begin
                        roi_x0 <= box_x0; roi_x1 <= box_x1;
                        roi_y0 <= box_y0; roi_y1 <= box_y1;
                    end else begin
                        roi_x0 <= 9'd0; roi_x1 <= IMG_W - 1;
                        roi_y0 <= 8'd0; roi_y1 <= IMG_H - 1;
                    end
                end
                if (clr_cnt == 9'd255) begin
                    npix  <= 17'd0;
                    ph    <= 2'd0;
                    bmode <= 1'b0;
                    // scan rectangle = ROI (roi_* are already valid one cycle after clr_cnt==0)
                    start_scan(roi_x0, roi_x1, roi_y0, roi_y1);
                    state <= S_HIST;
                end
                clr_cnt <= clr_cnt + 1'b1;
            end

            // 3 clocks per pixel: address -> read bin -> write bin+1 (no hazards)
            S_HIST: begin
                case (ph)
                    2'd0: ph <= 2'd1;
                    2'd1: begin p_h <= in_rdata; ph <= 2'd2; end
                    default: begin
                        npix <= npix + 1'b1;
                        ph   <= 2'd0;
                        if (scan_last) begin
                            scan_on   <= 1'b0;
                            v_cnt     <= 9'd0;
                            cdf       <= 17'd0;
                            min_found <= 1'b0;
                            cdf_min   <= 17'd0;
                            g_acc     <= 17'd0;
                            // contrast limit: CLIP_K * (N >> 8), at least 1
                            clip      <= (((npix + 1'b1) >> 8) == 0) ? 17'd1
                                         : (((npix + 1'b1) >> 8) * CLIP_K);
                            state     <= bmode ? S_BWALK : S_CDF;
                        end else if (sx == rx1) begin
                            sx <= rx0; sy <= sy + 1'b1;
                        end else begin
                            sx <= sx + 1'b1;
                        end
                    end
                endcase
            end

            S_CDF: begin
                if (ph == 2'd0) begin
                    ph <= 2'd1;
                end else begin
                    cdf <= cdf_new;
                    if (!min_found && cdf_new != 17'd0) begin
                        min_found <= 1'b1;
                        cdf_min   <= cdf_new;
                    end
                    ph <= 2'd0;
                    if (v_cnt == 9'd255) begin
                        state <= S_DIV;
                        dvsr  <= cdf_new - (min_found ? cdf_min : cdf_new);   // clipped total - cdf_min
                        quo   <= 28'd255 << FRAC;
                        rem   <= 18'd0;
                        div_cnt <= 5'd0;
                    end
                    v_cnt <= v_cnt + 1'b1;
                end
            end

            S_DIV: begin
                if (dvsr == 17'd0) begin
                    ident <= 1'b1;
                    v_cnt <= 9'd0; ph <= 2'd0;
                    state <= S_LUT;
                end else begin
                    ident <= 1'b0;
                    if (rem_ge) begin
                        rem <= rem_sh - {1'b0, dvsr};
                        quo <= {quo[26:0], 1'b1};
                    end else begin
                        rem <= rem_sh;
                        quo <= {quo[26:0], 1'b0};
                    end
                    div_cnt <= div_cnt + 1'b1;
                    if (div_cnt == 5'd27) begin
                        v_cnt <= 9'd0; ph <= 2'd0;
                        state <= S_LUT;
                    end
                end
            end

            // 4 clocks per entry: address -> diff -> multiply -> write
            S_LUT: begin
                case (ph)
                    2'd0: ph <= 2'd1;
                    2'd1: begin
                        diff <= (hist_q > cdf_min) ? (hist_q - cdf_min) : 17'd0;
                        ph   <= 2'd2;
                    end
                    2'd2: begin
                        prod <= diff * quo;
                        ph   <= 2'd3;
                    end
                    default: begin
                        ph <= 2'd0;
                        if (v_cnt == 9'd255) begin
                            start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                            state <= S_APPLY;
                        end
                        v_cnt <= v_cnt + 1'b1;
                    end
                endcase
            end

            S_APPLY: if (pipe_empty) state <= S_HOLD;

            S_HOLD: begin
                cap_enable <= 1'b0;
                if (go_live) begin
                    cap_enable <= 1'b1;
                    start_scan(9'd0, IMG_W - 1, 8'd0, IMG_H - 1);
                    state <= S_LIVE;
                end else if (reprocess) begin
                    clr_cnt <= 9'd0;
                    state   <= S_CLRB;
                end
            end

            default: state <= S_HOLD;
        endcase
    end

endmodule
