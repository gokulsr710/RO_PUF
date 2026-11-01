// =============================================================================
// Top level: low-light number-plate snapshot enhancement      (Terasic DE2-115)
//
//   OV7670 (YUV, Y only) --> buf_in 320x240x8 --> plate_enhance (50 MHz)
//                                                   |  edge detect, plate box,
//                                                   |  ROI histogram equalization
//                                                   v
//                              buf_orig (before) / buf_out (after) --> VGA
//
// Same module name and ports as before, so the Quartus project and pins.tcl
// work unchanged.
//
// Camera wiring on GPIO header (JP5): unchanged (see pin_connections.txt).
//
// Controls (all switches DOWN = recommended defaults)
//   KEY[0]  re-send camera configuration
//   KEY[1]  SNAPSHOT: grab the next full frame and process it
//   KEY[2]  back to LIVE preview
//   KEY[3]  re-process the held snapshot (after changing switches)
//   SW[0]   up = show original snapshot      down = show enhanced result
//   SW[1]   up = hide plate box              down = show green plate box
//   SW[2]   up = VGA test pattern
//   SW[3]   up = leave non-plate area raw    down = gamma-boost it
//   SW[4]   up = edge detection on raw Y     down = on gamma-boosted Y
//   SW[5]   up = equalize the whole frame (global HE, for comparison)
//   SW[17:14] all down = ADAPTIVE edge threshold (recommended)
//             otherwise fixed threshold = 16 * SW[17:14]  (16 .. 240)
//
// LEDs
//   LEDG[0] camera config done   LEDG[1] live preview   LEDG[2] processing
//   LEDG[3] result held          LEDG[4] plate found
//   LEDG[5] PCLK toggling  LEDG[6] HREF toggling  LEDG[7] VSYNC toggling
//   LEDG[8] data bus toggling
//
// Bench test without a camera: set USE_TEST_IMAGE = 1. buf_in is then loaded
// from TEST_IMAGE_FILE (made with tools/ref_model.py) and KEY[1] processes it.
// =============================================================================

module top_ov7670_vga #(
    parameter USE_TEST_IMAGE  = 0,
    parameter TEST_IMAGE_FILE = "test_image.hex",
    parameter Y_PHASE         = 0       // 1 if the camera outputs UYVY
) (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [17:0] SW,
    output wire [8:0]  LEDG,
    inout  wire [35:0] GPIO,

    output wire [7:0]  VGA_R,
    output wire [7:0]  VGA_G,
    output wire [7:0]  VGA_B,
    output wire        VGA_CLK,
    output wire        VGA_BLANK_N,
    output wire        VGA_SYNC_N,
    output wire        VGA_HS,
    output wire        VGA_VS
);

    // -------------------------------------------------------------------------
    // GPIO / camera pin map (unchanged)
    // -------------------------------------------------------------------------
    wire cam_sioc, cam_xclk, cam_reset, cam_pwdn, cfg_done;

    assign GPIO[0]     = cam_sioc;
    // GPIO[1] (SIOD) is driven by the i2c_sender inside ov7670_controller
    assign GPIO[2]     = 1'bz;      // VSYNC in
    assign GPIO[3]     = 1'bz;      // HREF in
    assign GPIO[4]     = 1'bz;      // PCLK in
    assign GPIO[5]     = cam_xclk;
    assign GPIO[13:6]  = 8'bz;      // D0..D7 in
    assign GPIO[14]    = cam_reset;
    assign GPIO[15]    = cam_pwdn;
    assign GPIO[35:16] = 20'bz;

    wire       cam_vsync = GPIO[2];
    wire       cam_href  = GPIO[3];
    wire       cam_pclk  = GPIO[4];
    wire [7:0] cam_d     = GPIO[13:6];

    // -------------------------------------------------------------------------
    // Clocks: 25 MHz VGA pixel clock from 50 MHz
    // -------------------------------------------------------------------------
    reg vga_clk_r = 1'b0;
    always @(posedge CLOCK_50) vga_clk_r <= ~vga_clk_r;
    wire vga_clk = vga_clk_r;

    // -------------------------------------------------------------------------
    // Keys (active low, already debounced on the DE2-115) -> 1-cycle pulses
    // -------------------------------------------------------------------------
    reg [2:0] k1 = 3'b111, k2 = 3'b111, k3 = 3'b111;
    reg [1:0] resend_sync = 2'b00;
    always @(posedge CLOCK_50) begin
        resend_sync <= {resend_sync[0], ~KEY[0]};
        k1 <= {k1[1:0], KEY[1]};
        k2 <= {k2[1:0], KEY[2]};
        k3 <= {k3[1:0], KEY[3]};
    end
    wire snap_p      = k1[2] & ~k1[1];   // falling edge = press
    wire live_p      = k2[2] & ~k2[1];
    wire reprocess_p = k3[2] & ~k3[1];

    // Switches into the 50 MHz domain
    reg [17:0] sw_s0 = 18'd0, sw_s = 18'd0;
    always @(posedge CLOCK_50) begin
        sw_s0 <= SW;
        sw_s  <= sw_s0;
    end

    // -------------------------------------------------------------------------
    // Camera configuration (50 MHz domain)
    // -------------------------------------------------------------------------
    ov7670_controller u_cam_ctrl (
        .clk             (CLOCK_50),
        .resend          (resend_sync[1]),
        .config_finished (cfg_done),
        .sioc            (cam_sioc),
        .siod            (GPIO[1]),
        .reset           (cam_reset),
        .pwdn            (cam_pwdn),
        .xclk            (cam_xclk)
    );

    // -------------------------------------------------------------------------
    // Capture: Y only, 320x240 (PCLK domain)
    // -------------------------------------------------------------------------
    wire        cap_enable;
    wire        cap_we;
    wire [16:0] cap_addr;
    wire [7:0]  cap_data;
    wire        frame_tog;
    wire        cap_busy;

    ov7670_capture_y #(.Y_PHASE(Y_PHASE)) u_capture (
        .pclk      (cam_pclk),
        .enable    (cap_enable),
        .vsync     (cam_vsync),
        .href      (cam_href),
        .d         (cam_d),
        .we        (cap_we),
        .waddr     (cap_addr),
        .wdata     (cap_data),
        .frame_tog (frame_tog),
        .busy      (cap_busy)
    );

    // -------------------------------------------------------------------------
    // Buffers
    // -------------------------------------------------------------------------
    localparam IN_INIT = USE_TEST_IMAGE ? TEST_IMAGE_FILE : "";

    wire [16:0] in_raddr;
    wire [7:0]  in_rdata;
    dp_ram #(.DW(8), .AW(17), .DEPTH(76800), .INIT_FILE(IN_INIT)) u_buf_in (
        .wclk(cam_pclk), .we(cap_we && (USE_TEST_IMAGE == 0)), .waddr(cap_addr), .wdata(cap_data),
        .rclk(CLOCK_50), .raddr(in_raddr), .rdata(in_rdata));

    wire        orig_we, out_we;
    wire [16:0] wr_addr;
    wire [7:0]  orig_wdata, out_wdata;
    wire [16:0] rd_addr;
    wire [7:0]  orig_q, out_q;

    dp_ram #(.DW(8), .AW(17), .DEPTH(76800)) u_buf_orig (
        .wclk(CLOCK_50), .we(orig_we), .waddr(wr_addr), .wdata(orig_wdata),
        .rclk(vga_clk),  .raddr(rd_addr), .rdata(orig_q));

    dp_ram #(.DW(8), .AW(17), .DEPTH(76800)) u_buf_out (
        .wclk(CLOCK_50), .we(out_we), .waddr(wr_addr), .wdata(out_wdata),
        .rclk(vga_clk),  .raddr(rd_addr), .rdata(out_q));

    // -------------------------------------------------------------------------
    // Plate localization + ROI histogram equalization (50 MHz)
    // -------------------------------------------------------------------------
    wire       st_live, st_busy, st_held, plate_found;
    wire [8:0] box_x0, box_x1;
    wire [7:0] box_y0, box_y1;

    plate_enhance u_proc (
        .clk           (CLOCK_50),
        .snap          (snap_p),
        .go_live       (live_p),
        .reprocess     (reprocess_p),
        .test_mode     (USE_TEST_IMAGE != 0),
        .frame_tog     (frame_tog),
        .cap_enable    (cap_enable),
        .edge_thr      ({2'b00, sw_s[17:14], 4'b0000}),   // 0 = adaptive
        .gamma_detect  (~sw_s[4]),
        .outside_gamma (~sw_s[3]),
        .force_global  (sw_s[5]),
        .in_raddr      (in_raddr),
        .in_rdata      (in_rdata),
        .orig_we       (orig_we),
        .out_we        (out_we),
        .wr_addr       (wr_addr),
        .orig_wdata    (orig_wdata),
        .out_wdata     (out_wdata),
        .live          (st_live),
        .busy          (st_busy),
        .held          (st_held),
        .plate_found   (plate_found),
        .box_x0        (box_x0),
        .box_x1        (box_x1),
        .box_y0        (box_y0),
        .box_y1        (box_y1)
    );

    // -------------------------------------------------------------------------
    // VGA (25 MHz domain)
    // -------------------------------------------------------------------------
    wire        vga_hs, vga_vs, vga_active;
    wire [11:0] test_pix;
    wire [8:0]  px_x;
    wire [7:0]  px_y;

    vga_timing u_vga (
        .clk      (vga_clk),
        .raddr    (rd_addr),
        .hs       (vga_hs),
        .vs       (vga_vs),
        .active   (vga_active),
        .test_pix (test_pix),
        .px_x     (px_x),
        .px_y     (px_y)
    );

    // Switches and box into the VGA domain (box changes only after processing)
    reg [1:0] orig_sel_s = 2'b00, hide_box_s = 2'b00, test_s = 2'b00, found_s = 2'b00;
    reg [8:0] bx0 = 9'd0, bx1 = 9'd0;
    reg [7:0] by0 = 8'd0, by1 = 8'd0;
    always @(posedge vga_clk) begin
        orig_sel_s <= {orig_sel_s[0], SW[0]};
        hide_box_s <= {hide_box_s[0], SW[1]};
        test_s     <= {test_s[0],     SW[2]};
        found_s    <= {found_s[0],    plate_found & st_held};
        bx0 <= box_x0; bx1 <= box_x1;
        by0 <= box_y0; by1 <= box_y1;
    end

    wire [7:0] grey   = orig_sel_s[1] ? orig_q : out_q;
    wire       on_box = found_s[1] && !hide_box_s[1] &&
                        ((((px_x == bx0) || (px_x == bx1)) && (px_y >= by0) && (px_y <= by1)) ||
                         (((px_y == by0) || (px_y == by1)) && (px_x >= bx0) && (px_x <= bx1)));

    reg [7:0] r_o, g_o, b_o;
    always @(*) begin
        if (!vga_active) begin
            r_o = 8'h00; g_o = 8'h00; b_o = 8'h00;
        end else if (test_s[1]) begin
            r_o = {test_pix[11:8], test_pix[11:8]};
            g_o = {test_pix[7:4],  test_pix[7:4]};
            b_o = {test_pix[3:0],  test_pix[3:0]};
        end else if (on_box) begin
            r_o = 8'h00; g_o = 8'hFF; b_o = 8'h00;
        end else begin
            r_o = grey; g_o = grey; b_o = grey;
        end
    end

    assign VGA_R       = r_o;
    assign VGA_G       = g_o;
    assign VGA_B       = b_o;
    assign VGA_CLK     = vga_clk;
    assign VGA_HS      = vga_hs;
    assign VGA_VS      = vga_vs;
    assign VGA_BLANK_N = vga_active;
    assign VGA_SYNC_N  = 1'b0;

    // -------------------------------------------------------------------------
    // Debug LEDs
    // -------------------------------------------------------------------------
    wire pclk_alive, href_alive, vsync_alive, data_alive;
    sig_activity u_act_pclk  (.clk(CLOCK_50), .sig(cam_pclk),  .alive(pclk_alive));
    sig_activity u_act_href  (.clk(CLOCK_50), .sig(cam_href),  .alive(href_alive));
    sig_activity u_act_vsync (.clk(CLOCK_50), .sig(cam_vsync), .alive(vsync_alive));
    sig_activity u_act_data  (.clk(CLOCK_50), .sig(^cam_d),    .alive(data_alive));

    assign LEDG = {data_alive, vsync_alive, href_alive, pclk_alive,
                   plate_found & st_held, st_held, st_busy, st_live, cfg_done};

endmodule
