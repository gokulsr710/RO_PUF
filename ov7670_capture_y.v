// =============================================================================
// OV7670 capture, greyscale (Y) version.
//
// Camera is set to VGA 640x480 YUV422 (see ov7670_registers.v). Each line is
// 1280 bytes:  Y0 U0 Y1 V0 Y2 U2 Y3 V2 ...
// We keep every 2nd Y sample (byte 0, 4, 8, ...) and every 2nd line, giving a
// 320x240 8-bit luminance image written in raster order to address 0..76799.
//
// ROBUST ADDRESSING: the write address is computed from the line number and
// the position inside the line (row*320 + col), not from a running counter.
// A missed or extra PCLK edge (noisy jumper wires) can then only damage the
// line where it happens; it can no longer shift every line below it sideways.
//
// enable     : level from the 50 MHz domain. Sampled during VSYNC, so a frame
//              is either written completely or not at all.
// frame_tog  : toggles at the start of VSYNC after every frame that was written.
//              The 50 MHz side uses it to know "buffer now holds a full frame".
// Y_PHASE    : 0 for YUYV byte order (default, TSLB=0x04/COM13[0]=0).
//              Set to 1 if your module outputs UYVY (picture looks flat grey).
// =============================================================================

module ov7670_capture_y #(
    parameter Y_PHASE = 0
) (
    input  wire        pclk,
    input  wire        enable,
    input  wire        vsync,
    input  wire        href,
    input  wire [7:0]  d,
    output reg         we        = 1'b0,
    output reg  [16:0] waddr     = 17'd0,
    output reg  [7:0]  wdata     = 8'd0,
    output reg         frame_tog = 1'b0,
    output wire        busy
);

    // Inputs latched on the falling edge (same as the original, working design)
    reg       vsync_l = 1'b0, href_l = 1'b0;
    reg [7:0] d_l     = 8'd0;
    always @(negedge pclk) begin
        vsync_l <= vsync;
        href_l  <= href;
        d_l     <= d;
    end

    reg [1:0]  en_s      = 2'b00;
    reg        vs_prev   = 1'b0;
    reg        href_prev = 1'b0;
    reg        writing   = 1'b0;
    reg        wrote_any = 1'b0;
    reg [10:0] byte_cnt  = 11'd0;
    reg [8:0]  line_cnt  = 9'd0;

    wire [7:0]  row      = line_cnt[8:1];          // 0..239
    wire [8:0]  col      = byte_cnt[10:2];         // 0..319
    wire [16:0] row_base = {1'b0, row, 8'b0} + {3'b0, row, 6'b0};   // row*320

    assign busy = writing;

    always @(posedge pclk) begin
        en_s    <= {en_s[0], enable};
        vs_prev <= vsync_l;
        we      <= 1'b0;

        if (vsync_l) begin
            // vertical blanking: end of the previous frame
            if (!vs_prev && writing && wrote_any) frame_tog <= ~frame_tog;
            writing   <= en_s[1];
            wrote_any <= 1'b0;
            byte_cnt  <= 11'd0;
            line_cnt  <= 9'd0;
            href_prev <= 1'b0;
        end else begin
            href_prev <= href_l;
            if (href_l) byte_cnt <= byte_cnt + 1'b1;
            else        byte_cnt <= 11'd0;
            if (href_prev && !href_l) line_cnt <= line_cnt + 1'b1;

            if (writing && href_l && !line_cnt[0] &&
                (byte_cnt[1:0] == Y_PHASE) && (byte_cnt < 11'd1280) &&
                (line_cnt < 9'd480)) begin
                we        <= 1'b1;
                waddr     <= row_base + {8'b0, col};
                wdata     <= d_l;
                wrote_any <= 1'b1;
            end
        end
    end

endmodule
