// VGA 640x480 @ 60 Hz timing generator (25 MHz pixel clock).
// Reads a 320x240 frame buffer, each source pixel shown 2x2.
// raddr is registered; the frame buffer adds one more clock of read latency,
// so hs/vs/active/px_x/px_y are delayed 2 clocks to line up with the pixel data.
// CHANGED: added px_x / px_y (source-pixel coordinates) for the box overlay.

module vga_timing (
    input  wire        clk,     // 25 MHz
    output reg  [16:0] raddr,   // frame-buffer read address
    output reg         hs,      // active-low
    output reg         vs,      // active-low
    output reg         active,  // 1 = visible area (aligned with frame-buffer data)
    output reg  [11:0] test_pix,// colour bars + white border, aligned with frame-buffer data
    output reg  [8:0]  px_x,    // source x 0..319, aligned with frame-buffer data
    output reg  [7:0]  px_y     // source y 0..239, aligned with frame-buffer data
);

    localparam H_ACT = 640, H_FP = 16, H_SYNC = 96, H_BP = 48, H_TOT = 800;
    localparam V_ACT = 480, V_FP = 10, V_SYNC = 2,  V_BP = 33, V_TOT = 525;

    reg [9:0] h = 10'd0;
    reg [9:0] v = 10'd0;

    always @(posedge clk) begin
        if (h == H_TOT - 1) begin
            h <= 10'd0;
            if (v == V_TOT - 1) v <= 10'd0;
            else                v <= v + 1'b1;
        end else begin
            h <= h + 1'b1;
        end
    end

    wire act0 = (h < H_ACT) && (v < V_ACT);
    wire hs0  = ~((h >= H_ACT + H_FP) && (h < H_ACT + H_FP + H_SYNC));
    wire vs0  = ~((v >= V_ACT + V_FP) && (v < V_ACT + V_FP + V_SYNC));

    // 2x upscale: source x = h/2 (0..319), source y = v/2 (0..239)
    wire [8:0] xs = h[9:1];
    wire [7:0] ys = v[8:1];

    // address = ys*320 + xs = (ys<<8) + (ys<<6) + xs
    wire [16:0] addr_calc = {1'b0, ys, 8'b0} + {3'b0, ys, 6'b0} + {8'b0, xs};

    // Test pattern: 8 colour bars (80 px each) with a 1-pixel white border
    reg [11:0] bars;
    always @(*) begin
        case (h[9:7])
            3'd0:    bars = 12'hFFF;
            3'd1:    bars = 12'hFF0;
            3'd2:    bars = 12'h0FF;
            3'd3:    bars = 12'h0F0;
            3'd4:    bars = 12'hF0F;
            3'd5:    bars = 12'hF00;
            3'd6:    bars = 12'h00F;
            default: bars = 12'h444;
        endcase
    end
    wire border = (h == 10'd0) || (h == H_ACT - 1) || (v == 10'd0) || (v == V_ACT - 1);
    wire [11:0] tp0 = border ? 12'hFFF : bars;

    reg [11:0] tp1 = 12'h000;
    reg act1 = 1'b0, hs1 = 1'b1, vs1 = 1'b1;
    reg [8:0] xs1 = 9'd0;
    reg [7:0] ys1 = 8'd0;

    always @(posedge clk) begin
        // stage 1
        tp1   <= tp0;
        raddr <= act0 ? addr_calc : 17'd0;
        act1  <= act0;
        hs1   <= hs0;
        vs1   <= vs0;
        xs1   <= xs;
        ys1   <= ys;
        // stage 2 (matches frame-buffer read register)
        test_pix <= tp1;
        active <= act1;
        hs     <= hs1;
        vs     <= vs1;
        px_x   <= xs1;
        px_y   <= ys1;
    end

endmodule
