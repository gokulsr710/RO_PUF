// Debug helper: "alive" = 1 if the (asynchronous) input toggled at least once
// during the last ~84 ms window. Runs on the 50 MHz clock, so it works even
// when the camera's PCLK is missing.

module sig_activity (
    input  wire clk,        // 50 MHz
    input  wire sig,        // asynchronous signal to watch
    output reg  alive = 1'b0
);

    reg [2:0]  s       = 3'b000;
    reg [21:0] cnt     = 22'd0;
    reg        toggled = 1'b0;

    always @(posedge clk) begin
        s   <= {s[1:0], sig};
        cnt <= cnt + 1'b1;

        if (s[2] ^ s[1]) toggled <= 1'b1;

        if (cnt == 22'd0) begin
            alive   <= toggled;
            toggled <= 1'b0;
        end
    end

endmodule
