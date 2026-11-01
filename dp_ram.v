// Generic simple dual-port RAM: one write port, one registered read port,
// independent clocks. Quartus infers M9K block RAM.
// Read latency: 1 clock (rdata valid the cycle after raddr is presented).
// INIT_FILE: optional $readmemh image (e.g. a test picture for buf_in).

module dp_ram #(
    parameter DW        = 8,
    parameter AW        = 17,
    parameter DEPTH     = 76800,
    parameter INIT_FILE = ""
) (
    input  wire          wclk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,

    input  wire          rclk,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);

    (* ramstyle = "M9K, no_rw_check" *) reg [DW-1:0] mem [0:DEPTH-1];

    initial begin
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    always @(posedge wclk) begin
        if (we) mem[waddr] <= wdata;
    end

    always @(posedge rclk) begin
        rdata <= mem[raddr];
    end

endmodule
