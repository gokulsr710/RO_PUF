module i2c_sender (
    input  wire       clk,
    inout  wire       siod,
    output reg        sioc,
    output reg        taken,
    input  wire       send,
    input  wire [7:0] id,
    input  wire [7:0] reg_addr,
    input  wire [7:0] value
);

    reg [7:0]  divider;
    reg [31:0] busy_sr;
    reg [31:0] data_sr;

    initial begin
        divider = 8'b00000001;
        busy_sr = 32'b0;
        data_sr = 32'b11111111111111111111111111111111;
        sioc    = 1'b1;
        taken   = 1'b0;
    end

    assign siod =
        ((busy_sr[11:10] == 2'b10) ||
         (busy_sr[20:19] == 2'b10) ||
         (busy_sr[29:28] == 2'b10))
        ? 1'bz
        : data_sr[31];

    always @(posedge clk) begin

        taken <= 1'b0;

        if (busy_sr[31] == 1'b0) begin

            sioc <= 1'b1;

            if (send == 1'b1) begin

                if (divider == 8'b00000000) begin

                    data_sr <= {
                        3'b100,
                        id,
                        1'b0,
                        reg_addr,
                        1'b0,
                        value,
                        1'b0,
                        2'b01
                    };

                    busy_sr <= {
                        3'b111,
                        9'b111111111,
                        9'b111111111,
                        9'b111111111,
                        2'b11
                    };

                    taken <= 1'b1;

                end
                else begin
                    divider <= divider + 1'b1;
                end

            end

        end
        else begin

            case ({busy_sr[31:29], busy_sr[2:0]})

                6'b111111: begin
                    case (divider[7:6])
                        2'b00: sioc <= 1'b1;
                        2'b01: sioc <= 1'b1;
                        2'b10: sioc <= 1'b1;
                        default: sioc <= 1'b1;
                    endcase
                end

                6'b111110: begin
                    case (divider[7:6])
                        2'b00: sioc <= 1'b1;
                        2'b01: sioc <= 1'b1;
                        2'b10: sioc <= 1'b1;
                        default: sioc <= 1'b1;
                    endcase
                end

                6'b111100: begin
                    case (divider[7:6])
                        2'b00: sioc <= 1'b0;
                        2'b01: sioc <= 1'b0;
                        2'b10: sioc <= 1'b0;
                        default: sioc <= 1'b0;
                    endcase
                end

                6'b110000: begin
                    case (divider[7:6])
                        2'b00: sioc <= 1'b0;
                        2'b01: sioc <= 1'b1;
                        2'b10: sioc <= 1'b1;
                        default: sioc <= 1'b1;
                    endcase
                end

                6'b100000: begin
                    case (divider[7:6])
                        2'b00: sioc <= 1'b1;
                        2'b01: sioc <= 1'b1;
                        2'b10: sioc <= 1'b1;
                        default: sioc <= 1'b1;
                    endcase
                end

                6'b000000: begin
                    case (divider[7:6])
                        2'b00: sioc <= 1'b1;
                        2'b01: sioc <= 1'b1;
                        2'b10: sioc <= 1'b1;
                        default: sioc <= 1'b1;
                    endcase
                end

                default: begin
                    case (divider[7:6])
                        2'b00: sioc <= 1'b0;
                        2'b01: sioc <= 1'b1;
                        2'b10: sioc <= 1'b1;
                        default: sioc <= 1'b0;
                    endcase
                end

            endcase

            if (divider == 8'b11111111) begin

                busy_sr <= {busy_sr[30:0], 1'b0};
                data_sr <= {data_sr[30:0], 1'b1};
                divider <= 8'b0;

            end
            else begin
                divider <= divider + 1'b1;
            end

        end

    end

endmodule
