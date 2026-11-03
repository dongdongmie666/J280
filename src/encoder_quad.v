module encoder_quad (
    input clk, input rst_n, input clear, input a, input b,
    output reg signed [31:0] count,
    output reg [31:0] errors,
    output [1:0] ab
);
    reg [1:0] a_sync, b_sync, previous;
    reg [3:0] warmup;
    assign ab = {a_sync[1], b_sync[1]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 0; b_sync <= 0; previous <= 0;
            count <= 0; errors <= 0; warmup <= 0;
        end else begin
            a_sync <= {a_sync[0], a};
            b_sync <= {b_sync[0], b};
            previous <= ab;
            if (warmup != 15) warmup <= warmup + 1'b1;
            if (clear || warmup != 15) begin
                count <= 0; errors <= 0;
            end else begin
                case ({previous, ab})
                    4'b0010,4'b1011,4'b1101,4'b0100: count <= count + 1'b1;
                    4'b0001,4'b0111,4'b1110,4'b1000: count <= count - 1'b1;
                    4'b0011,4'b1100,4'b0110,4'b1001: errors <= errors + 1'b1;
                    default: begin end
                endcase
            end
        end
    end
endmodule
