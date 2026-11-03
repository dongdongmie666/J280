
// Official Interface_test_new cadence at 50 MHz: 25 MHz ADC clock,
// one selected conversion per 9375 clocks (187.5 us), 16-point mean / 3 ms.
// Keep RAW 10-bit units (official theta_out is raw * 4), so ZERO stays 535.
module adc_measure (
    input clk,rst_n, input [9:0] data_in, input otr_in,
    output reg adc_clk, output oe_n,
    output reg [9:0] raw,mean,
    output reg valid, output reg otr
);
    reg [13:0] select_count;
    reg [3:0] samples;
    reg [13:0] sum;
    reg otr_window;
    reg [9:0] warmup;
    wire [13:0] total=sum+{4'b0,raw};
    assign oe_n=0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            select_count<=0; samples<=0; sum<=0; otr_window<=0;
            warmup<=0; adc_clk<=0; raw<=0; mean<=0; valid<=0; otr<=0;
        end else begin
            valid<=0;
            adc_clk<=~adc_clk;
            // Capture the previous conversion before the next ADC rising edge.
            if(!adc_clk) raw<=data_in;
            if(warmup!=1023) warmup<=warmup+1'b1;
            else begin
                otr_window<=otr_window|otr_in;
                if(select_count==9374) begin
                    select_count<=0;
                    if(samples==15) begin
                        mean<=total[13:4]; valid<=1; otr<=otr_window|otr_in;
                        sum<=0; samples<=0; otr_window<=0;
                    end else begin sum<=total; samples<=samples+1'b1; end
                end else select_count<=select_count+1'b1;
            end
        end
    end
endmodule
