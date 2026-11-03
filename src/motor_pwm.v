// Signed command is percent * 256. Latch at each 20 kHz PWM boundary.
// A direction change inserts one complete zero-output PWM period.
module motor_pwm #(parameter integer CLK_HZ=50000000) (
    input clk,rst_n,enable,stop_key_n,
    input signed [15:0] command_q8,
    output in1,in2,pwm,
    output reg signed [15:0] applied_q8
);
    localparam integer PERIOD=CLK_HZ/20000;
    reg [15:0] counter;
    localparam signed [15:0] MAX_DUTY_Q8=16'sd23040; // 90%; existing hardware cap unchanged
    wire signed [15:0] limited=command_q8>MAX_DUTY_Q8 ? MAX_DUTY_Q8 :
                                  command_q8< -MAX_DUTY_Q8 ? -MAX_DUTY_Q8 : command_q8;
    wire [15:0] magnitude=applied_q8[15] ? -applied_q8 : applied_q8;
    wire [31:0] on_cycles=(magnitude*(PERIOD/100))>>8;
    wire gate=enable && rst_n && stop_key_n;
    assign in1=gate && applied_q8>0;
    assign in2=gate && applied_q8<0;
    assign pwm=gate && counter<on_cycles;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin counter<=0; applied_q8<=0; end
        else if(!gate) begin counter<=0; applied_q8<=0; end
        else if(counter==PERIOD-1) begin
            counter<=0;
            if(applied_q8!=0 && limited!=0 && applied_q8[15]!=limited[15]) applied_q8<=0;
            else applied_q8<=limited;
        end else counter<=counter+1'b1;
    end
endmodule
