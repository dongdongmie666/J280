// Output-shaft signed RPM * 1000, averaged over independent 50 ms windows.
// Count direction follows ENC1. Division is sequential to preserve clock timing.
module encoder_speed #(
    parameter integer CLK_HZ=50000000,
    parameter integer COUNTS_PER_REV=1040
) (
    input clk,rst_n,clear,
    input signed [31:0] encoder,
    input [31:0] encoder_errors,
    output reg signed [31:0] rpm_milli,
    output reg speed_valid
);
    localparam integer WINDOW_CYCLES=CLK_HZ/20;
    reg [31:0] timer,previous_count,previous_errors;
    reg seed_pending,multiply_pending,dividing,negative;
    reg [31:0] magnitude_seed;
    reg [63:0] dividend,quotient;
    reg [32:0] remainder;
    reg [5:0] bit_index;
    // 32-bit subtraction intentionally handles signed encoder counter wrap.
    wire signed [31:0] delta=encoder-previous_count;
    wire [31:0] magnitude=delta[31] ? -delta : delta;
    wire [32:0] trial={remainder[31:0],dividend[63]};
    wire subtract=trial>=COUNTS_PER_REV;
    wire [63:0] next_quotient={quotient[62:0],subtract};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            timer<=0;previous_count<=0;previous_errors<=0;seed_pending<=1;
            multiply_pending<=0;magnitude_seed<=0;
            dividing<=0;negative<=0;dividend<=0;quotient<=0;remainder<=0;bit_index<=0;
            rpm_milli<=0;speed_valid<=0;
        end else if(clear) begin
            timer<=0;seed_pending<=1;multiply_pending<=0;dividing<=0;rpm_milli<=0;speed_valid<=0;
        end else if(seed_pending) begin
            previous_count<=encoder;previous_errors<=encoder_errors;
            timer<=0;seed_pending<=0;
        end else begin
            if(timer==WINDOW_CYCLES-1) begin
                timer<=0;previous_count<=encoder;previous_errors<=encoder_errors;
                if(encoder_errors!=previous_errors) begin
                    speed_valid<=0;rpm_milli<=0;multiply_pending<=0;dividing<=0;
                end else begin
                    // Register the signed window delta before multiplication;
                    // this removes the encoder-to-multiplier combinational path.
                    magnitude_seed<=magnitude;
                    negative<=delta[31];multiply_pending<=1;
                end
            end else timer<=timer+1'b1;
            if(multiply_pending) begin
                // delta / 1040 / 0.050 * 60 * 1000
                dividend<=magnitude_seed*64'd1200000;
                quotient<=0;remainder<=0;bit_index<=63;
                multiply_pending<=0;dividing<=1;
            end else if(dividing) begin
                dividend<={dividend[62:0],1'b0};
                remainder<=subtract ? trial-COUNTS_PER_REV : trial;
                quotient<=next_quotient;
                if(bit_index==0) begin
                    dividing<=0;speed_valid<=1;
                    if(next_quotient>64'd2147483647)
                        rpm_milli<=negative ? -32'sd2147483647 : 32'sd2147483647;
                    else rpm_milli<=negative ? -$signed(next_quotient[31:0]) : $signed(next_quotient[31:0]);
                end else bit_index<=bit_index-1'b1;
            end
        end
    end
endmodule

// Display-only conversion: Q16 encoder counts per outer tick -> milli-deg/s.
// Sequential division keeps the large constant conversion out of control timing.
module velocity_units #(
    parameter integer COUNTS_PER_REV=1040,
    parameter integer PERIOD_MS=10
)(
    input clk,rst_n,
    input signed [31:0] velocity_q16,
    output reg signed [31:0] mdeg_s,
    output reg valid
);
    localparam [63:0] DENOMINATOR=64'd65536*COUNTS_PER_REV*PERIOD_MS;
    reg [1:0] state;
    reg signed [31:0] captured;
    reg negative;
    (* syn_preserve=1, syn_keep=1 *) reg [31:0] magnitude;
    (* syn_preserve=1, syn_keep=1 *) reg [63:0] dividend;
    reg [63:0] quotient,remainder;
    reg [5:0] bit_index;
    wire [63:0] trial={remainder[62:0],dividend[63]};
    wire subtract=trial>=DENOMINATOR;
    wire [63:0] next_quotient={quotient[62:0],subtract};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=0;captured<=0;negative<=0;magnitude<=0;dividend<=0;
            quotient<=0;remainder<=0;bit_index<=0;mdeg_s<=0;valid<=0;
        end else case(state)
            0: if(!valid || velocity_q16!=captured) begin
                captured<=velocity_q16;negative<=velocity_q16[31];
                magnitude<=velocity_q16[31] ? -velocity_q16 : velocity_q16;state<=1;
            end
            1: begin
                dividend<=magnitude*64'd360000000;quotient<=0;remainder<=0;
                bit_index<=63;state<=2;
            end
            2: begin
                dividend<={dividend[62:0],1'b0};
                remainder<=subtract ? trial-DENOMINATOR : trial;
                quotient<=next_quotient;
                if(bit_index==0) begin
                    mdeg_s<=next_quotient>64'd2147483647 ?
                        (negative ? -32'sh7fffffff : 32'sh7fffffff) :
                        (negative ? -$signed(next_quotient[31:0]) : $signed(next_quotient[31:0]));
                    valid<=1;state<=0;
                end else bit_index<=bit_index-1'b1;
            end
            default:state<=0;
        endcase
    end
endmodule
