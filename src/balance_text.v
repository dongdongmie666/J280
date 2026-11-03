// Decimal ASCII telemetry with atomic snapshots. Fixed-point fields have
// three decimal places; no host-side binary decoding is required.
module balance_text #(
    parameter integer ZERO=535,
    parameter integer PID_PERIOD_MS=3,
    parameter integer COMPACT=0
) (
    input clk,rst_n,start, input [31:0] seq, input [9:0] raw,mean,
    input signed [31:0] error_code,position,encoder,
    input signed [15:0] applied_q8, input [7:0] limit_percent,
    input [15:0] kp_q8,kd_q8, input running, input [3:0] fault,
    input polarity_set,polarity_reverse, input [31:0] encoder_errors,
    input otr, input signed [31:0] close_code,move_counts,
    input [2:0] effect,
    input tx_busy, output reg tx_start, output reg [7:0] tx_data,
    output busy, input boost_enabled,
    input signed [31:0] rpm_milli, input speed_valid,
    input signed [15:0] target_q8, input [15:0] ki_q8,
    input position_loop_enabled,
    input signed [15:0] target_angle_code,target_offset_q8,
    input [15:0] position_kp_q12,position_kd_q8,
    input signed [19:0] diag_rate_q8,
    input signed [31:0] diag_p_q8,diag_d_q8,
    input [5:0] run_state,
    input [3:0] key_levels,
    input [15:0] key1_count,rx_count,swing_request_count,otr_count,
    input signed [31:0] diag_angle_pwm_q8,diag_i_q8,diag_position_pwm_q8,
    input [15:0] position_ki_q12,
    input signed [15:0] diag_target_request_q8,
    input diag_target_slew,
    input signed [31:0] position_target_counts,
    input position_ready,
    input signed [31:0] position_target_encoder,
    input signed [31:0] position_p_q8,position_d_raw_q8,position_d_limit_q8,
    input signed [31:0] position_i_q8,
    input signed [31:0] target_speed_mdeg_s,actual_speed_mdeg_s,
    input actual_speed_valid,target_speed_valid,
    input [3:0] stop_reason,
    input [9:0] stop_adc,
    input [31:0] stop_age,
    input [7:0] stop_flags,
    input signed [31:0] stop_encoder
);
    localparam IDLE=0,LABEL=1,PREP=2,CONVERT=3,SIGN=4,DIGITS=5,
               SPECIAL=6,SEP=7,LF=8,POINT=9;
    reg [3:0] state,digit;
    reg [5:0] field;
    reg [31:0] words[0:61];
    reg actual_speed_valid_snapshot,target_speed_valid_snapshot;
    reg signed [39:0] pos_i_a,pos_i_b,pos_i_scale;
    reg signed [31:0] pos_i_milli;
    reg signed [39:0] pos_p_scale,pos_d_raw_scale,pos_d_limit_scale;
    reg signed [39:0] pos_p_a,pos_p_b,pos_d_raw_a,pos_d_raw_b,pos_d_limit_a,pos_d_limit_b;
    reg signed [31:0] pos_p_milli,pos_d_raw_milli,pos_d_limit_milli;
    reg signed [39:0] angle_pwm_scale,i_scale,position_pwm_scale;
    reg signed [31:0] angle_pwm_delay,i_delay,position_pwm_delay;
    reg signed [31:0] angle_pwm_milli,i_milli,position_pwm_milli;
    reg rpm_valid_snapshot;
    reg signed [31:0] angle_milli_reg,out_milli_reg,target_milli_reg,target_angle_milli_reg;
    reg signed [31:0] out_scale_a,out_scale_b,out_scale_product;
    reg signed [31:0] target_scale_a,target_scale_b,target_scale_product;
    // Wide intermediates avoid overflow before the display-only division.
    // Q8 code difference -> deg/s, using the actual PID interval.
    reg signed [39:0] rate_scale_product,p_scale_product,d_scale_product;
    reg [39:0] rate_magnitude;
    reg [31:0] rate_quotient;
    reg rate_negative_a,rate_negative_b;
    reg signed [31:0] rate_milli_reg,p_milli_reg,d_milli_reg;
    wire signed [39:0] p_rounded=p_scale_product+(p_scale_product<0 ? 40'sd31 : 40'sd0);
    wire signed [39:0] d_rounded=d_scale_product+(d_scale_product<0 ? 40'sd31 : 40'sd0);
    reg [31:0] kp_milli_reg,kd_milli_reg,ki_milli_reg,pos_kp_milli_reg,pos_kd_milli_reg,pos_ki_micro_reg;
    reg [127:0] text_shift;
    reg [31:0] binary;
    reg [39:0] bcd,adjusted;
    reg [5:0] bit_index;
    reg negative,leading;
    wire fixed_field=field==3 || field==6 || field==8 || field==9 || field==20 || field==21 || field==22 || field==23 || field==25 || field==26 || field==27 || (field>=29 && field<=31) || (field>=39 && field<=43) || (field>=49 && field<=54);
    wire [3:0] fractional_digits=(field==26 || field==42) ? 4'd6 : 4'd3;
    wire signed_field=field==3 || field==4 || field==5 || field==6 || field==16 || field==17 || field==20 || field==21 || field==22 || field==25 || (field>=29 && field<=31) || (field>=39 && field<=41) || field==43 || field==45 || field==47 || field==48 || (field>=49 && field<=54) || field==61;
    assign busy=state!=IDLE;
    // CSV: SEQ,ANGLE,POS,ENC1,OUT,PWM_TARGET,TGT_ANGLE,ANG_RATE,
    // I_TERM,POS_KP,POS_KD,POS_TARGET,POS_READY,POS_ACT,POS_SET,
    // POS_P,POS_D_RAW,POS_D_LIM,POS_KI,POS_I,ANGLE_KP,ANGLE_KI,ANGLE_KD,
    // SPEED_SET,SPEED_ACT (signed degrees/second),ADC_RAW,ADC_MEAN (0..1023).
    // POS_I is the signed integral angle contribution in degrees, before
    // PI clamp, engagement ramp and slew limit; POS_KI has six decimals.
    // POS_ACT and POS_SET share the absolute ENC1 coordinate, in counts.
    function [5:0] next_field;
        input [5:0] f;
        begin
            if(!COMPACT) next_field=f+1'b1;
            else case(f)
                0:next_field=3; 3:next_field=4; 4:next_field=5;
                5:next_field=6; 6:next_field=20; 20:next_field=25;
                25:next_field=29; 29:next_field=40; 40:next_field=26;
                27:next_field=45; 45:next_field=46;
                46:next_field=47; 47:next_field=48;
                48:next_field=49; 49:next_field=50; 50:next_field=51;
                51:next_field=42; 42:next_field=52;
                52:next_field=8; 8:next_field=23; 23:next_field=9;
                9:next_field=53; 53:next_field=54; 54:next_field=1; 1:next_field=2;
                2:next_field=55;
                55,56,57,58,59,60:next_field=f+1'b1;
                default:next_field=27;
            endcase
        end
    endfunction
    function [127:0] label_text;
        input [5:0] f;
        begin
            case(f)
                0:label_text={"SEQ=",96'b0};
                1:label_text={"ADC=",96'b0};
                2:label_text={"MEAN=",88'b0};
                3:label_text={"ANGLE=",80'b0};
                4:label_text={"POS=",96'b0};
                5:label_text={"ENC1=",88'b0};
                6:label_text={"OUT=",96'b0};
                7:label_text={"LIMIT=",80'b0};
                8:label_text={"KP=",104'b0};
                9:label_text={"KD=",104'b0};
                10:label_text={"STATE=",80'b0};
                11:label_text={"REASON=",72'b0};
                12:label_text={"POL=",96'b0};
                13:label_text={"ERR1=",88'b0};
                14:label_text={"OTR=",96'b0};
                15:label_text={"ZERO=",88'b0};
                16:label_text={"CLOSE=",80'b0};
                17:label_text={"MOVE=",88'b0};
                18:label_text={"EFFECT=",72'b0};
                19:label_text={"BOOST=",80'b0};
                20:label_text={"PWM_TARGET=",40'b0};
                21:label_text={"PWM_ACT=",64'b0};
                22:label_text={"RPM_ACT=",64'b0};
                23:label_text={"KI=",104'b0};
                24:label_text={"TGT_ADC=",64'b0};
                25:label_text={"TGT_ANGLE=",48'b0};
                26:label_text={"POS_KP=",72'b0};
                27:label_text={"POS_KD=",72'b0};
                28:label_text={"POS_LOOP=",56'b0};
                29:label_text={"ANG_RATE=",56'b0};
                30:label_text={"P_TERM=",72'b0};
                31:label_text={"D_TERM=",72'b0};
                32:label_text={"RUN_STATE=",48'b0};
                33:label_text={"FW=",104'b0};
                34:label_text={"SW=",104'b0};
                35:label_text={"KEY_N=",80'b0};
                36:label_text={"RX_N=",88'b0};
                37:label_text={"W_N=",96'b0};
                38:label_text={"OTR_N=",80'b0};
                39:label_text={"ANGLE_PWM=",48'b0};
                40:label_text={"I_TERM=",72'b0};
                41:label_text={"POS_PWM=",64'b0};
                42:label_text={"POS_KI=",72'b0};
                43:label_text={"TGT_REQ=",64'b0};
                44:label_text={"TGT_SLEW=",56'b0};
                45:label_text={"POS_TARGET=",40'b0};
                46:label_text={"POS_READY=",48'b0};
                47:label_text={"POS_ACT=",64'b0};
                48:label_text={"POS_SET=",64'b0};
                49:label_text={"POS_P=",80'b0};
                50:label_text={"POS_D_RAW=",48'b0};
                51:label_text={"POS_D_LIM=",48'b0};
                52:label_text={"POS_I=",80'b0};
                53:label_text={"SPEED_SET=",48'b0};
                default:label_text={"SPEED_ACT=",48'b0};
            endcase
        end
    endfunction
    integer i,j;
    always @* begin
        adjusted=bcd;
        for(j=0;j<10;j=j+1)
            if(bcd[j*4 +: 4]>=5) adjusted[j*4 +: 4]=bcd[j*4 +: 4]+4'd3;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; field<=0; digit<=0; text_shift<=0; binary<=0;
            bcd<=0; bit_index<=0; negative<=0; leading<=1;
            tx_start<=0; tx_data<=0;
            rpm_valid_snapshot<=0;
            actual_speed_valid_snapshot<=0;target_speed_valid_snapshot<=0;
            angle_milli_reg<=0;out_milli_reg<=0;target_milli_reg<=0;target_angle_milli_reg<=0;
            out_scale_a<=0;out_scale_b<=0;out_scale_product<=0;
            target_scale_a<=0;target_scale_b<=0;target_scale_product<=0;
            rate_scale_product<=0;p_scale_product<=0;d_scale_product<=0;
            rate_magnitude<=0;rate_quotient<=0;rate_negative_a<=0;rate_negative_b<=0;
            rate_milli_reg<=0;p_milli_reg<=0;d_milli_reg<=0;
            kp_milli_reg<=0;kd_milli_reg<=0;ki_milli_reg<=0;
            pos_kp_milli_reg<=0;pos_kd_milli_reg<=0;pos_ki_micro_reg<=0;
            angle_pwm_scale<=0;i_scale<=0;position_pwm_scale<=0;
            angle_pwm_delay<=0;i_delay<=0;position_pwm_delay<=0;
            angle_pwm_milli<=0;i_milli<=0;position_pwm_milli<=0;
            pos_p_scale<=0;pos_d_raw_scale<=0;pos_d_limit_scale<=0;
            pos_p_a<=0;pos_p_b<=0;pos_d_raw_a<=0;pos_d_raw_b<=0;pos_d_limit_a<=0;pos_d_limit_b<=0;
            pos_p_milli<=0;pos_d_raw_milli<=0;pos_d_limit_milli<=0;
            pos_i_a<=0;pos_i_b<=0;pos_i_scale<=0;pos_i_milli<=0;
            for(i=0;i<62;i=i+1) words[i]<=0;
        end else begin
            tx_start<=0;
            // Identical display latency; signed Q8 degrees to milli-degrees.
            pos_i_a<=$signed({{8{position_i_q8[31]}},position_i_q8})<<<7;
            pos_i_b<=($signed({{8{position_i_q8[31]}},position_i_q8})<<<1)+position_i_q8;
            pos_i_scale<=pos_i_a-pos_i_b;
            pos_i_milli<=(pos_i_scale+(pos_i_scale<0 ? 40'sd31 : 40'sd0))>>>5;
            pos_p_a<=$signed({{8{position_p_q8[31]}},position_p_q8})<<<7;
            pos_p_b<=($signed({{8{position_p_q8[31]}},position_p_q8})<<<1)+position_p_q8;
            pos_d_raw_a<=$signed({{8{position_d_raw_q8[31]}},position_d_raw_q8})<<<7;
            pos_d_raw_b<=($signed({{8{position_d_raw_q8[31]}},position_d_raw_q8})<<<1)+position_d_raw_q8;
            pos_d_limit_a<=$signed({{8{position_d_limit_q8[31]}},position_d_limit_q8})<<<7;
            pos_d_limit_b<=($signed({{8{position_d_limit_q8[31]}},position_d_limit_q8})<<<1)+position_d_limit_q8;
            pos_p_scale<=pos_p_a-pos_p_b;
            pos_d_raw_scale<=pos_d_raw_a-pos_d_raw_b;
            pos_d_limit_scale<=pos_d_limit_a-pos_d_limit_b;
            pos_p_milli<=(pos_p_scale+(pos_p_scale<0 ? 40'sd31 : 40'sd0))>>>5;
            pos_d_raw_milli<=(pos_d_raw_scale+(pos_d_raw_scale<0 ? 40'sd31 : 40'sd0))>>>5;
            pos_d_limit_milli<=(pos_d_limit_scale+(pos_d_limit_scale<0 ? 40'sd31 : 40'sd0))>>>5;
            // All three components have identical display pipeline latency.
            angle_pwm_delay<=diag_angle_pwm_q8;i_delay<=diag_i_q8;position_pwm_delay<=diag_position_pwm_q8;
            angle_pwm_scale<=$signed(angle_pwm_delay)*40'sd125;
            i_scale<=$signed(i_delay)*40'sd125;
            position_pwm_scale<=$signed(position_pwm_delay)*40'sd125;
            angle_pwm_milli<=(angle_pwm_scale+(angle_pwm_scale<0 ? 40'sd31 : 40'sd0))>>>5;
            i_milli<=(i_scale+(i_scale<0 ? 40'sd31 : 40'sd0))>>>5;
            position_pwm_milli<=(position_pwm_scale+(position_pwm_scale<0 ? 40'sd31 : 40'sd0))>>>5;
            // Precompute display-only scaling outside the atomic snapshot mux.
            angle_milli_reg<=error_code*32'sd337;
            // Q8 percent to milli-percent is q8*125/32. Split the
            // shift/add implementation across registers for 50 MHz timing.
            out_scale_a<=$signed({{16{applied_q8[15]}},applied_q8})<<<7;
            out_scale_b<=($signed({{16{applied_q8[15]}},applied_q8})<<<1)+
                         $signed({{16{applied_q8[15]}},applied_q8});
            out_scale_product<=out_scale_a-out_scale_b;
            out_milli_reg<=(out_scale_product+(out_scale_product<0 ? 31 : 0))>>>5;
            target_scale_a<=$signed({{16{target_q8[15]}},target_q8})<<<7;
            target_scale_b<=($signed({{16{target_q8[15]}},target_q8})<<<1)+
                            $signed({{16{target_q8[15]}},target_q8});
            target_scale_product<=target_scale_a-target_scale_b;
            target_milli_reg<=(target_scale_product+(target_scale_product<0 ? 31 : 0))>>>5;
            target_angle_milli_reg<=($signed(target_offset_q8)*32'sd1000)/32'sd256;
            kp_milli_reg<=({16'b0,kp_q8}*32'd1000)>>8;
            kd_milli_reg<=({16'b0,kd_q8}*32'd1000)>>8;
            ki_milli_reg<=({16'b0,ki_q8}*32'd1000)>>8;
            // Q12 Kp displayed to six decimals; use 64 bits before scaling.
            pos_kp_milli_reg<=({48'b0,position_kp_q12}*64'd1000000)>>12;
            pos_kd_milli_reg<=({16'b0,position_kd_q8}*32'd1000)>>8;
            pos_ki_micro_reg<=({48'b0,position_ki_q12}*64'd1000000)>>12;
            rate_scale_product<=$signed(diag_rate_q8)*40'sd337000;
            p_scale_product<=$signed(diag_p_q8)*40'sd125;
            d_scale_product<=$signed(diag_d_q8)*40'sd125;
            // Pipeline magnitude, unsigned constant division and sign restore.
            // floor(floor(abs(x)/256)/T) == floor(abs(x)/(256*T)).
            // This preserves signed truncation without a long signed /768 path.
            rate_magnitude<=rate_scale_product[39] ? -rate_scale_product : rate_scale_product;
            rate_negative_a<=rate_scale_product[39];
            rate_quotient<=rate_magnitude[39:8]/PID_PERIOD_MS;
            rate_negative_b<=rate_negative_a;
            rate_milli_reg<=rate_negative_b ? -$signed(rate_quotient) : $signed(rate_quotient);
            // Physical ADC/gain bounds keep P/D milli-percent within signed
            // 32 bits, even before output clamping; upper bits are sign only.
            p_milli_reg<=p_rounded[36:5];
            d_milli_reg<=d_rounded[36:5];
            case(state)
                IDLE: if(start && !tx_busy && !tx_start) begin
                    words[0]<=seq; words[1]<={22'b0,raw}; words[2]<={22'b0,mean};
                    words[3]<=angle_milli_reg; words[4]<=position;
                    words[5]<=encoder; words[6]<=out_milli_reg;
                    words[7]<={24'b0,limit_percent};
                    words[8]<=kp_milli_reg;
                    words[9]<=kd_milli_reg;
                    words[10]<={31'b0,running}; words[11]<={28'b0,fault};
                    words[12]<=!polarity_set ? 0 : polarity_reverse ? 2 : 1;
                    words[13]<=encoder_errors; words[14]<={31'b0,otr}; words[15]<=ZERO;
                    words[16]<=running ? close_code : 0;
                    words[17]<=running ? move_counts : 0;
                    words[18]<=running ? {29'b0,effect} : 0;
                    words[19]<=boost_enabled ? 25 : 0;
                    words[20]<=target_milli_reg; words[21]<=out_milli_reg;
                    words[22]<=rpm_milli;rpm_valid_snapshot<=speed_valid;
                    words[23]<=ki_milli_reg;
                    words[24]<={{16{target_angle_code[15]}},target_angle_code};
                    words[25]<=target_angle_milli_reg;
                    words[26]<=pos_kp_milli_reg;
                    words[27]<=pos_kd_milli_reg;
                    words[28]<={31'b0,position_loop_enabled};
                    words[29]<=running ? rate_milli_reg : 0;
                    words[30]<=running ? p_milli_reg : 0;
                    words[31]<=running ? d_milli_reg : 0;
                    words[32]<={26'b0,run_state};
                    words[33]<=32'd102020;
                    words[34]<={28'b0,key_levels};
                    words[35]<={16'b0,key1_count};
                    words[36]<={16'b0,rx_count};
                    words[37]<={16'b0,swing_request_count};
                    words[38]<={16'b0,otr_count};
                    words[39]<=running ? angle_pwm_milli : 0;
                    words[40]<=running ? i_milli : 0;
                    words[41]<=running ? position_pwm_milli : 0;
                    words[42]<=pos_ki_micro_reg;
                    // Same edge snapshots request, applied angle and flag.
                    words[25]<=($signed(target_offset_q8)*32'sd1000)/32'sd256;
                    words[43]<=($signed(diag_target_request_q8)*32'sd1000)/32'sd256;
                    words[44]<={31'b0,diag_target_slew};
                    words[45]<=position_target_counts;
                    words[46]<={31'b0,position_ready};
                    words[47]<=encoder;
                    words[48]<=position_target_encoder;
                    words[49]<=pos_p_milli;
                    words[50]<=pos_d_raw_milli;
                    words[51]<=pos_d_limit_milli;
                    words[52]<=pos_i_milli;
                    words[53]<=target_speed_mdeg_s;
                    words[54]<=actual_speed_mdeg_s;
                    words[55]<={31'b0,running};words[56]<={28'b0,fault};
                    words[57]<={28'b0,stop_reason};words[58]<={22'b0,stop_adc};
                    words[59]<=stop_age;words[60]<={24'b0,stop_flags};words[61]<=stop_encoder;
                    actual_speed_valid_snapshot<=actual_speed_valid;
                    target_speed_valid_snapshot<=target_speed_valid;
                    field<=0; text_shift<=label_text(0); state<=COMPACT ? PREP : LABEL;
                end
                LABEL: if(text_shift==0) state<=PREP;
                    else if(!tx_busy && !tx_start) begin
                        tx_data<=text_shift[127:120]; tx_start<=1;
                        text_shift<={text_shift[119:0],8'b0};
                    end
                PREP: begin
                    if((field==22 && !rpm_valid_snapshot) ||
                       (field==53 && !target_speed_valid_snapshot) ||
                       (field==54 && !actual_speed_valid_snapshot)) begin
                        text_shift<={"N/A",104'b0};state<=SPECIAL;
                    end else if(field==10) begin
                        text_shift<=words[10]!=0 ? {"RUN",104'b0} : {"IDLE",96'b0};
                        state<=SPECIAL;
                    end else if(field==11 && !COMPACT) begin
                        case(words[11])
                            1:text_shift<={"SET_POLARITY",32'b0};
                            2:text_shift<={"HOLD_UPRIGHT",32'b0};
                            3:text_shift<={"ADC_OTR",72'b0};
                            4:text_shift<={"ADC_TIMEOUT",40'b0};
                            5:text_shift<={"TILT_LIMIT",48'b0};
                            6:text_shift<={"ARM_LIMIT",56'b0};
                            7:text_shift<={"ENC_ERROR",56'b0};
                            8:text_shift<={"LIMIT_LOW",56'b0};
                            9:text_shift<={"STOP_KEY",64'b0};
                            10:text_shift<={"ADC_WARMUP",48'b0};
                            11:text_shift<={"CONFIG_BUSY",40'b0};
                            default:text_shift<={"OK",112'b0};
                        endcase
                        state<=SPECIAL;
                    end else if(field==12) begin
                        case(words[12])
                            1:text_shift<={"PLUS",96'b0};
                            2:text_shift<={"MINUS",88'b0};
                            default:text_shift<={"UNSET",88'b0};
                        endcase
                        state<=SPECIAL;
                    end else if(field==18) begin
                        case(words[18])
                            1:text_shift<={"WAIT",96'b0};
                            2:text_shift<={"CENTER",80'b0};
                            3:text_shift<={"NO_MOTION",56'b0};
                            4:text_shift<={"RETURN",80'b0};
                            5:text_shift<={"AWAY",96'b0};
                            6:text_shift<={"FLAT",96'b0};
                            default:text_shift<={"IDLE",96'b0};
                        endcase
                        state<=SPECIAL;
                    end else if(field==28 && !COMPACT) begin
                        text_shift<=words[28]!=0 ? {"ON",112'b0} : {"OFF",104'b0};
                        state<=SPECIAL;
                    end else begin
                        negative<=signed_field && words[field][31];
                        binary<=(signed_field && words[field][31]) ? -words[field] : words[field];
                        bcd<=0; bit_index<=0; digit<=9; leading<=1; state<=CONVERT;
                    end
                end
                CONVERT: begin
                    bcd<={adjusted[38:0],binary[31]}; binary<={binary[30:0],1'b0};
                    if(bit_index==31) state<=SIGN; else bit_index<=bit_index+1'b1;
                end
                SIGN: if(!negative) state<=DIGITS;
                    else if(!tx_busy && !tx_start) begin tx_data<="-"; tx_start<=1; state<=DIGITS; end
                DIGITS: begin
                    if(leading && bcd[39:36]==0 && digit>(fixed_field ? fractional_digits : 0)) begin
                        bcd<={bcd[35:0],4'b0}; digit<=digit-1'b1;
                    end else if(!tx_busy && !tx_start) begin
                        tx_data<=8'h30+{4'b0,bcd[39:36]}; tx_start<=1; leading<=0;
                        bcd<={bcd[35:0],4'b0}; digit<=digit-1'b1;
                        if(fixed_field && digit==fractional_digits) state<=POINT;
                        else if(digit==0) begin
                            if(COMPACT) state<=SEP;
                            else if(field>=53) begin text_shift<={"deg/s",88'b0}; state<=SPECIAL; end
                            else if(field==3 || field==25 || field==43 || (field>=49 && field<=52)) begin text_shift<={"deg",104'b0}; state<=SPECIAL; end
                            else if(field==6 || field==7 || field==19 || field==20 || field==21 || field==30 || field==31 || (field>=39 && field<=41)) begin text_shift<={"%",120'b0}; state<=SPECIAL; end
                            else if(field==22) begin text_shift<={"rpm",104'b0}; state<=SPECIAL; end
                            else if(field==29) begin text_shift<={"deg/s",88'b0}; state<=SPECIAL; end
                            else state<=SEP;
                        end
                    end
                end
                POINT: if(!tx_busy && !tx_start) begin tx_data<="."; tx_start<=1; state<=DIGITS; end
                SPECIAL: if(text_shift==0) state<=SEP;
                    else if(!tx_busy && !tx_start) begin
                        tx_data<=text_shift[127:120]; tx_start<=1;
                        text_shift<={text_shift[119:0],8'b0};
                    end
                SEP: if(!tx_busy && !tx_start) begin
                    tx_start<=1;
                    if(COMPACT ? field==61 : field==54) begin tx_data<=13; state<=LF; end
                    else begin
                        tx_data<=COMPACT ? "," : " "; field<=next_field(field);
                        text_shift<=label_text(next_field(field)); state<=COMPACT ? PREP : LABEL;
                    end
                end
                LF: if(!tx_busy && !tx_start) begin tx_data<=10; tx_start<=1; state<=IDLE; end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
