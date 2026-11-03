// Angle-domain discrete positional PID, once per fresh 3 ms ADC mean.
// e is degrees, Ki includes T, Kd includes 1/T. D uses measured angle only.
module balance_control #(
    parameter integer CLK_HZ=50000000,  
    parameter integer ZERO=535,
    parameter integer COUNTS_PER_REV=1040,
    parameter integer PID_PERIOD_MS=3,
    parameter integer POSITION_PERIOD_MS=10,
    parameter integer POSITION_MOVE_SAMPLE_TICKS=2, // 0: bypass; default move = 5.12 s
    parameter integer SINE_MOVE_SAMPLE_TICKS=4, // DIP1 half-cycle 10.24 s, full cycle 20.48 s.
    parameter integer TRIANGLE_MOVE_SAMPLE_TICKS=6, // DIP2 rise: 15.36 s.
    parameter integer SAW_RETURN_SAMPLE_TICKS=4, // DIP2 return: 10.24 s.
    parameter integer SAW_HOLD_MS=2000, // Hold at each endpoint.
    parameter integer PROFILE_SPEED_MDEG_S=10000, // Legacy, unused by shared UART/DIP planner.
    parameter integer PROFILE_RAMP_MS=2000, // Legacy, unused by shared UART/DIP planner.
    parameter integer PROFILE_HOLD_MS=3000,
    parameter integer PROFILE_ARRIVE_COUNTS=57, // About +/-10.04 degrees at 1040 counts/rev.
    parameter integer PROFILE_SETTLE_MS=500,
    parameter integer PROFILE_RELEASE_COUNTS=57, // Dwell hysteresis, about +/-14.88 degrees.
    parameter integer PROFILE_SETTLE_SPEED_COUNTS=20, // Per 100ms, qualification only.
    parameter integer POSITION_RAMP_MS=500,
    parameter integer TARGET_RATE_DEG_S=40,
    parameter integer POSITION_RATE_WINDOW_MS=30,
    parameter integer POSITION_ENGAGE_RATE_DEG_S=80,
    parameter integer ENCODER_FAULT_PROTECTION=0,
    parameter [15:0] POSITION_KI_Q12=16'd0,
    parameter signed [31:0] POSITION_ANGLE_LIMIT_Q8=32'sd12800, // +/-5 degrees
    // Freeze (retain, do not clear) angle I inside this error band.
    // 350 mdeg covers one nominal ADC code (0.337005615 degrees).
    parameter integer ANGLE_I_DEADBAND_MDEG=750,
    // Absolute PWM percentage points contributed by I, independent of P/D.
    parameter integer ANGLE_I_LIMIT_PERCENT=5,
    parameter integer POSITION_VELOCITY_WINDOW_MS=30,
    // Direct velocity braking is bounded independently of the slow P target.
    parameter integer POSITION_D_LIMIT_DEG=4
) (
    input clk, rst_n, stop_key_n,
    input sample_valid, input [9:0] adc_mean, input sample_otr,
    input signed [31:0] encoder, input [31:0] encoder_errors,
    input cmd_valid, input [7:0] cmd,
    output reg running, output reg [3:0] fault,
    output reg polarity_set, output reg polarity_reverse,
    output reg [7:0] limit_percent,
    output reg [15:0] kp_q8, kd_q8,
    output reg signed [31:0] error_code, delta_code, position,
    output reg signed [15:0] output_q8,
    output reg signed [31:0] close_code, move_counts,
    output reg [2:0] effect,
    output reg boost_enabled,
    output reg [15:0] ki_q8,
    output reg position_loop_enabled,
    output reg [15:0] position_kp_q12, position_kd_q8,
    output reg signed [15:0] target_angle_code, target_offset_q8,
    output reg signed [19:0] diag_rate_q8,
    output reg signed [31:0] diag_p_q8, diag_d_q8,
    output [5:0] run_state,
    output reg signed [31:0] diag_angle_pwm_q8, diag_i_q8, diag_position_pwm_q8,
    output reg [15:0] position_ki_q12,
    output signed [15:0] diag_target_request_q8,
    output diag_target_slew,
    output reg signed [31:0] position_target_counts,
    output position_ready,
    output signed [31:0] position_target_encoder,
    output signed [31:0] diag_position_p_q8,diag_position_d_raw_q8,diag_position_d_limit_q8,
    output signed [31:0] diag_position_i_q8,
    input sine_enable,
    output signed [31:0] target_velocity_q16,
    input curve2_enable,
    output reg [3:0] stop_reason,
    output reg [9:0] stop_adc,
    output reg [31:0] stop_age,
    output reg [7:0] stop_flags,
    output reg signed [31:0] stop_encoder
);
    localparam integer STALE_CYCLES=CLK_HZ/100; // 10 ms
    localparam integer POSITION_CYCLES=CLK_HZ/1000*POSITION_PERIOD_MS;
    reg [31:0] position_timer;
    reg [8:0] position_valid;
    reg position_engaged;
    reg [2:0] position_stable_samples;
    localparam integer POSITION_ENGAGE_SAMPLES=5;
    // floor(5 degrees / 0.337005615 degrees per ADC code).
    localparam integer POSITION_ENGAGE_CODES=14;
    localparam integer RATE_SAMPLES=(POSITION_RATE_WINDOW_MS+PID_PERIOD_MS-1)/PID_PERIOD_MS;
    localparam integer RATE_TRAVEL_CODES=POSITION_ENGAGE_RATE_DEG_S*RATE_SAMPLES*PID_PERIOD_MS/337;
    localparam integer TARGET_STEP_Q8=(TARGET_RATE_DEG_S*256*PID_PERIOD_MS/1000)<1 ? 1 :
        TARGET_RATE_DEG_S*256*PID_PERIOD_MS/1000;
    reg signed [15:0] target_request_q8;
    assign diag_target_request_q8=target_request_q8;
    localparam integer VELOCITY_SAMPLES_RAW=(POSITION_VELOCITY_WINDOW_MS+POSITION_PERIOD_MS-1)/POSITION_PERIOD_MS;
    localparam integer VELOCITY_SAMPLES=VELOCITY_SAMPLES_RAW<1 ? 1 : VELOCITY_SAMPLES_RAW;
    reg signed [31:0] velocity_history[0:VELOCITY_SAMPLES-1];
    reg signed [31:0] command_history[0:VELOCITY_SAMPLES-1];
    reg sine_active;
    reg sine_active_previous;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) sine_active_previous<=0;
        else sine_active_previous<=sine_active;
    integer velocity_index;
    reg signed [32:0] position_d_product_reg;
    // Split the constant window division at a register boundary. A single
    // signed 33-bit /3 path did not meet the board's 50 MHz clock.
    reg [32:0] position_d_magnitude;
    reg position_d_negative_a,position_d_negative_b,position_d_negative_c;
    reg [16:0] position_d_upper_quotient,position_d_upper_delay;
    localparam integer POSITION_D_REMAINDER_BITS=VELOCITY_SAMPLES<=1 ? 1 : $clog2(VELOCITY_SAMPLES);
    reg [POSITION_D_REMAINDER_BITS-1:0] position_d_remainder;
    reg [15:0] position_d_lower,position_d_lower_quotient;
    wire [POSITION_D_REMAINDER_BITS+15:0] position_d_lower_dividend={position_d_remainder,position_d_lower};
    reg signed [15:0] position_p_request_q8,position_d_request_q8,position_p_slew_q8;
    reg signed [42:0] position_p_scaled,position_d_scaled;
    (* syn_preserve=1, syn_keep=1 *) reg signed [31:0] position_p_term_q8, position_d_term_q8;
    wire signed [31:0] position_pi_raw=-($signed(position_p_term_q8)+$signed(position_i_term_q8));
    wire signed [31:0] position_pi_limited=position_pi_raw>POSITION_ANGLE_LIMIT_Q8 ? POSITION_ANGLE_LIMIT_Q8 :
        position_pi_raw< -POSITION_ANGLE_LIMIT_Q8 ? -POSITION_ANGLE_LIMIT_Q8 : position_pi_raw;
    localparam signed [31:0] POSITION_D_LIMIT_Q8=POSITION_D_LIMIT_DEG*256;
    wire signed [31:0] position_d_raw=-$signed(position_d_term_q8);
    // Compare before sign inversion, avoiding a negate/compare/mux chain.
    wire position_d_high=position_d_term_q8< -POSITION_D_LIMIT_Q8;
    wire position_d_low=position_d_term_q8>POSITION_D_LIMIT_Q8;
    wire signed [31:0] position_d_limited=
        ({32{position_d_high}} & POSITION_D_LIMIT_Q8) |
        ({32{position_d_low}} & -POSITION_D_LIMIT_Q8) |
        ({32{!position_d_high && !position_d_low}} & position_d_raw);
    wire signed [16:0] target_distance=$signed(position_p_request_q8)-$signed(position_p_slew_q8);
    wire signed [16:0] position_p_next=target_distance>TARGET_STEP_Q8 ? $signed(position_p_slew_q8)+TARGET_STEP_Q8 :
        target_distance< -TARGET_STEP_Q8 ? $signed(position_p_slew_q8)-TARGET_STEP_Q8 : $signed(position_p_request_q8);
    wire signed [31:0] position_target_sum=$signed(position_p_next)+$signed(position_d_request_q8);
    wire signed [31:0] position_target_limited=position_target_sum>POSITION_ANGLE_LIMIT_Q8 ? POSITION_ANGLE_LIMIT_Q8 :
        position_target_sum< -POSITION_ANGLE_LIMIT_Q8 ? -POSITION_ANGLE_LIMIT_Q8 : position_target_sum;
    reg [9:0] rate_travel [0:RATE_SAMPLES-1];
    reg [15:0] rate_total;
    reg [15:0] rate_samples;
    integer rate_index;
    localparam integer RAMP_STEPS=POSITION_RAMP_MS==0 ? 1 : (POSITION_RAMP_MS+POSITION_PERIOD_MS-1)/POSITION_PERIOD_MS;
    reg [15:0] ramp_step;
    reg signed [42:0] position_scaled;
    // Register the ramp division before the target multipliers.
    // The outer result is consumed eight clocks after ramp_step advances.
    reg [10:0] ramp_gain;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) ramp_gain<=0;
        else ramp_gain<=POSITION_RAMP_MS==0 ? 11'd1024 : (ramp_step*1024)/RAMP_STEPS;
    end
    reg signed [15:0] position_operand, velocity_operand;
    reg [31:0] position_sum_limit;
    reg [15:0] position_bound_ki;
    reg [31:0] position_div_quotient;
    reg [16:0] position_div_remainder;
    reg [5:0] position_div_count;
    wire [16:0] position_div_trial={position_div_remainder[15:0],position_div_quotient[31]};
    wire position_div_bit=position_div_trial>={1'b0,position_bound_ki};
    wire position_bound_ready=position_div_count==0 && position_bound_ki==position_ki_q12;
    wire signed [32:0] position_integral_bound=$signed({1'b0,position_sum_limit});
    reg signed [31:0] position_integral;
    reg signed [31:0] position_i_term_q8;
    // Ki >= 1 bounds |S| to 16*angle_limit; zero Ki forces S=0.
    localparam integer POSITION_SUM_BITS=$clog2(POSITION_ANGLE_LIMIT_Q8*16+1)+1;
    reg signed [POSITION_SUM_BITS+16:0] position_i_product;
    wire signed [32:0] position_sum_next=$signed(position_integral)+$signed(position_operand);
    wire position_sum_over=!position_sum_next[32] && position_sum_next[31:0]>position_sum_limit;
    wire position_sum_under=position_sum_next[32] && (~position_sum_next[31:0])>=position_sum_limit;
    wire signed [32:0] position_sum_clamped=
        ({33{position_sum_over}} & position_integral_bound) |
        ({33{position_sum_under}} & -position_integral_bound) |
        ({33{!position_sum_over && !position_sum_under}} & position_sum_next);
    reg signed [26:0] previous_angle_measurement_q16;
    // Sequential degree-to-ADC conversion for telemetry, off the PID datapath.
    reg [23:0] target_quotient;
    reg [15:0] target_remainder;
    reg [4:0] target_divide_count;
    reg target_negative;
    reg signed [15:0] target_converted_offset;
    wire [16:0] target_trial={target_remainder,target_quotient[23]};
    wire target_divide_bit=target_trial>=17'd22086;
    wire [23:0] target_quotient_next={target_quotient[22:0],target_divide_bit};
    localparam signed [31:0] ARM_ERROR_CODES=15; // ~5.1 degrees, start only
    localparam signed [31:0] RUN_ERROR_CODES=148; // 49.876 deg allowed; 149 codes = 50.213 deg trips
    localparam [7:0] START_BOOST_SAMPLES=(20+PID_PERIOD_MS-1)/PID_PERIOD_MS;
    localparam integer EFFECT_SAMPLES=(100+PID_PERIOD_MS-1)/PID_PERIOD_MS;
    localparam signed [15:0] ZERO_CODE=ZERO;
    reg boost_pending, boost_negative;
    reg [7:0] boost_remaining;
    reg [31:0] age, errors_start;
    reg signed [31:0] reference, previous_error, previous_position, previous_upright;
    reg signed [31:0] arm_delta_code;
    reg swing_active, arm_swing, auto_swing_session;
    wire pid_active=running && !swing_active;
    wire pid_tick=sample_valid && pid_active;
    wire [5:0] swing_state;
    wire signed [15:0] swing_drive;
    wire swing_capture, startup_kick;
    wire [9:0] capture_adc;
    wire signed [31:0] capture_error=$signed({1'b0,capture_adc})-ZERO;
    assign run_state=!running ? 6'd0 : swing_active ? (swing_state==0 ? 6'd21 : swing_state) : 6'd4;
    swing_up #(.CLK_HZ(CLK_HZ),.ZERO(ZERO),.ADC_PERIOD_MS(PID_PERIOD_MS)) swing(
        .clk(clk),.rst_n(rst_n),.enable(running && swing_active),
        .sample_valid(sample_valid),.adc_mean(adc_mean),.state(swing_state),
        .drive_q8(swing_drive),.capture(swing_capture),.capture_adc(capture_adc),
        .startup_kick(startup_kick),.sample_otr(sample_otr),.diagnostic_capture_flags());
    reg signed [19:0] derivative_q8;
    // Nominal 0.337 deg/code, NOT a measured analog-front-end calibration.
    // 22086/65536 = 0.337005615 deg/code (Q16 approximation of that nominal).
    // Preserve sub-degree precision through the products and integral sum.
    reg signed [26:0] angle_error_q16, angle_delta_q16;
    reg signed [26:0] d_raw_q16;
    reg signed [29:0] d_filter_sum_reg;
    reg [1:0] d_filter_valid;
    reg [1:0] quiet_filter_pipe;
    reg signed [31:0] control_error_code, position_velocity_q8;
    assign diag_position_p_q8=position_ready ? -position_p_term_q8 : 32'sd0;
    assign diag_position_d_raw_q8=position_ready ? position_d_raw : 32'sd0;
    assign diag_position_d_limit_q8=position_ready ? position_d_limited : 32'sd0;
    assign diag_position_i_q8=position_ready ? -position_i_term_q8 : 32'sd0;
    reg [7:0] calc_valid, quiet_pipe;
    reg signed [43:0] p_product_reg,d_product_reg;
    reg signed [31:0] p_term,d_term,effort,boosted_effort;
    reg signed [19:0] diag_rate_pending;
    reg signed [31:0] diag_p_pending,diag_d_pending;
    // Q16 DEGREE-ERROR SUM, not a sum of Ki-weighted outputs.
    // Accumulate only outside the deadband; retain S inside it.
    // S[k]=clamp(S[k-1]+enabled*e[k], +/-I_LIMIT/Ki); I[k]=Ki*S[k].
    // Keep the clamp result in fabric registers before the DSP input stage.
    // Absorbing this boundary into a DSP creates a long clamp-to-DSP path.
    (* syn_preserve=1, syn_keep=1 *) reg signed [31:0] integral_acc;
    reg signed [32:0] integral_sum_pending;
    reg signed [48:0] integral_product_reg;
    reg signed [33:0] integral_product_lo;
    reg signed [32:0] integral_product_hi;
    reg [31:0] integral_bound_q16;
    reg [15:0] bound_ki, divide_denominator;
    reg [7:0] bound_limit;
    reg [31:0] divide_quotient;
    reg [16:0] divide_remainder;
    reg [5:0] divide_count;
    wire [16:0] divide_trial={divide_remainder[15:0],divide_quotient[31]};
    wire divide_bit=divide_trial>={1'b0,divide_denominator};
    localparam [7:0] ANGLE_I_CAP_PERCENT=ANGLE_I_LIMIT_PERCENT;
    localparam signed [26:0] ANGLE_I_DEADBAND_Q16=(ANGLE_I_DEADBAND_MDEG*65536+500)/1000;
    wire [7:0] integral_limit_percent=limit_percent<ANGLE_I_CAP_PERCENT ? limit_percent : ANGLE_I_CAP_PERCENT;
    wire integral_bound_ready=divide_count==0 && bound_ki==ki_q8 && bound_limit==integral_limit_percent;
    reg signed [31:0] i_term;
    function signed [31:0] percent_q8;
        input signed [49:0] value_q24;
        reg signed [33:0] whole;
        begin
            // Truncate after the product register. Adjust a negative fractional
            // result by one Q8 LSB instead of adding 65535 to the full product;
            // this avoids synthesis merging a combinational product-sign path
            // into a DSP multiply/add input register.
            whole=$signed(value_q24[49:16]);
            percent_q8=whole[31:0]+((value_q24[49] && |value_q24[15:0]) ? 32'd1 : 32'd0);
        end
    endfunction
    reg [7:0] ready_samples;
    reg sample_seen, arm_pending;
    reg [6:0] effect_samples;
    reg signed [31:0] effect_start_abs, effect_start_pos;
    wire stop = !stop_key_n || (cmd_valid && (cmd=="s" || cmd==" "));
    wire tilt_exceeded,encoder_fault;
    // Capture the inputs on the exact edge that disarms a running controller.
    // First cause survives subsequent falling-rod samples and stop commands.
    wire timeout_stop=!startup_kick && age>=STALE_CYCLES && !sample_valid;
    wire sensor_stop=sample_valid && !swing_active && (sample_otr || tilt_exceeded || encoder_fault);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin stop_reason<=0;stop_adc<=0;stop_age<=0;stop_flags<=0;stop_encoder<=0;end
        else if(!running && cmd_valid && (cmd=="w" || cmd=="b") && !stop) begin
            stop_reason<=0;stop_adc<=0;stop_age<=0;stop_flags<=0;stop_encoder<=0;
        end else if(running && stop_reason==0 && (stop || timeout_stop || sensor_stop)) begin
            stop_reason<=stop ? (!stop_key_n ? 4'd9 : 4'd12) : timeout_stop ? 4'd4 :
                sample_otr ? 4'd3 : encoder_fault ? 4'd7 : 4'd5;
            stop_adc<=adc_mean;stop_age<=age;stop_encoder<=encoder;
            stop_flags<={1'b0,startup_kick,sample_valid,encoder_fault,tilt_exceeded,sample_otr,
                (cmd_valid && (cmd=="s" || cmd==" ")),!stop_key_n};
        end
    end
    // Snapshot flag: the remaining distance exceeds one fresh-ADC slew step.
    assign diag_target_slew=pid_active && position_loop_enabled && position_engaged && !stop &&
        (target_distance>TARGET_STEP_Q8 || target_distance< -TARGET_STEP_Q8);
    // Cascade: position PD (Ki_pos=0) produces a degree target, not motor PWM.
    wire signed [31:0] upright_error_now=$signed({1'b0,adc_mean})-ZERO;
    wire signed [26:0] angle_measurement_q16=$signed(upright_error_now[11:0])*27'sd22086;
    wire signed [26:0] measured_delta_q16=angle_measurement_q16-previous_angle_measurement_q16;
    // D-only low-pass: y[k]=(5*y[k-1]+3*dx[k])/8.
    // Pole 0.625 gives a 6.38ms decay time at 3ms. Shift/add uses no extra DSP.
    wire signed [29:0] d_filter_sum=($signed(angle_delta_q16)<<<2)+$signed(angle_delta_q16)+
        ($signed(d_raw_q16)<<<1)+$signed(d_raw_q16);
    wire signed [26:0] d_filtered_next=($signed(d_filter_sum_reg)>>>3)+
        ((d_filter_sum_reg[29] && |d_filter_sum_reg[2:0]) ? 27'sd1 : 27'sd0);
    wire signed [26:0] angle_error_now_q16=angle_measurement_q16-
        ($signed(target_offset_q8)*27'sd256);
    wire signed [31:0] control_error_now=$signed({1'b0,adc_mean})-target_angle_code;
    // P/I use target error; D measures motion only, never a target step.
    reg signed [31:0] position_command_counts, move_start;
    wire signed [31:0] position_measured=encoder-reference;
    wire signed [31:0] pos_now=position_measured-position_command_counts;
    wire signed [31:0] abs_upright_error=upright_error_now<0 ? -upright_error_now : upright_error_now;
    wire signed [31:0] abs_control_error=control_error_now<0 ? -control_error_now : control_error_now;
    wire signed [31:0] abs_registered_error=error_code<0 ? -error_code : error_code;
    wire signed [11:0] tilt_delta=$signed({1'b0,adc_mean})-$signed(target_angle_code);
    assign tilt_exceeded=tilt_delta>12'sd148 || tilt_delta< -12'sd148;
    wire signed [31:0] window_close=effect_start_abs-abs_upright_error;
    wire signed [31:0] window_move=position_measured-effect_start_pos;
    wire signed [31:0] difference=upright_error_now-previous_upright;
    wire [9:0] rate_travel_now=difference<0 ? -difference : difference;
    wire [15:0] rate_total_next=rate_total-rate_travel[RATE_SAMPLES-1]+rate_travel_now;
    wire position_rate_ok=rate_samples>=RATE_SAMPLES-1 && rate_total_next<=RATE_TRAVEL_CODES;
    wire signed [31:0] derivative_next=difference<<<8;
    // Reference-relative history is seeded at zero on capture. The first
    // window ramps in over N ticks; no pre-capture motion is differentiated.
    wire signed [31:0] position_difference=position_measured-velocity_history[VELOCITY_SAMPLES-1];
    wire signed [31:0] tracked_velocity_difference=position_difference;
    // CW position raises target angle; e_angle=measured-target then commands CCW.
    // Sliding-window D, normalized to counts per POSITION_PERIOD_MS.
    // Divide AFTER gain multiplication to retain fractional encoder counts.
    // Bound operands before DSP multiplication; preserve error sign at large travel.
    (* syn_preserve=1, syn_keep=1 *) reg signed [32:0] position_p_product;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) position_p_product<=0;
        else position_p_product<=position_operand*$signed({1'b0,position_kp_q12});
    end
    wire signed [32:0] position_d_product=velocity_operand*$signed({1'b0,position_kd_q8});
    reg signed [32:0] position_pwm_raw;
    wire signed [31:0] position_limited=position_pwm_raw>POSITION_ANGLE_LIMIT_Q8 ? POSITION_ANGLE_LIMIT_Q8 :
        position_pwm_raw< -POSITION_ANGLE_LIMIT_Q8 ? -POSITION_ANGLE_LIMIT_Q8 : position_pwm_raw[31:0];
    reg signed [31:0] position_pwm_q8;
    // Bounded operands: inner-loop error is within the 10-bit ADC range.
    // Register multiply, sum, friction offset, then polarity/clamp.
    wire signed [43:0] p_product=angle_error_q16*$signed({1'b0,kp_q8});
    wire signed [43:0] d_product=angle_delta_q16*$signed({1'b0,kd_q8});
    wire signed [31:0] p_calc=percent_q8(p_product_reg);
    wire signed [31:0] d_scaled=percent_q8(d_product_reg);
    wire signed [31:0] angle_mapped=polarity_reverse ? -boosted_effort : boosted_effort;
    // Position polarity follows the confirmed CW encoder/PWM convention.
    wire signed [31:0] mapped=angle_mapped;
    wire signed [31:0] bound=$signed({1'b0,limit_percent,8'b0});
    wire signed [32:0] integral_bound=$signed({1'b0,integral_bound_q16});
    // angle_error_q16 is measured angle minus the applied position-loop target.
    // It is latched with this ADC sample before calc_valid[0] consumes it.
    wire angle_i_accumulate=angle_error_q16>ANGLE_I_DEADBAND_Q16 ||
                            angle_error_q16< -ANGLE_I_DEADBAND_Q16;
    wire signed [32:0] integral_next=$signed(integral_acc)+
        (angle_i_accumulate ? $signed(angle_error_q16) : 33'sd0);
    wire integral_over=!integral_sum_pending[32] && integral_sum_pending[31:0]>integral_bound_q16;
    // For negative x, abs(x)>bound iff one's-complement(x)>=bound.
    wire integral_under=integral_sum_pending[32] && (~integral_sum_pending[31:0])>=integral_bound_q16;
    // Parallel selections avoid a priority mux after two wide comparisons.
    wire signed [32:0] integral_clamped=
        ({33{integral_over}} & integral_bound) |
        ({33{integral_under}} & -integral_bound) |
        ({33{!integral_over && !integral_under}} & integral_sum_pending);
    // Two <=18x18 multiplies plus a registered combine avoid a long 32x17
    // DSP cascade. Low half is unsigned, high half is signed.
    wire signed [33:0] integral_lo= $signed({1'b0,integral_acc[15:0]})*$signed({1'b0,ki_q8});
    wire signed [32:0] integral_hi= $signed(integral_acc[31:16])*$signed({1'b0,ki_q8});
    wire signed [48:0] integral_product=($signed(integral_product_hi)<<<16)+$signed(integral_product_lo);
    wire signed [31:0] integral_output_bound=$signed({1'b0,integral_limit_percent,8'b0});
    wire signed [31:0] integral_output_raw=percent_q8(integral_product_reg);
    wire signed [31:0] integral_output_limited=integral_output_raw>integral_output_bound ? integral_output_bound :
        integral_output_raw< -integral_output_bound ? -integral_output_bound : integral_output_raw;
    wire signed [31:0] pid_effort=p_term+d_term+i_term;
    // Divide only while disarmed, using a 32-cycle restoring divider.
    // floor((I_LIMIT << 24)/Ki_q8) is the Q16 error-sum limit; rounding
    // never allows |Ki*S| to exceed I_LIMIT. Ki=0 clears S and never divides.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            integral_bound_q16<=0; bound_ki<=0; bound_limit<=0;
            divide_denominator<=0; divide_quotient<=0;
            divide_remainder<=0; divide_count<=0;
        end else if(divide_count!=0) begin
            divide_remainder<=divide_bit ? divide_trial-{1'b0,divide_denominator} : divide_trial;
            divide_quotient<={divide_quotient[30:0],divide_bit};
            divide_count<=divide_count-1'b1;
            if(divide_count==1) integral_bound_q16<={divide_quotient[30:0],divide_bit};
        end else if(!pid_active && !integral_bound_ready) begin
            bound_ki<=ki_q8; bound_limit<=integral_limit_percent;
            if(ki_q8==0) integral_bound_q16<=0;
            else begin
                divide_quotient<={integral_limit_percent,24'b0}; divide_remainder<=0;
                divide_denominator<=ki_q8; divide_count<=32;
            end
        end
    end
    // Wait for the first real correction; never kick an upright, quiet rod.
    // A zero request, quiet center, or direction reversal ends the pulse early.
    wire use_start_boost=boost_enabled && pid_active && !quiet_pipe[6] && effort!=0 &&
        (boost_pending || (boost_remaining!=0 && effort[31]==boost_negative));
    // Observe, never feed back: retain the operands actually used to form
    // this sample's effort, then publish them with its output. Rate is measured
    // Q8 ADC codes per PID interval; P/D are Q8 percent BEFORE polarity/limits.
    // Encoder feedback is safety-critical only when the position loop uses it.
    // Angle-only mode retains error telemetry without interrupting balance.
    // Keep counting/reporting illegal AB transitions, but make the shutdown
    // response selectable.  Angle/ADC safety checks remain independent.
    assign encoder_fault=ENCODER_FAULT_PROTECTION &&
        position_loop_enabled && encoder_errors!=errors_start;
    // Recompute the outer integral bound off the PID datapath after a Ki edit.
    // Restoring division avoids a variable combinational divider at 50 MHz.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            position_sum_limit<=POSITION_KI_Q12==0 ? 0 :
                (33'sd16*POSITION_ANGLE_LIMIT_Q8)/POSITION_KI_Q12;
            position_bound_ki<=POSITION_KI_Q12;
            position_div_quotient<=0; position_div_remainder<=0; position_div_count<=0;
        end else if(position_div_count!=0) begin
            position_div_remainder<=position_div_bit ?
                position_div_trial-{1'b0,position_bound_ki} : position_div_trial;
            position_div_quotient<={position_div_quotient[30:0],position_div_bit};
            position_div_count<=position_div_count-1'b1;
            if(position_div_count==1)
                position_sum_limit<={position_div_quotient[30:0],position_div_bit};
        end else if(!position_bound_ready) begin
            position_bound_ki<=position_ki_q12;
            if(position_ki_q12==0) position_sum_limit<=0;
            else begin
                position_div_quotient<=POSITION_ANGLE_LIMIT_Q8*16;
                position_div_remainder<=0; position_div_count<=32;
            end
        end
    end
    wire position_angle_ok=abs_upright_error<=POSITION_ENGAGE_CODES;
    // Sum absolute sample motion over the window: reversals cannot cancel.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rate_total<=0; rate_samples<=0;
            for(rate_index=0;rate_index<RATE_SAMPLES;rate_index=rate_index+1) rate_travel[rate_index]<=0;
        end else if(!pid_active || stop || !position_loop_enabled) begin
            rate_total<=0; rate_samples<=0;
            for(rate_index=0;rate_index<RATE_SAMPLES;rate_index=rate_index+1) rate_travel[rate_index]<=0;
        end else if(sample_valid) begin
            rate_total<=rate_total_next;
            if(rate_samples<RATE_SAMPLES) rate_samples<=rate_samples+1'b1;
            rate_travel[0]<=rate_travel_now;
            for(rate_index=1;rate_index<RATE_SAMPLES;rate_index=rate_index+1)
                rate_travel[rate_index]<=rate_travel[rate_index-1];
        end
    end
    wire position_capture=pid_active && position_loop_enabled && !position_engaged && !stop &&
        position_stable_samples==POSITION_ENGAGE_SAMPLES-1 && position_angle_ok && position_rate_ok &&
        sample_valid && position_bound_ready && !sample_otr && !tilt_exceeded && !encoder_fault;
    localparam signed [31:0] POSITION_STEP_COUNTS=COUNTS_PER_REV/4;
    wire curve2_requested=(curve2_enable===1'b1);
    wire sine_requested=(sine_enable===1'b1) || curve2_requested;
    reg dip_mode2,dip_first;
    reg signed [31:0] move_velocity_q16;
    reg move_busy;
    wire move_trigger;
    assign target_velocity_q16=position_ready && move_busy ? move_velocity_q16 : 32'sd0;
    wire trajectory_tick=position_timer==POSITION_CYCLES-1;
    reg dip_armed,dip_pending,dip_waiting;
    reg [1:0] dip_segment;
    reg [31:0] dip_hold_ticks;
    reg [31:0] dip_settle_ticks;
    reg [31:0] dip_speed_ticks;
    reg signed [31:0] dip_speed_previous;
    reg dip_speed_ok;
    localparam integer DIP_HOLD_TICKS=PROFILE_HOLD_MS/POSITION_PERIOD_MS;
    localparam integer SAW_HOLD_TICKS=SAW_HOLD_MS/POSITION_PERIOD_MS;
    localparam integer DIP_SETTLE_TICKS=PROFILE_SETTLE_MS/POSITION_PERIOD_MS;
    localparam integer DIP_SPEED_TICKS=100/POSITION_PERIOD_MS;
    wire signed [32:0] dip_error=$signed({position_measured[31],position_measured})-
        $signed({position_target_counts[31],position_target_counts});
    wire signed [32:0] dip_speed_delta=$signed({position_measured[31],position_measured})-
        $signed({dip_speed_previous[31],dip_speed_previous});
    wire dip_arrived=dip_error<=PROFILE_ARRIVE_COUNTS && dip_error>=-PROFILE_ARRIVE_COUNTS && dip_speed_ok;
    wire dip_outside_hold=dip_error>PROFILE_RELEASE_COUNTS || dip_error< -PROFILE_RELEASE_COUNTS;
    // Independent 100ms speed check; only needed before dwell is latched.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin dip_speed_ticks<=0;dip_speed_previous<=0;dip_speed_ok<=0;end
        else if(!sine_active || move_busy || move_trigger) begin
            dip_speed_ticks<=0;dip_speed_previous<=position_measured;dip_speed_ok<=0;
        end else if(trajectory_tick) begin
            if(dip_speed_ticks>=DIP_SPEED_TICKS-1) begin
                dip_speed_ticks<=0;dip_speed_previous<=position_measured;
                dip_speed_ok<=dip_speed_delta<=PROFILE_SETTLE_SPEED_COUNTS &&
                    dip_speed_delta>=-PROFILE_SETTLE_SPEED_COUNTS;
            end else dip_speed_ticks<=dip_speed_ticks+1'b1;
        end
    end
    reg [15:0] move_distance;
    reg [15:0] requested_distance;
    always @* begin
        case(cmd)
            "g": requested_distance=COUNTS_PER_REV;
            "h": requested_distance=(COUNTS_PER_REV*3+2)/4;
            "e": requested_distance=(COUNTS_PER_REV+1)/2;
            "u": requested_distance=(COUNTS_PER_REV+1)/3;
            "y": requested_distance=(COUNTS_PER_REV+3)/6;
            "r": requested_distance=(COUNTS_PER_REV*3+4)/8;
            "t": requested_distance=(COUNTS_PER_REV+6)/12;
            "f": requested_distance=(COUNTS_PER_REV+4)/8;
            default: requested_distance=POSITION_STEP_COUNTS;
        endcase
    end
    reg [8:0] move_index;
    integer move_tick;
    reg move_reverse;
    reg move_sinusoidal;
    reg move_triangular;
    reg curve_display;
    wire signed [31:0] launch_start=(sine_active && dip_pending && !dip_first) ?
        position_command_counts : position_measured;
    // DIP1: sine. DIP2: slow rise, top hold, quicker return, bottom hold.
    wire [15:0] dip_distance=(COUNTS_PER_REV*3+2)/4;
    wire dip_reverse=dip_segment==1;
    wire dip_range_ok=dip_reverse ?
        launch_start>=(-32'sh7fffffff-1)+$signed({1'b0,dip_distance}) :
        launch_start<=32'sh7fffffff-$signed({1'b0,dip_distance});
    wire dip_trigger=sine_active && dip_pending && sine_requested && position_ready && dip_range_ok && !move_busy;
    wire uart_move_trigger=cmd_valid && (cmd=="n" || cmd=="e" || cmd=="t" || cmd=="f" || cmd=="g" || cmd=="h" || cmd=="u" || cmd=="y" || cmd=="r") && position_ready && !sine_active && !sine_requested &&
        position_measured<=32'sh7fffffff-$signed({1'b0,requested_distance});
    assign move_trigger=uart_move_trigger || dip_trigger;
    wire [15:0] launch_distance=dip_trigger ? dip_distance : requested_distance;
    wire launch_reverse=dip_trigger && dip_reverse;
    wire signed [31:0] launch_delta=launch_reverse ? -$signed({1'b0,launch_distance}) : $signed({1'b0,launch_distance});
    // DIP only schedules commands; UART and DIP share the entire move/PID path.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) dip_settle_ticks<=0;
        else if(!sine_active || !sine_requested || !position_ready || move_busy || dip_pending)
            dip_settle_ticks<=0;
        else if(trajectory_tick) begin
            if(dip_outside_hold) dip_settle_ticks<=0;
            else if(dip_settle_ticks<DIP_SETTLE_TICKS) begin
                if(dip_arrived) dip_settle_ticks<=dip_settle_ticks+1'b1;
                else dip_settle_ticks<=0;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            dip_mode2<=0;dip_first<=1;
            sine_active<=0;dip_armed<=1;dip_pending<=0;dip_waiting<=0;dip_segment<=0;dip_hold_ticks<=0;
        end else if(!position_ready || position_capture) begin
            dip_mode2<=0;dip_first<=1;
            sine_active<=0;dip_armed<=1;dip_pending<=0;dip_waiting<=0;dip_segment<=0;dip_hold_ticks<=0;
        end else if(!sine_requested) begin
            // Like UART, an already accepted move finishes; cancel later commands.
            sine_active<=0;dip_armed<=1;dip_pending<=0;dip_waiting<=0;dip_hold_ticks<=0;
        end else if(!sine_active) begin
            if(dip_armed && !move_busy) begin
                dip_mode2<=curve2_requested;dip_first<=1;
                sine_active<=1;dip_armed<=0;dip_pending<=1;dip_waiting<=0;dip_segment<=0;dip_hold_ticks<=0;
            end
        end else if(dip_trigger) begin
            dip_first<=0;
            dip_pending<=0;dip_waiting<=1;dip_hold_ticks<=0;
        end else if(dip_pending && !dip_range_ok) begin
            sine_active<=0;dip_pending<=0;
        end else if(dip_waiting && !move_busy && dip_mode2==curve2_requested &&
                    dip_mode2 && dip_hold_ticks<SAW_HOLD_TICKS) begin
            if(trajectory_tick) dip_hold_ticks<=dip_hold_ticks+1'b1;
        end else if(dip_waiting && !move_busy) begin
            // Change mode only at a zero-speed segment boundary. The reference
            // remains the previous endpoint, including at every cycle seam.
            if(dip_mode2!=curve2_requested) begin
                dip_mode2<=curve2_requested;dip_segment<=0;
            end else if(dip_segment==1) dip_segment<=0;
            else dip_segment<=dip_segment+1'b1;
            dip_pending<=1;dip_waiting<=0;dip_hold_ticks<=0;
        end
    end
    // Q16 (1-cos(pi*index/256))/2: a zero-speed half sine-position cycle.
    function [16:0] sine_position_offset(input [8:0] index);
        begin
            case(index)
                9'd0: sine_position_offset=17'd0;
                9'd1: sine_position_offset=17'd2;
                9'd2: sine_position_offset=17'd10;
                9'd3: sine_position_offset=17'd22;
                9'd4: sine_position_offset=17'd39;
                9'd5: sine_position_offset=17'd62;
                9'd6: sine_position_offset=17'd89;
                9'd7: sine_position_offset=17'd121;
                9'd8: sine_position_offset=17'd158;
                9'd9: sine_position_offset=17'd200;
                9'd10: sine_position_offset=17'd246;
                9'd11: sine_position_offset=17'd298;
                9'd12: sine_position_offset=17'd355;
                9'd13: sine_position_offset=17'd416;
                9'd14: sine_position_offset=17'd482;
                9'd15: sine_position_offset=17'd554;
                9'd16: sine_position_offset=17'd630;
                9'd17: sine_position_offset=17'd710;
                9'd18: sine_position_offset=17'd796;
                9'd19: sine_position_offset=17'd887;
                9'd20: sine_position_offset=17'd982;
                9'd21: sine_position_offset=17'd1082;
                9'd22: sine_position_offset=17'd1187;
                9'd23: sine_position_offset=17'd1297;
                9'd24: sine_position_offset=17'd1411;
                9'd25: sine_position_offset=17'd1530;
                9'd26: sine_position_offset=17'd1654;
                9'd27: sine_position_offset=17'd1782;
                9'd28: sine_position_offset=17'd1915;
                9'd29: sine_position_offset=17'd2053;
                9'd30: sine_position_offset=17'd2196;
                9'd31: sine_position_offset=17'd2343;
                9'd32: sine_position_offset=17'd2494;
                9'd33: sine_position_offset=17'd2650;
                9'd34: sine_position_offset=17'd2811;
                9'd35: sine_position_offset=17'd2976;
                9'd36: sine_position_offset=17'd3146;
                9'd37: sine_position_offset=17'd3320;
                9'd38: sine_position_offset=17'd3499;
                9'd39: sine_position_offset=17'd3682;
                9'd40: sine_position_offset=17'd3869;
                9'd41: sine_position_offset=17'd4061;
                9'd42: sine_position_offset=17'd4257;
                9'd43: sine_position_offset=17'd4457;
                9'd44: sine_position_offset=17'd4662;
                9'd45: sine_position_offset=17'd4871;
                9'd46: sine_position_offset=17'd5084;
                9'd47: sine_position_offset=17'd5301;
                9'd48: sine_position_offset=17'd5522;
                9'd49: sine_position_offset=17'd5748;
                9'd50: sine_position_offset=17'd5977;
                9'd51: sine_position_offset=17'd6211;
                9'd52: sine_position_offset=17'd6448;
                9'd53: sine_position_offset=17'd6690;
                9'd54: sine_position_offset=17'd6935;
                9'd55: sine_position_offset=17'd7185;
                9'd56: sine_position_offset=17'd7438;
                9'd57: sine_position_offset=17'd7695;
                9'd58: sine_position_offset=17'd7956;
                9'd59: sine_position_offset=17'd8220;
                9'd60: sine_position_offset=17'd8489;
                9'd61: sine_position_offset=17'd8760;
                9'd62: sine_position_offset=17'd9036;
                9'd63: sine_position_offset=17'd9315;
                9'd64: sine_position_offset=17'd9598;
                9'd65: sine_position_offset=17'd9884;
                9'd66: sine_position_offset=17'd10173;
                9'd67: sine_position_offset=17'd10466;
                9'd68: sine_position_offset=17'd10762;
                9'd69: sine_position_offset=17'd11062;
                9'd70: sine_position_offset=17'd11365;
                9'd71: sine_position_offset=17'd11671;
                9'd72: sine_position_offset=17'd11980;
                9'd73: sine_position_offset=17'd12293;
                9'd74: sine_position_offset=17'd12608;
                9'd75: sine_position_offset=17'd12927;
                9'd76: sine_position_offset=17'd13248;
                9'd77: sine_position_offset=17'd13573;
                9'd78: sine_position_offset=17'd13900;
                9'd79: sine_position_offset=17'd14230;
                9'd80: sine_position_offset=17'd14563;
                9'd81: sine_position_offset=17'd14899;
                9'd82: sine_position_offset=17'd15237;
                9'd83: sine_position_offset=17'd15578;
                9'd84: sine_position_offset=17'd15922;
                9'd85: sine_position_offset=17'd16268;
                9'd86: sine_position_offset=17'd16617;
                9'd87: sine_position_offset=17'd16968;
                9'd88: sine_position_offset=17'd17321;
                9'd89: sine_position_offset=17'd17677;
                9'd90: sine_position_offset=17'd18035;
                9'd91: sine_position_offset=17'd18395;
                9'd92: sine_position_offset=17'd18758;
                9'd93: sine_position_offset=17'd19122;
                9'd94: sine_position_offset=17'd19489;
                9'd95: sine_position_offset=17'd19858;
                9'd96: sine_position_offset=17'd20228;
                9'd97: sine_position_offset=17'd20601;
                9'd98: sine_position_offset=17'd20975;
                9'd99: sine_position_offset=17'd21351;
                9'd100: sine_position_offset=17'd21729;
                9'd101: sine_position_offset=17'd22108;
                9'd102: sine_position_offset=17'd22489;
                9'd103: sine_position_offset=17'd22872;
                9'd104: sine_position_offset=17'd23256;
                9'd105: sine_position_offset=17'd23641;
                9'd106: sine_position_offset=17'd24028;
                9'd107: sine_position_offset=17'd24417;
                9'd108: sine_position_offset=17'd24806;
                9'd109: sine_position_offset=17'd25197;
                9'd110: sine_position_offset=17'd25588;
                9'd111: sine_position_offset=17'd25981;
                9'd112: sine_position_offset=17'd26375;
                9'd113: sine_position_offset=17'd26770;
                9'd114: sine_position_offset=17'd27166;
                9'd115: sine_position_offset=17'd27563;
                9'd116: sine_position_offset=17'd27960;
                9'd117: sine_position_offset=17'd28358;
                9'd118: sine_position_offset=17'd28757;
                9'd119: sine_position_offset=17'd29156;
                9'd120: sine_position_offset=17'd29556;
                9'd121: sine_position_offset=17'd29957;
                9'd122: sine_position_offset=17'd30357;
                9'd123: sine_position_offset=17'd30759;
                9'd124: sine_position_offset=17'd31160;
                9'd125: sine_position_offset=17'd31562;
                9'd126: sine_position_offset=17'd31964;
                9'd127: sine_position_offset=17'd32366;
                9'd128: sine_position_offset=17'd32768;
                9'd129: sine_position_offset=17'd33170;
                9'd130: sine_position_offset=17'd33572;
                9'd131: sine_position_offset=17'd33974;
                9'd132: sine_position_offset=17'd34376;
                9'd133: sine_position_offset=17'd34777;
                9'd134: sine_position_offset=17'd35179;
                9'd135: sine_position_offset=17'd35579;
                9'd136: sine_position_offset=17'd35980;
                9'd137: sine_position_offset=17'd36380;
                9'd138: sine_position_offset=17'd36779;
                9'd139: sine_position_offset=17'd37178;
                9'd140: sine_position_offset=17'd37576;
                9'd141: sine_position_offset=17'd37973;
                9'd142: sine_position_offset=17'd38370;
                9'd143: sine_position_offset=17'd38766;
                9'd144: sine_position_offset=17'd39161;
                9'd145: sine_position_offset=17'd39555;
                9'd146: sine_position_offset=17'd39948;
                9'd147: sine_position_offset=17'd40339;
                9'd148: sine_position_offset=17'd40730;
                9'd149: sine_position_offset=17'd41119;
                9'd150: sine_position_offset=17'd41508;
                9'd151: sine_position_offset=17'd41895;
                9'd152: sine_position_offset=17'd42280;
                9'd153: sine_position_offset=17'd42664;
                9'd154: sine_position_offset=17'd43047;
                9'd155: sine_position_offset=17'd43428;
                9'd156: sine_position_offset=17'd43807;
                9'd157: sine_position_offset=17'd44185;
                9'd158: sine_position_offset=17'd44561;
                9'd159: sine_position_offset=17'd44935;
                9'd160: sine_position_offset=17'd45308;
                9'd161: sine_position_offset=17'd45678;
                9'd162: sine_position_offset=17'd46047;
                9'd163: sine_position_offset=17'd46414;
                9'd164: sine_position_offset=17'd46778;
                9'd165: sine_position_offset=17'd47141;
                9'd166: sine_position_offset=17'd47501;
                9'd167: sine_position_offset=17'd47859;
                9'd168: sine_position_offset=17'd48215;
                9'd169: sine_position_offset=17'd48568;
                9'd170: sine_position_offset=17'd48919;
                9'd171: sine_position_offset=17'd49268;
                9'd172: sine_position_offset=17'd49614;
                9'd173: sine_position_offset=17'd49958;
                9'd174: sine_position_offset=17'd50299;
                9'd175: sine_position_offset=17'd50637;
                9'd176: sine_position_offset=17'd50973;
                9'd177: sine_position_offset=17'd51306;
                9'd178: sine_position_offset=17'd51636;
                9'd179: sine_position_offset=17'd51963;
                9'd180: sine_position_offset=17'd52288;
                9'd181: sine_position_offset=17'd52609;
                9'd182: sine_position_offset=17'd52928;
                9'd183: sine_position_offset=17'd53243;
                9'd184: sine_position_offset=17'd53556;
                9'd185: sine_position_offset=17'd53865;
                9'd186: sine_position_offset=17'd54171;
                9'd187: sine_position_offset=17'd54474;
                9'd188: sine_position_offset=17'd54774;
                9'd189: sine_position_offset=17'd55070;
                9'd190: sine_position_offset=17'd55363;
                9'd191: sine_position_offset=17'd55652;
                9'd192: sine_position_offset=17'd55938;
                9'd193: sine_position_offset=17'd56221;
                9'd194: sine_position_offset=17'd56500;
                9'd195: sine_position_offset=17'd56776;
                9'd196: sine_position_offset=17'd57047;
                9'd197: sine_position_offset=17'd57316;
                9'd198: sine_position_offset=17'd57580;
                9'd199: sine_position_offset=17'd57841;
                9'd200: sine_position_offset=17'd58098;
                9'd201: sine_position_offset=17'd58351;
                9'd202: sine_position_offset=17'd58601;
                9'd203: sine_position_offset=17'd58846;
                9'd204: sine_position_offset=17'd59088;
                9'd205: sine_position_offset=17'd59325;
                9'd206: sine_position_offset=17'd59559;
                9'd207: sine_position_offset=17'd59788;
                9'd208: sine_position_offset=17'd60014;
                9'd209: sine_position_offset=17'd60235;
                9'd210: sine_position_offset=17'd60452;
                9'd211: sine_position_offset=17'd60665;
                9'd212: sine_position_offset=17'd60874;
                9'd213: sine_position_offset=17'd61079;
                9'd214: sine_position_offset=17'd61279;
                9'd215: sine_position_offset=17'd61475;
                9'd216: sine_position_offset=17'd61667;
                9'd217: sine_position_offset=17'd61854;
                9'd218: sine_position_offset=17'd62037;
                9'd219: sine_position_offset=17'd62216;
                9'd220: sine_position_offset=17'd62390;
                9'd221: sine_position_offset=17'd62560;
                9'd222: sine_position_offset=17'd62725;
                9'd223: sine_position_offset=17'd62886;
                9'd224: sine_position_offset=17'd63042;
                9'd225: sine_position_offset=17'd63193;
                9'd226: sine_position_offset=17'd63340;
                9'd227: sine_position_offset=17'd63483;
                9'd228: sine_position_offset=17'd63621;
                9'd229: sine_position_offset=17'd63754;
                9'd230: sine_position_offset=17'd63882;
                9'd231: sine_position_offset=17'd64006;
                9'd232: sine_position_offset=17'd64125;
                9'd233: sine_position_offset=17'd64239;
                9'd234: sine_position_offset=17'd64349;
                9'd235: sine_position_offset=17'd64454;
                9'd236: sine_position_offset=17'd64554;
                9'd237: sine_position_offset=17'd64649;
                9'd238: sine_position_offset=17'd64740;
                9'd239: sine_position_offset=17'd64826;
                9'd240: sine_position_offset=17'd64906;
                9'd241: sine_position_offset=17'd64982;
                9'd242: sine_position_offset=17'd65054;
                9'd243: sine_position_offset=17'd65120;
                9'd244: sine_position_offset=17'd65181;
                9'd245: sine_position_offset=17'd65238;
                9'd246: sine_position_offset=17'd65290;
                9'd247: sine_position_offset=17'd65336;
                9'd248: sine_position_offset=17'd65378;
                9'd249: sine_position_offset=17'd65415;
                9'd250: sine_position_offset=17'd65447;
                9'd251: sine_position_offset=17'd65474;
                9'd252: sine_position_offset=17'd65497;
                9'd253: sine_position_offset=17'd65514;
                9'd254: sine_position_offset=17'd65526;
                9'd255: sine_position_offset=17'd65534;
                9'd256: sine_position_offset=17'd65536;
                default: sine_position_offset=17'd65536;
            endcase
        end
    endfunction
    // Rounded triangle half-wave: central 50% constant speed; quintic
    // velocity ramps over each end 25%. Zero velocity/acceleration/jerk at turns.
    function [16:0] triangle_position_offset(input [8:0] index);
        begin
            case(index)
                9'd0: triangle_position_offset=17'd0;
                9'd1: triangle_position_offset=17'd0;
                9'd2: triangle_position_offset=17'd0;
                9'd3: triangle_position_offset=17'd0;
                9'd4: triangle_position_offset=17'd1;
                9'd5: triangle_position_offset=17'd2;
                9'd6: triangle_position_offset=17'd4;
                9'd7: triangle_position_offset=17'd7;
                9'd8: triangle_position_offset=17'd11;
                9'd9: triangle_position_offset=17'd18;
                9'd10: triangle_position_offset=17'd27;
                9'd11: triangle_position_offset=17'd38;
                9'd12: triangle_position_offset=17'd53;
                9'd13: triangle_position_offset=17'd72;
                9'd14: triangle_position_offset=17'd95;
                9'd15: triangle_position_offset=17'd122;
                9'd16: triangle_position_offset=17'd155;
                9'd17: triangle_position_offset=17'd193;
                9'd18: triangle_position_offset=17'd237;
                9'd19: triangle_position_offset=17'd288;
                9'd20: triangle_position_offset=17'd346;
                9'd21: triangle_position_offset=17'd411;
                9'd22: triangle_position_offset=17'd484;
                9'd23: triangle_position_offset=17'd565;
                9'd24: triangle_position_offset=17'd655;
                9'd25: triangle_position_offset=17'd753;
                9'd26: triangle_position_offset=17'd861;
                9'd27: triangle_position_offset=17'd977;
                9'd28: triangle_position_offset=17'd1104;
                9'd29: triangle_position_offset=17'd1240;
                9'd30: triangle_position_offset=17'd1385;
                9'd31: triangle_position_offset=17'd1541;
                9'd32: triangle_position_offset=17'd1707;
                9'd33: triangle_position_offset=17'd1882;
                9'd34: triangle_position_offset=17'd2068;
                9'd35: triangle_position_offset=17'd2264;
                9'd36: triangle_position_offset=17'd2469;
                9'd37: triangle_position_offset=17'd2684;
                9'd38: triangle_position_offset=17'd2909;
                9'd39: triangle_position_offset=17'd3142;
                9'd40: triangle_position_offset=17'd3385;
                9'd41: triangle_position_offset=17'd3637;
                9'd42: triangle_position_offset=17'd3897;
                9'd43: triangle_position_offset=17'd4166;
                9'd44: triangle_position_offset=17'd4442;
                9'd45: triangle_position_offset=17'd4725;
                9'd46: triangle_position_offset=17'd5016;
                9'd47: triangle_position_offset=17'd5313;
                9'd48: triangle_position_offset=17'd5616;
                9'd49: triangle_position_offset=17'd5925;
                9'd50: triangle_position_offset=17'd6239;
                9'd51: triangle_position_offset=17'd6557;
                9'd52: triangle_position_offset=17'd6880;
                9'd53: triangle_position_offset=17'd7206;
                9'd54: triangle_position_offset=17'd7536;
                9'd55: triangle_position_offset=17'd7869;
                9'd56: triangle_position_offset=17'd8203;
                9'd57: triangle_position_offset=17'd8540;
                9'd58: triangle_position_offset=17'd8878;
                9'd59: triangle_position_offset=17'd9218;
                9'd60: triangle_position_offset=17'd9558;
                9'd61: triangle_position_offset=17'd9899;
                9'd62: triangle_position_offset=17'd10240;
                9'd63: triangle_position_offset=17'd10581;
                9'd64: triangle_position_offset=17'd10923;
                9'd65: triangle_position_offset=17'd11264;
                9'd66: triangle_position_offset=17'd11605;
                9'd67: triangle_position_offset=17'd11947;
                9'd68: triangle_position_offset=17'd12288;
                9'd69: triangle_position_offset=17'd12629;
                9'd70: triangle_position_offset=17'd12971;
                9'd71: triangle_position_offset=17'd13312;
                9'd72: triangle_position_offset=17'd13653;
                9'd73: triangle_position_offset=17'd13995;
                9'd74: triangle_position_offset=17'd14336;
                9'd75: triangle_position_offset=17'd14677;
                9'd76: triangle_position_offset=17'd15019;
                9'd77: triangle_position_offset=17'd15360;
                9'd78: triangle_position_offset=17'd15701;
                9'd79: triangle_position_offset=17'd16043;
                9'd80: triangle_position_offset=17'd16384;
                9'd81: triangle_position_offset=17'd16725;
                9'd82: triangle_position_offset=17'd17067;
                9'd83: triangle_position_offset=17'd17408;
                9'd84: triangle_position_offset=17'd17749;
                9'd85: triangle_position_offset=17'd18091;
                9'd86: triangle_position_offset=17'd18432;
                9'd87: triangle_position_offset=17'd18773;
                9'd88: triangle_position_offset=17'd19115;
                9'd89: triangle_position_offset=17'd19456;
                9'd90: triangle_position_offset=17'd19797;
                9'd91: triangle_position_offset=17'd20139;
                9'd92: triangle_position_offset=17'd20480;
                9'd93: triangle_position_offset=17'd20821;
                9'd94: triangle_position_offset=17'd21163;
                9'd95: triangle_position_offset=17'd21504;
                9'd96: triangle_position_offset=17'd21845;
                9'd97: triangle_position_offset=17'd22187;
                9'd98: triangle_position_offset=17'd22528;
                9'd99: triangle_position_offset=17'd22869;
                9'd100: triangle_position_offset=17'd23211;
                9'd101: triangle_position_offset=17'd23552;
                9'd102: triangle_position_offset=17'd23893;
                9'd103: triangle_position_offset=17'd24235;
                9'd104: triangle_position_offset=17'd24576;
                9'd105: triangle_position_offset=17'd24917;
                9'd106: triangle_position_offset=17'd25259;
                9'd107: triangle_position_offset=17'd25600;
                9'd108: triangle_position_offset=17'd25941;
                9'd109: triangle_position_offset=17'd26283;
                9'd110: triangle_position_offset=17'd26624;
                9'd111: triangle_position_offset=17'd26965;
                9'd112: triangle_position_offset=17'd27307;
                9'd113: triangle_position_offset=17'd27648;
                9'd114: triangle_position_offset=17'd27989;
                9'd115: triangle_position_offset=17'd28331;
                9'd116: triangle_position_offset=17'd28672;
                9'd117: triangle_position_offset=17'd29013;
                9'd118: triangle_position_offset=17'd29355;
                9'd119: triangle_position_offset=17'd29696;
                9'd120: triangle_position_offset=17'd30037;
                9'd121: triangle_position_offset=17'd30379;
                9'd122: triangle_position_offset=17'd30720;
                9'd123: triangle_position_offset=17'd31061;
                9'd124: triangle_position_offset=17'd31403;
                9'd125: triangle_position_offset=17'd31744;
                9'd126: triangle_position_offset=17'd32085;
                9'd127: triangle_position_offset=17'd32427;
                9'd128: triangle_position_offset=17'd32768;
                9'd129: triangle_position_offset=17'd33109;
                9'd130: triangle_position_offset=17'd33451;
                9'd131: triangle_position_offset=17'd33792;
                9'd132: triangle_position_offset=17'd34133;
                9'd133: triangle_position_offset=17'd34475;
                9'd134: triangle_position_offset=17'd34816;
                9'd135: triangle_position_offset=17'd35157;
                9'd136: triangle_position_offset=17'd35499;
                9'd137: triangle_position_offset=17'd35840;
                9'd138: triangle_position_offset=17'd36181;
                9'd139: triangle_position_offset=17'd36523;
                9'd140: triangle_position_offset=17'd36864;
                9'd141: triangle_position_offset=17'd37205;
                9'd142: triangle_position_offset=17'd37547;
                9'd143: triangle_position_offset=17'd37888;
                9'd144: triangle_position_offset=17'd38229;
                9'd145: triangle_position_offset=17'd38571;
                9'd146: triangle_position_offset=17'd38912;
                9'd147: triangle_position_offset=17'd39253;
                9'd148: triangle_position_offset=17'd39595;
                9'd149: triangle_position_offset=17'd39936;
                9'd150: triangle_position_offset=17'd40277;
                9'd151: triangle_position_offset=17'd40619;
                9'd152: triangle_position_offset=17'd40960;
                9'd153: triangle_position_offset=17'd41301;
                9'd154: triangle_position_offset=17'd41643;
                9'd155: triangle_position_offset=17'd41984;
                9'd156: triangle_position_offset=17'd42325;
                9'd157: triangle_position_offset=17'd42667;
                9'd158: triangle_position_offset=17'd43008;
                9'd159: triangle_position_offset=17'd43349;
                9'd160: triangle_position_offset=17'd43691;
                9'd161: triangle_position_offset=17'd44032;
                9'd162: triangle_position_offset=17'd44373;
                9'd163: triangle_position_offset=17'd44715;
                9'd164: triangle_position_offset=17'd45056;
                9'd165: triangle_position_offset=17'd45397;
                9'd166: triangle_position_offset=17'd45739;
                9'd167: triangle_position_offset=17'd46080;
                9'd168: triangle_position_offset=17'd46421;
                9'd169: triangle_position_offset=17'd46763;
                9'd170: triangle_position_offset=17'd47104;
                9'd171: triangle_position_offset=17'd47445;
                9'd172: triangle_position_offset=17'd47787;
                9'd173: triangle_position_offset=17'd48128;
                9'd174: triangle_position_offset=17'd48469;
                9'd175: triangle_position_offset=17'd48811;
                9'd176: triangle_position_offset=17'd49152;
                9'd177: triangle_position_offset=17'd49493;
                9'd178: triangle_position_offset=17'd49835;
                9'd179: triangle_position_offset=17'd50176;
                9'd180: triangle_position_offset=17'd50517;
                9'd181: triangle_position_offset=17'd50859;
                9'd182: triangle_position_offset=17'd51200;
                9'd183: triangle_position_offset=17'd51541;
                9'd184: triangle_position_offset=17'd51883;
                9'd185: triangle_position_offset=17'd52224;
                9'd186: triangle_position_offset=17'd52565;
                9'd187: triangle_position_offset=17'd52907;
                9'd188: triangle_position_offset=17'd53248;
                9'd189: triangle_position_offset=17'd53589;
                9'd190: triangle_position_offset=17'd53931;
                9'd191: triangle_position_offset=17'd54272;
                9'd192: triangle_position_offset=17'd54613;
                9'd193: triangle_position_offset=17'd54955;
                9'd194: triangle_position_offset=17'd55296;
                9'd195: triangle_position_offset=17'd55637;
                9'd196: triangle_position_offset=17'd55978;
                9'd197: triangle_position_offset=17'd56318;
                9'd198: triangle_position_offset=17'd56658;
                9'd199: triangle_position_offset=17'd56996;
                9'd200: triangle_position_offset=17'd57333;
                9'd201: triangle_position_offset=17'd57667;
                9'd202: triangle_position_offset=17'd58000;
                9'd203: triangle_position_offset=17'd58330;
                9'd204: triangle_position_offset=17'd58656;
                9'd205: triangle_position_offset=17'd58979;
                9'd206: triangle_position_offset=17'd59297;
                9'd207: triangle_position_offset=17'd59611;
                9'd208: triangle_position_offset=17'd59920;
                9'd209: triangle_position_offset=17'd60223;
                9'd210: triangle_position_offset=17'd60520;
                9'd211: triangle_position_offset=17'd60811;
                9'd212: triangle_position_offset=17'd61094;
                9'd213: triangle_position_offset=17'd61370;
                9'd214: triangle_position_offset=17'd61639;
                9'd215: triangle_position_offset=17'd61899;
                9'd216: triangle_position_offset=17'd62151;
                9'd217: triangle_position_offset=17'd62394;
                9'd218: triangle_position_offset=17'd62627;
                9'd219: triangle_position_offset=17'd62852;
                9'd220: triangle_position_offset=17'd63067;
                9'd221: triangle_position_offset=17'd63272;
                9'd222: triangle_position_offset=17'd63468;
                9'd223: triangle_position_offset=17'd63654;
                9'd224: triangle_position_offset=17'd63829;
                9'd225: triangle_position_offset=17'd63995;
                9'd226: triangle_position_offset=17'd64151;
                9'd227: triangle_position_offset=17'd64296;
                9'd228: triangle_position_offset=17'd64432;
                9'd229: triangle_position_offset=17'd64559;
                9'd230: triangle_position_offset=17'd64675;
                9'd231: triangle_position_offset=17'd64783;
                9'd232: triangle_position_offset=17'd64881;
                9'd233: triangle_position_offset=17'd64971;
                9'd234: triangle_position_offset=17'd65052;
                9'd235: triangle_position_offset=17'd65125;
                9'd236: triangle_position_offset=17'd65190;
                9'd237: triangle_position_offset=17'd65248;
                9'd238: triangle_position_offset=17'd65299;
                9'd239: triangle_position_offset=17'd65343;
                9'd240: triangle_position_offset=17'd65381;
                9'd241: triangle_position_offset=17'd65414;
                9'd242: triangle_position_offset=17'd65441;
                9'd243: triangle_position_offset=17'd65464;
                9'd244: triangle_position_offset=17'd65483;
                9'd245: triangle_position_offset=17'd65498;
                9'd246: triangle_position_offset=17'd65509;
                9'd247: triangle_position_offset=17'd65518;
                9'd248: triangle_position_offset=17'd65525;
                9'd249: triangle_position_offset=17'd65529;
                9'd250: triangle_position_offset=17'd65532;
                9'd251: triangle_position_offset=17'd65534;
                9'd252: triangle_position_offset=17'd65535;
                9'd253: triangle_position_offset=17'd65536;
                9'd254: triangle_position_offset=17'd65536;
                9'd255: triangle_position_offset=17'd65536;
                9'd256: triangle_position_offset=17'd65536;
                default: triangle_position_offset=17'd65536;
            endcase
        end
    endfunction
    function [16:0] move_offset(input [8:0] index);
        begin
            case(index)
                9'd0: move_offset=17'd0;
                9'd1: move_offset=17'd0;
                9'd2: move_offset=17'd0;
                9'd3: move_offset=17'd1;
                9'd4: move_offset=17'd2;
                9'd5: move_offset=17'd5;
                9'd6: move_offset=17'd8;
                9'd7: move_offset=17'd13;
                9'd8: move_offset=17'd19;
                9'd9: move_offset=17'd27;
                9'd10: move_offset=17'd37;
                9'd11: move_offset=17'd49;
                9'd12: move_offset=17'd63;
                9'd13: move_offset=17'd79;
                9'd14: move_offset=17'd99;
                9'd15: move_offset=17'd121;
                9'd16: move_offset=17'd145;
                9'd17: move_offset=17'd173;
                9'd18: move_offset=17'd204;
                9'd19: move_offset=17'd239;
                9'd20: move_offset=17'd277;
                9'd21: move_offset=17'd319;
                9'd22: move_offset=17'd364;
                9'd23: move_offset=17'd414;
                9'd24: move_offset=17'd467;
                9'd25: move_offset=17'd524;
                9'd26: move_offset=17'd586;
                9'd27: move_offset=17'd652;
                9'd28: move_offset=17'd723;
                9'd29: move_offset=17'd798;
                9'd30: move_offset=17'd878;
                9'd31: move_offset=17'd963;
                9'd32: move_offset=17'd1052;
                9'd33: move_offset=17'd1146;
                9'd34: move_offset=17'd1246;
                9'd35: move_offset=17'd1350;
                9'd36: move_offset=17'd1460;
                9'd37: move_offset=17'd1574;
                9'd38: move_offset=17'd1695;
                9'd39: move_offset=17'd1820;
                9'd40: move_offset=17'd1951;
                9'd41: move_offset=17'd2087;
                9'd42: move_offset=17'd2229;
                9'd43: move_offset=17'd2376;
                9'd44: move_offset=17'd2529;
                9'd45: move_offset=17'd2687;
                9'd46: move_offset=17'd2851;
                9'd47: move_offset=17'd3021;
                9'd48: move_offset=17'd3196;
                9'd49: move_offset=17'd3377;
                9'd50: move_offset=17'd3564;
                9'd51: move_offset=17'd3757;
                9'd52: move_offset=17'd3955;
                9'd53: move_offset=17'd4159;
                9'd54: move_offset=17'd4369;
                9'd55: move_offset=17'd4585;
                9'd56: move_offset=17'd4806;
                9'd57: move_offset=17'd5033;
                9'd58: move_offset=17'd5266;
                9'd59: move_offset=17'd5505;
                9'd60: move_offset=17'd5749;
                9'd61: move_offset=17'd5999;
                9'd62: move_offset=17'd6255;
                9'd63: move_offset=17'd6517;
                9'd64: move_offset=17'd6784;
                9'd65: move_offset=17'd7057;
                9'd66: move_offset=17'd7335;
                9'd67: move_offset=17'd7619;
                9'd68: move_offset=17'd7909;
                9'd69: move_offset=17'd8204;
                9'd70: move_offset=17'd8504;
                9'd71: move_offset=17'd8810;
                9'd72: move_offset=17'd9121;
                9'd73: move_offset=17'd9438;
                9'd74: move_offset=17'd9759;
                9'd75: move_offset=17'd10086;
                9'd76: move_offset=17'd10418;
                9'd77: move_offset=17'd10755;
                9'd78: move_offset=17'd11098;
                9'd79: move_offset=17'd11445;
                9'd80: move_offset=17'd11797;
                9'd81: move_offset=17'd12154;
                9'd82: move_offset=17'd12515;
                9'd83: move_offset=17'd12882;
                9'd84: move_offset=17'd13253;
                9'd85: move_offset=17'd13628;
                9'd86: move_offset=17'd14008;
                9'd87: move_offset=17'd14393;
                9'd88: move_offset=17'd14781;
                9'd89: move_offset=17'd15174;
                9'd90: move_offset=17'd15571;
                9'd91: move_offset=17'd15973;
                9'd92: move_offset=17'd16378;
                9'd93: move_offset=17'd16787;
                9'd94: move_offset=17'd17199;
                9'd95: move_offset=17'd17616;
                9'd96: move_offset=17'd18036;
                9'd97: move_offset=17'd18460;
                9'd98: move_offset=17'd18887;
                9'd99: move_offset=17'd19317;
                9'd100: move_offset=17'd19751;
                9'd101: move_offset=17'd20187;
                9'd102: move_offset=17'd20627;
                9'd103: move_offset=17'd21070;
                9'd104: move_offset=17'd21515;
                9'd105: move_offset=17'd21963;
                9'd106: move_offset=17'd22414;
                9'd107: move_offset=17'd22867;
                9'd108: move_offset=17'd23323;
                9'd109: move_offset=17'd23781;
                9'd110: move_offset=17'd24241;
                9'd111: move_offset=17'd24703;
                9'd112: move_offset=17'd25168;
                9'd113: move_offset=17'd25634;
                9'd114: move_offset=17'd26101;
                9'd115: move_offset=17'd26571;
                9'd116: move_offset=17'd27042;
                9'd117: move_offset=17'd27514;
                9'd118: move_offset=17'd27987;
                9'd119: move_offset=17'd28462;
                9'd120: move_offset=17'd28938;
                9'd121: move_offset=17'd29415;
                9'd122: move_offset=17'd29892;
                9'd123: move_offset=17'd30370;
                9'd124: move_offset=17'd30849;
                9'd125: move_offset=17'd31329;
                9'd126: move_offset=17'd31808;
                9'd127: move_offset=17'd32288;
                9'd128: move_offset=17'd32768;
                9'd129: move_offset=17'd33248;
                9'd130: move_offset=17'd33728;
                9'd131: move_offset=17'd34207;
                9'd132: move_offset=17'd34687;
                9'd133: move_offset=17'd35166;
                9'd134: move_offset=17'd35644;
                9'd135: move_offset=17'd36121;
                9'd136: move_offset=17'd36598;
                9'd137: move_offset=17'd37074;
                9'd138: move_offset=17'd37549;
                9'd139: move_offset=17'd38022;
                9'd140: move_offset=17'd38494;
                9'd141: move_offset=17'd38965;
                9'd142: move_offset=17'd39435;
                9'd143: move_offset=17'd39902;
                9'd144: move_offset=17'd40368;
                9'd145: move_offset=17'd40833;
                9'd146: move_offset=17'd41295;
                9'd147: move_offset=17'd41755;
                9'd148: move_offset=17'd42213;
                9'd149: move_offset=17'd42669;
                9'd150: move_offset=17'd43122;
                9'd151: move_offset=17'd43573;
                9'd152: move_offset=17'd44021;
                9'd153: move_offset=17'd44466;
                9'd154: move_offset=17'd44909;
                9'd155: move_offset=17'd45349;
                9'd156: move_offset=17'd45785;
                9'd157: move_offset=17'd46219;
                9'd158: move_offset=17'd46649;
                9'd159: move_offset=17'd47076;
                9'd160: move_offset=17'd47500;
                9'd161: move_offset=17'd47920;
                9'd162: move_offset=17'd48337;
                9'd163: move_offset=17'd48749;
                9'd164: move_offset=17'd49158;
                9'd165: move_offset=17'd49563;
                9'd166: move_offset=17'd49965;
                9'd167: move_offset=17'd50362;
                9'd168: move_offset=17'd50755;
                9'd169: move_offset=17'd51143;
                9'd170: move_offset=17'd51528;
                9'd171: move_offset=17'd51908;
                9'd172: move_offset=17'd52283;
                9'd173: move_offset=17'd52654;
                9'd174: move_offset=17'd53021;
                9'd175: move_offset=17'd53382;
                9'd176: move_offset=17'd53739;
                9'd177: move_offset=17'd54091;
                9'd178: move_offset=17'd54438;
                9'd179: move_offset=17'd54781;
                9'd180: move_offset=17'd55118;
                9'd181: move_offset=17'd55450;
                9'd182: move_offset=17'd55777;
                9'd183: move_offset=17'd56098;
                9'd184: move_offset=17'd56415;
                9'd185: move_offset=17'd56726;
                9'd186: move_offset=17'd57032;
                9'd187: move_offset=17'd57332;
                9'd188: move_offset=17'd57627;
                9'd189: move_offset=17'd57917;
                9'd190: move_offset=17'd58201;
                9'd191: move_offset=17'd58479;
                9'd192: move_offset=17'd58752;
                9'd193: move_offset=17'd59019;
                9'd194: move_offset=17'd59281;
                9'd195: move_offset=17'd59537;
                9'd196: move_offset=17'd59787;
                9'd197: move_offset=17'd60031;
                9'd198: move_offset=17'd60270;
                9'd199: move_offset=17'd60503;
                9'd200: move_offset=17'd60730;
                9'd201: move_offset=17'd60951;
                9'd202: move_offset=17'd61167;
                9'd203: move_offset=17'd61377;
                9'd204: move_offset=17'd61581;
                9'd205: move_offset=17'd61779;
                9'd206: move_offset=17'd61972;
                9'd207: move_offset=17'd62159;
                9'd208: move_offset=17'd62340;
                9'd209: move_offset=17'd62515;
                9'd210: move_offset=17'd62685;
                9'd211: move_offset=17'd62849;
                9'd212: move_offset=17'd63007;
                9'd213: move_offset=17'd63160;
                9'd214: move_offset=17'd63307;
                9'd215: move_offset=17'd63449;
                9'd216: move_offset=17'd63585;
                9'd217: move_offset=17'd63716;
                9'd218: move_offset=17'd63841;
                9'd219: move_offset=17'd63962;
                9'd220: move_offset=17'd64076;
                9'd221: move_offset=17'd64186;
                9'd222: move_offset=17'd64290;
                9'd223: move_offset=17'd64390;
                9'd224: move_offset=17'd64484;
                9'd225: move_offset=17'd64573;
                9'd226: move_offset=17'd64658;
                9'd227: move_offset=17'd64738;
                9'd228: move_offset=17'd64813;
                9'd229: move_offset=17'd64884;
                9'd230: move_offset=17'd64950;
                9'd231: move_offset=17'd65012;
                9'd232: move_offset=17'd65069;
                9'd233: move_offset=17'd65122;
                9'd234: move_offset=17'd65172;
                9'd235: move_offset=17'd65217;
                9'd236: move_offset=17'd65259;
                9'd237: move_offset=17'd65297;
                9'd238: move_offset=17'd65332;
                9'd239: move_offset=17'd65363;
                9'd240: move_offset=17'd65391;
                9'd241: move_offset=17'd65415;
                9'd242: move_offset=17'd65437;
                9'd243: move_offset=17'd65457;
                9'd244: move_offset=17'd65473;
                9'd245: move_offset=17'd65487;
                9'd246: move_offset=17'd65499;
                9'd247: move_offset=17'd65509;
                9'd248: move_offset=17'd65517;
                9'd249: move_offset=17'd65523;
                9'd250: move_offset=17'd65528;
                9'd251: move_offset=17'd65531;
                9'd252: move_offset=17'd65534;
                9'd253: move_offset=17'd65535;
                9'd254: move_offset=17'd65536;
                9'd255: move_offset=17'd65536;
                9'd256: move_offset=17'd65536;
                default: move_offset=17'd65536;
            endcase
        end
    endfunction
    wire [16:0] trajectory_offset_now=move_sinusoidal ? sine_position_offset(move_index) :
        move_triangular ? triangle_position_offset(move_index) : move_offset(move_index);
    wire [16:0] trajectory_offset_next=move_sinusoidal ? sine_position_offset(move_index+1'b1) :
        move_triangular ? triangle_position_offset(move_index+1'b1) : move_offset(move_index+1'b1);
    wire [31:0] trajectory_velocity_product=(trajectory_offset_next-trajectory_offset_now)*{16'b0,move_distance};
    wire [31:0] saw_sample_ticks=move_reverse ? SAW_RETURN_SAMPLE_TICKS : TRIANGLE_MOVE_SAMPLE_TICKS;
    wire [31:0] trajectory_velocity=move_sinusoidal ?
        trajectory_velocity_product/(SINE_MOVE_SAMPLE_TICKS>0 ? SINE_MOVE_SAMPLE_TICKS : 1) :
        move_triangular ? (move_reverse ?
            trajectory_velocity_product/(SAW_RETURN_SAMPLE_TICKS>0 ? SAW_RETURN_SAMPLE_TICKS : 1) :
            trajectory_velocity_product/(TRIANGLE_MOVE_SAMPLE_TICKS>0 ? TRIANGLE_MOVE_SAMPLE_TICKS : 1)) :
        trajectory_velocity_product/(POSITION_MOVE_SAMPLE_TICKS>0 ? POSITION_MOVE_SAMPLE_TICKS : 1);
    wire launch_sinusoidal=dip_trigger && !dip_mode2;
    wire launch_triangular=dip_trigger && dip_mode2;
    wire launch_bypass=launch_sinusoidal ? SINE_MOVE_SAMPLE_TICKS==0 :
        launch_triangular ? (launch_reverse ? SAW_RETURN_SAMPLE_TICKS==0 : TRIANGLE_MOVE_SAMPLE_TICKS==0) : POSITION_MOVE_SAMPLE_TICKS==0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            position_command_counts<=0;move_start<=0;move_index<=0;move_tick<=0;move_busy<=0;move_distance<=0;move_reverse<=0;move_velocity_q16<=0;move_sinusoidal<=0;move_triangular<=0;
        end else if(!position_ready || position_capture ||
                (age>=STALE_CYCLES && !sample_valid) ||
                (sample_valid && (sample_otr || tilt_exceeded || encoder_fault))) begin
            position_command_counts<=0;move_index<=0;move_tick<=0;move_busy<=0;move_velocity_q16<=0;
        end else if(move_trigger) begin
            move_sinusoidal<=dip_trigger && !dip_mode2;
            move_triangular<=launch_triangular;
            move_distance<=launch_distance;move_reverse<=launch_reverse;move_velocity_q16<=0;
            move_start<=launch_start;move_index<=0;move_tick<=0;
            move_busy<=!launch_bypass;
            position_command_counts<=launch_bypass ?
                launch_start+launch_delta : launch_start;
        end else if(move_busy && position_timer==POSITION_CYCLES-1) begin
            if(move_tick>=(move_sinusoidal ? SINE_MOVE_SAMPLE_TICKS :
                    move_triangular ? saw_sample_ticks : POSITION_MOVE_SAMPLE_TICKS)-1) begin
                move_tick<=0;
                move_index<=move_index+1'b1;
                position_command_counts<=move_reverse ?
                    move_start-$signed((trajectory_offset_next*{16'b0,move_distance})>>16) :
                    move_start+$signed((trajectory_offset_next*{16'b0,move_distance})>>16);
                move_velocity_q16<=move_reverse ?
                    -$signed(trajectory_velocity) : $signed(trajectory_velocity);
                if(move_index==255) move_busy<=0;
            end else move_tick<=move_tick+1;
        end
    end
    assign position_ready=pid_active && position_loop_enabled && position_engaged && !stop;
    // Absolute encoder coordinate for plotting against live ENC1.
    assign position_target_encoder=position_ready ?
        reference+(curve_display ? position_command_counts : position_target_counts) : encoder;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) curve_display<=0;
        else if(!position_ready || position_capture) curve_display<=0;
        else if(move_trigger) curve_display<=dip_trigger;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) position_target_counts<=0;
        else if(!position_ready || position_capture ||
                (age>=STALE_CYCLES && !sample_valid) ||
                (sample_valid && (sample_otr || tilt_exceeded || encoder_fault)))
            position_target_counts<=0;
        // Rebase each press on live position, even during an unfinished move.
        else if(move_trigger)
            position_target_counts<=launch_start+launch_delta;
    end
    // Encoder window advances at the OUTER tick, independently of ADC cadence.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(velocity_index=0;velocity_index<VELOCITY_SAMPLES;velocity_index=velocity_index+1)
                begin velocity_history[velocity_index]<=0;command_history[velocity_index]<=0;end
        end else if(!pid_active || !position_loop_enabled || !position_engaged || stop || position_capture ||
                    (sample_valid && (sample_otr || tilt_exceeded || encoder_fault))) begin
            for(velocity_index=0;velocity_index<VELOCITY_SAMPLES;velocity_index=velocity_index+1)
                begin velocity_history[velocity_index]<=0;command_history[velocity_index]<=0;end
        end else if(position_timer==POSITION_CYCLES-1) begin
            velocity_history[0]<=position_measured;
            command_history[0]<=position_command_counts;
            for(velocity_index=1;velocity_index<VELOCITY_SAMPLES;velocity_index=velocity_index+1)
                begin velocity_history[velocity_index]<=velocity_history[velocity_index-1];
                    command_history[velocity_index]<=command_history[velocity_index-1];end
        end
    end
    // P (and optional outer I) retains the old slew rate. D bypasses that
    // rate limiter but keeps its 30ms measurement window and a degree cap.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            position_p_request_q8<=0;position_d_request_q8<=0;position_p_slew_q8<=0;
            position_p_scaled<=0;position_d_scaled<=0;
        end else if(!pid_active || !position_loop_enabled || !position_engaged || stop || position_capture ||
                    (sample_valid && (sample_otr || tilt_exceeded || encoder_fault))) begin
            position_p_request_q8<=0;position_d_request_q8<=0;position_p_slew_q8<=0;
            position_p_scaled<=0;position_d_scaled<=0;
        end else begin
            position_p_scaled<=position_pi_limited*$signed({1'b0,ramp_gain});
            position_d_scaled<=position_d_limited*$signed({1'b0,ramp_gain});
            if(position_valid[8]) begin
                position_p_request_q8<=position_p_scaled>>>10;
                position_d_request_q8<=position_d_scaled>>>10;
            end
            if(sample_valid) position_p_slew_q8<=position_p_next;
        end
    end
    // Capture exactly the operands consumed on the final PWM update edge.
    // Angle includes polarity/boost; I is the angle integral before polarity.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            diag_angle_pwm_q8<=0; diag_i_q8<=0; diag_position_pwm_q8<=0;
        end else if(!pid_active || stop ||
                    (age>=STALE_CYCLES && !sample_valid) ||
                    (sample_valid && (sample_otr || tilt_exceeded || encoder_fault))) begin
            diag_angle_pwm_q8<=0; diag_i_q8<=0; diag_position_pwm_q8<=0;
        end else if(calc_valid[7]) begin
            diag_angle_pwm_q8<=(boost_enabled && quiet_pipe[7] && position_pwm_q8==0) ? 0 : angle_mapped;
            diag_i_q8<=i_term;
            diag_position_pwm_q8<=0; // Cascade has no direct position PWM branch.
        end
    end
    // Stop/fault clears the diagnostics together with the motor request.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            diag_rate_q8<=0; diag_p_q8<=0; diag_d_q8<=0;
            diag_rate_pending<=0; diag_p_pending<=0; diag_d_pending<=0;
        end else if(!pid_active || stop ||
                    (age>=STALE_CYCLES && !sample_valid) ||
                    (sample_valid && (sample_otr || tilt_exceeded || encoder_fault))) begin
            diag_rate_q8<=0; diag_p_q8<=0; diag_d_q8<=0;
            diag_rate_pending<=0; diag_p_pending<=0; diag_d_pending<=0;
        end else begin
            if(calc_valid[5]) begin
                diag_rate_pending<=derivative_q8;
                diag_p_pending<=p_term; diag_d_pending<=d_term;
            end
            if(calc_valid[7]) begin
                diag_rate_q8<=diag_rate_pending;
                diag_p_q8<=diag_p_pending; diag_d_q8<=diag_d_pending;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            running<=0; fault<=0; polarity_set<=0; polarity_reverse<=0;
            swing_active<=0; arm_swing<=0; auto_swing_session<=0;
            boost_enabled<=0;
            position_loop_enabled<=1;
            position_kp_q12<=16'd34; position_kd_q8<=16'd729; // 240/256 = 0.9375 at 10 ms
            position_ki_q12<=POSITION_KI_Q12;
            target_angle_code<=ZERO; target_offset_q8<=0; target_request_q8<=0;
            boost_pending<=0; boost_negative<=0; boost_remaining<=0;
            limit_percent<=90; kp_q8<=960; kd_q8<=8192;
            ki_q8<=16'd5; integral_acc<=0; i_term<=0; // 5/256 = 0.01953125
            integral_sum_pending<=0; integral_product_reg<=0;
            integral_product_lo<=0; integral_product_hi<=0;
            p_product_reg<=0; d_product_reg<=0;
            angle_error_q16<=0; angle_delta_q16<=0;
            d_raw_q16<=0; d_filter_sum_reg<=0; d_filter_valid<=0;
            quiet_filter_pipe<=0;
            previous_upright<=0; arm_delta_code<=0;
            error_code<=0; control_error_code<=0; delta_code<=0; position<=0; output_q8<=0;
            age<=STALE_CYCLES; sample_seen<=0; ready_samples<=0;
            arm_pending<=0; reference<=0; errors_start<=0;
            previous_error<=0; derivative_q8<=0;
            previous_position<=0; position_velocity_q8<=0;
            previous_angle_measurement_q16<=0; position_timer<=0; position_valid<=0;
            position_engaged<=0; position_stable_samples<=0;
            ramp_step<=0; position_scaled<=0;
            position_integral<=0; position_i_term_q8<=0; position_i_product<=0;
            target_quotient<=0; target_remainder<=0; target_divide_count<=0;
            target_negative<=0; target_converted_offset<=0;
            position_p_term_q8<=0; position_d_term_q8<=0; position_d_product_reg<=0;
            position_d_magnitude<=0;
            position_d_negative_a<=0;position_d_negative_b<=0;position_d_negative_c<=0;
            position_d_upper_quotient<=0;position_d_upper_delay<=0;
            position_d_remainder<=0;position_d_lower<=0;position_d_lower_quotient<=0;
            position_operand<=0; velocity_operand<=0;
            position_pwm_raw<=0; position_pwm_q8<=0;
            calc_valid<=0; quiet_pipe<=0; p_term<=0; d_term<=0; effort<=0; boosted_effort<=0;
            close_code<=0; move_counts<=0; effect<=1;
            effect_samples<=0; effect_start_abs<=0; effect_start_pos<=0;
        end else begin
            // Register sampled position PD products before their sum/clamp.
            // Convert signed Q12 product to Q8, truncating symmetrically toward zero.
            position_p_term_q8<=($signed(position_p_product)>>>4)+
                ((position_p_product[32] && |position_p_product[3:0]) ? 32'sd1 : 32'sd0);
            position_d_product_reg<=position_d_product;
            position_d_magnitude<=position_d_product_reg[32] ? -position_d_product_reg : position_d_product_reg;
            position_d_negative_a<=position_d_product_reg[32];
            position_d_upper_quotient<=position_d_magnitude[32:16] / VELOCITY_SAMPLES;
            position_d_remainder<=position_d_magnitude[32:16] % VELOCITY_SAMPLES;
            position_d_lower<=position_d_magnitude[15:0];
            position_d_negative_b<=position_d_negative_a;
            position_d_upper_delay<=position_d_upper_quotient;
            position_d_lower_quotient<=position_d_lower_dividend / VELOCITY_SAMPLES;
            position_d_negative_c<=position_d_negative_b;
            position_d_term_q8<=position_d_negative_c ?
                -$signed({position_d_upper_delay,position_d_lower_quotient}) :
                 $signed({position_d_upper_delay,position_d_lower_quotient});
            position_pwm_raw<=-($signed(position_p_term_q8)+$signed(position_d_term_q8)+$signed(position_i_term_q8));
            position_pwm_q8<=0;
            position_valid<={position_valid[7:0],1'b0};
            position_scaled<=position_limited*$signed({1'b0,ramp_gain});
            if(!pid_active || !position_loop_enabled || stop) begin
                position_engaged<=0; position_stable_samples<=0; ramp_step<=0;
            end else if(!position_engaged && sample_valid) begin
                if(position_bound_ready && !sample_otr && !tilt_exceeded && !encoder_fault && position_angle_ok && position_rate_ok) begin
                    if(position_stable_samples<POSITION_ENGAGE_SAMPLES)
                        position_stable_samples<=position_stable_samples+1'b1;
                end else position_stable_samples<=0;
            end
            if(pid_active && position_loop_enabled && position_engaged) begin
                if(position_valid[0]) begin
                    position_integral<=position_sum_clamped[31:0];
                end
                if(position_valid[1])
                    position_i_product<=$signed(position_integral[POSITION_SUM_BITS-1:0])*$signed({1'b0,position_ki_q12});
                // Q12 gain times integer error sum -> Q8 degrees. Preserve
                // fractional accumulation and truncate both signs toward zero.
                if(position_valid[2])
                    position_i_term_q8<=($signed(position_i_product)>>>4)+
                        ((position_i_product[POSITION_SUM_BITS+16] && |position_i_product[3:0]) ? 32'sd1 : 32'sd0);
                if(position_timer==POSITION_CYCLES-1) begin
                    position_timer<=0; position_valid[0]<=1;
                    if(ramp_step<RAMP_STEPS) ramp_step<=ramp_step+1'b1;
                    position<=pos_now; previous_position<=pos_now;
                    position_velocity_q8<=($signed({{8{position_difference[31]}},position_difference})*40'sd256)/VELOCITY_SAMPLES;
                    position_operand<=pos_now>32767 ? 16'sd32767 :
                        pos_now< -32767 ? -16'sd32767 : pos_now[15:0];
                    velocity_operand<=tracked_velocity_difference>32767 ? 16'sd32767 :
                        tracked_velocity_difference< -32767 ? -16'sd32767 : tracked_velocity_difference[15:0];
                end else position_timer<=position_timer+1'b1;
                if(position_valid[8]) begin
                    target_request_q8<=($signed(position_p_scaled)+$signed(position_d_scaled))>>>10;
                end
                // Apply the slewed P plus bounded direct D on each fresh ADC sample.
                if(sample_valid) begin
                    target_offset_q8<=position_target_limited[15:0];
                end
                // ADC target is telemetry only; 24-cycle exact restoring divider.
                if(target_divide_count!=0) begin
                    target_quotient<=target_quotient_next;
                    target_remainder<=target_divide_bit ? target_trial-17'd22086 : target_trial;
                    target_divide_count<=target_divide_count-1'b1;
                    if(target_divide_count==1)
                        target_angle_code<=target_negative ? ZERO_CODE-$signed(target_quotient_next[15:0]) :
                            ZERO_CODE+$signed(target_quotient_next[15:0]);
                end else if(target_converted_offset!=target_offset_q8) begin
                    target_converted_offset<=target_offset_q8;
                    target_negative<=target_offset_q8[15];
                    target_quotient<=(target_offset_q8<0 ? -target_offset_q8 : target_offset_q8)*24'd256;
                    target_remainder<=0; target_divide_count<=24;
                end
            end
            if(!pid_active || !position_loop_enabled || !position_engaged) begin
                position_timer<=0; position_valid<=0;
                position_integral<=0; position_i_term_q8<=0; position_i_product<=0;
                target_divide_count<=0; target_converted_offset<=0;
                target_offset_q8<=0; target_angle_code<=ZERO; target_request_q8<=0;
                position_operand<=0; velocity_operand<=0;
                position_p_term_q8<=0; position_d_term_q8<=0; position_d_product_reg<=0;
                position_pwm_raw<=0; position_pwm_q8<=0;
            end
            d_filter_valid<={d_filter_valid[0],pid_tick};
            if(d_filter_valid[0]) d_filter_sum_reg<=d_filter_sum;
            if(d_filter_valid[1]) angle_delta_q16<=d_filtered_next;
            calc_valid<={calc_valid[6:0],d_filter_valid[1]};
            // Suppress near-upright ADC noise, but retain damping at speed.
            // Carry the decision alongside the PD result from the same sample.
            quiet_filter_pipe<={quiet_filter_pipe[0],ki_q8==0 && abs_control_error<=2 &&
                         derivative_next>=-64 && derivative_next<=64};
            quiet_pipe<={quiet_pipe[6:0],quiet_filter_pipe[1]};
            p_product_reg<=p_product; d_product_reg<=d_product;
            p_term<=p_calc; d_term<=d_scaled[31:0]; effort<=pid_effort;
            if(!pid_active || ki_q8==0) begin
                integral_acc<=0; i_term<=0;
            end else begin
                // Separate sum, clamp, multiply and Q24->Q8 rounding stages.
                // Then effort, boost, and output; all use this sample's I.
                if(calc_valid[0]) integral_sum_pending<=integral_next;
                if(calc_valid[1]) integral_acc<=integral_clamped[31:0];
                if(calc_valid[2]) begin integral_product_lo<=integral_lo; integral_product_hi<=integral_hi; end
                if(calc_valid[3]) integral_product_reg<=integral_product;
                if(calc_valid[4]) i_term<=integral_output_limited;
            end
            if(use_start_boost && effort>0) boosted_effort<=effort+32'sd6400;
            else if(use_start_boost && effort<0) boosted_effort<=effort-32'sd6400;
            else boosted_effort<=effort;
            if(sample_valid && boost_remaining!=0) boost_remaining<=boost_remaining-1'b1;
            if(pid_active && calc_valid[6]) begin
                if(boost_pending && use_start_boost) begin
                    boost_pending<=0;
                    boost_negative<=effort[31];
                    boost_remaining<=START_BOOST_SAMPLES;
                end else if(!use_start_boost) boost_remaining<=0;
            end
            if(!pid_active) begin
                calc_valid<=0; angle_delta_q16<=0;
                d_raw_q16<=0; d_filter_sum_reg<=0; d_filter_valid<=0;
            end
            if(running && swing_active) output_q8<=swing_drive;
            if(pid_active && calc_valid[7]) begin
                if(boost_enabled && quiet_pipe[7] && position_pwm_q8==0) output_q8<=0;
                else if(mapped>bound) output_q8<=bound[15:0];
                else if(mapped< -bound) output_q8<= -$signed(bound[15:0]);
                else output_q8<=mapped[15:0];
            end
            if(age<STALE_CYCLES) age<=age+1'b1;
            if(sample_valid) begin
                age<=0; sample_seen<=1; error_code<=upright_error_now;
                previous_upright<=upright_error_now;
                arm_delta_code<=upright_error_now-previous_upright;
                if(!sample_seen || age>=STALE_CYCLES) begin
                    derivative_q8<=0; delta_code<=0; ready_samples<=0; arm_delta_code<=0;
                end else begin
                    if(ready_samples!=255) ready_samples<=ready_samples+1'b1;
                end
                if(pid_tick) begin
                    control_error_code<=control_error_now;
                    previous_error<=control_error_now;
                    angle_error_q16<=angle_error_now_q16;
                    d_raw_q16<=measured_delta_q16;
                    previous_angle_measurement_q16<=angle_measurement_q16;
                    derivative_q8<=derivative_next[19:0]; delta_code<=difference;
                end
            end
            // All six PID gains and presets accept commands while running.
            if(cmd_valid && !stop) begin
                case(cmd)
                    "P": if(kp_q8<1024) kp_q8<=kp_q8+16'd32;
                    "p": if(kp_q8>=32) kp_q8<=kp_q8-16'd32;
                    "I": if(ki_q8<1024) ki_q8<=ki_q8+16'd1;
                    "i": if(ki_q8>=1) ki_q8<=ki_q8-16'd1;
                    "J": if(position_ki_q12<16384) position_ki_q12<=position_ki_q12+1'b1;
                    "j": if(position_ki_q12!=0) position_ki_q12<=position_ki_q12-1'b1;
                    "V": if(position_kd_q8<1024) position_kd_q8<=position_kd_q8+16'd1;
                    "v": if(position_kd_q8!=0) position_kd_q8<=position_kd_q8-16'd1;
                    "k": begin // Image 1: position KP=123/4096.
                        kp_q8<=16'd960;ki_q8<=16'd5;kd_q8<=16'd20800;
                        position_kp_q12<=16'd123;position_ki_q12<=0;position_kd_q8<=16'd729;
                    end
                    "l": begin // Image 2: position KP=132/4096.
                        kp_q8<=16'd960;ki_q8<=16'd5;kd_q8<=16'd20800;
                        position_kp_q12<=16'd132;position_ki_q12<=0;position_kd_q8<=16'd729;
                    end
                    "D": begin
                        if(kd_q8<16'd32768)
                            kd_q8<=(kd_q8<=16'd32736) ? kd_q8+16'd32 : 16'd32768;
                    end
                    "d": begin
                        if(kd_q8>=32) kd_q8<=kd_q8-16'd32;
                    end
                    "A": begin
                        if(position_kp_q12<16384) position_kp_q12<=position_kp_q12+1'b1;
                    end
                    "a": begin
                        if(position_kp_q12!=0) position_kp_q12<=position_kp_q12-1'b1;
                    end
                    default: begin end
                endcase
            end
            // All other configuration is accepted only when disarmed.
            if(!running && cmd_valid) begin
                case(cmd)
                    "+": begin polarity_set<=1; polarity_reverse<=0; end
                    "-": begin polarity_set<=1; polarity_reverse<=1; end
                    "c": boost_enabled<=1;
                    "C": boost_enabled<=0;
                    "O": position_loop_enabled<=1;
                    "o": position_loop_enabled<=0;
                    "1": limit_percent<=8'd5;
                    "2": limit_percent<=8'd10;
                    "3": limit_percent<=8'd15;
                    "4": limit_percent<=8'd20;
                    "5": limit_percent<=8'd25;
                    "6": limit_percent<=8'd30;
                    "7": limit_percent<=8'd35;
                    "8": limit_percent<=8'd40;
                    "9": limit_percent<=8'd45;
                    "0": limit_percent<=8'd90;
                    "b": begin arm_pending<=1; arm_swing<=0; end
                    "w": begin arm_pending<=1; arm_swing<=1; end
                    default: begin end
                endcase
            end
            if(stop) begin
                angle_delta_q16<=0;
                d_filter_valid<=0; d_filter_sum_reg<=0; d_raw_q16<=0;
                target_request_q8<=0;
                swing_active<=0; arm_swing<=0; auto_swing_session<=0;
                integral_acc<=0; i_term<=0;
                target_angle_code<=ZERO; target_offset_q8<=0;
                previous_position<=0; position_velocity_q8<=0;
                boost_pending<=0; boost_remaining<=0;
                running<=0; output_q8<=0; arm_pending<=0; fault<=!stop_key_n ? 4'd9 : 4'd0; calc_valid<=0;
                effect_samples<=0; close_code<=0; move_counts<=0; effect<=1;
            end else if(!running && cmd_valid && cmd=="w") begin
                kp_q8<=16'd960;ki_q8<=16'd5;kd_q8<=16'd8192;
                position_kp_q12<=16'd34;position_ki_q12<=0;position_kd_q8<=16'd729;
                // SW1 and UART w start immediately, even with no ADC data/OTR.
                // Angle sensing begins only AFTER the first 100 ms kick.
                arm_pending<=0; arm_swing<=0; calc_valid<=0; quiet_pipe<=0;
                integral_acc<=0; i_term<=0; boost_pending<=0; boost_remaining<=0;
                target_angle_code<=ZERO; target_offset_q8<=0;
                previous_position<=0; position_velocity_q8<=0;
                position<=0; errors_start<=encoder_errors;
                effect_samples<=0; close_code<=0; move_counts<=0; effect<=1;
                if(limit_percent<50) begin
                    running<=0; swing_active<=0; auto_swing_session<=0; output_q8<=0; fault<=8;
                end else begin
                    auto_swing_session<=1;
                    position_loop_enabled<=0; // Re-enable qualification only after swing capture.
                    running<=1; swing_active<=1; output_q8<=-16'sd12800; fault<=0;
                    polarity_set<=1; polarity_reverse<=0;
                    boost_enabled<=0;
                end
            end else if(!startup_kick && age>=STALE_CYCLES && !sample_valid) begin
                angle_delta_q16<=0;
                d_filter_valid<=0; d_filter_sum_reg<=0; d_raw_q16<=0;
                swing_active<=0; arm_swing<=0; auto_swing_session<=0;
                integral_acc<=0; i_term<=0;
                target_angle_code<=ZERO; target_offset_q8<=0;
                previous_position<=0; position_velocity_q8<=0;
                boost_pending<=0; boost_remaining<=0;
                if(running || arm_pending) fault<=4;
                running<=0; output_q8<=0; arm_pending<=0; calc_valid<=0;
                effect_samples<=0; close_code<=0; move_counts<=0; effect<=1;
            end else if(sample_valid) begin
                if(!running) begin
                    output_q8<=0;
                    if(arm_pending) begin
                        arm_pending<=0;
                        if(!arm_swing && !polarity_set) fault<=1;
                        else if((arm_swing && limit_percent<50) || (!arm_swing && boost_enabled && limit_percent<25)) fault<=8;
                        // Arm from the preceding complete measurement.
                        // The current sample is captured below for the next update.
                        else if(ready_samples<20) fault<=10;
                        else if(!integral_bound_ready) fault<=11;
                        else if(!arm_swing && (abs_registered_error>ARM_ERROR_CODES ||
                                arm_delta_code>2*PID_PERIOD_MS || arm_delta_code< -2*PID_PERIOD_MS)) fault<=2;
                        else if(sample_otr) fault<=3;
                        else begin
                            integral_acc<=0; i_term<=0;
                            running<=1; fault<=0;
                            swing_active<=arm_swing;
                            auto_swing_session<=arm_swing;
                            if(arm_swing) begin
                                kp_q8<=16'd960;ki_q8<=16'd5;kd_q8<=16'd8192;
                                position_kp_q12<=16'd34;position_ki_q12<=0;position_kd_q8<=16'd729;
                                position_loop_enabled<=0;
                                // Known motor/sensor direction; position acts only after capture.
                                polarity_set<=1; polarity_reverse<=0;
                                boost_enabled<=0;
                            end
                            boost_pending<=!arm_swing && boost_enabled; boost_remaining<=0;
                            position<=0; errors_start<=encoder_errors;
                            target_angle_code<=ZERO; target_offset_q8<=0;
                            previous_position<=0; position_velocity_q8<=0;
                            control_error_code<=upright_error_now;
                            derivative_q8<=0; previous_error<=upright_error_now;
                            previous_angle_measurement_q16<=angle_measurement_q16;
                            effect_samples<=0; close_code<=0; move_counts<=0; effect<=1;
                        end
                    end
                end else begin
                    // No relative position is published before reference capture.
                    position<=position_engaged ? pos_now : 0;
                    if(!swing_active && (sample_otr || tilt_exceeded || encoder_fault)) begin
                        d_filter_valid<=0; d_filter_sum_reg<=0; d_raw_q16<=0;
                        target_request_q8<=0;
                        // Losing balance always stops; a new w is required to swing again.
                        running<=0; swing_active<=0; auto_swing_session<=0;
                        if(sample_otr) fault<=3;
                        else if(encoder_fault) fault<=7;
                        else fault<=5;
                        integral_acc<=0; i_term<=0;
                        integral_sum_pending<=0; integral_product_reg<=0;
                        integral_product_lo<=0; integral_product_hi<=0;
                        p_product_reg<=0; d_product_reg<=0; p_term<=0; d_term<=0;
                        effort<=0; boosted_effort<=0; quiet_pipe<=0;
                        angle_error_q16<=0; angle_delta_q16<=0;
                        previous_error<=0; previous_angle_measurement_q16<=0;
                        control_error_code<=0; derivative_q8<=0; delta_code<=0;
                        target_angle_code<=ZERO; target_offset_q8<=0;
                        target_divide_count<=0; target_converted_offset<=0;
                        previous_position<=0; position_velocity_q8<=0;
                        position_engaged<=0; position_stable_samples<=0; ramp_step<=0;
                        position_timer<=0; position_valid<=0; position_scaled<=0;
                        position_integral<=0; position_i_term_q8<=0; position_i_product<=0;
                        position_operand<=0; velocity_operand<=0;
                        position_p_term_q8<=0; position_d_term_q8<=0; position_d_product_reg<=0;
                        position_pwm_raw<=0; position_pwm_q8<=0;
                        output_q8<=0; calc_valid<=0;
                        boost_pending<=0; boost_remaining<=0;
                        effect_samples<=0; close_code<=0; move_counts<=0; effect<=1;
                    end else if(effect_samples==0) begin
                        effect_start_abs<=abs_upright_error;
                        effect_start_pos<=position_measured;
                        effect_samples<=1;
                    end else if(effect_samples==EFFECT_SAMPLES) begin
                        close_code<=window_close;
                        move_counts<=window_move;
                        if(abs_upright_error<=3) effect<=2; // CENTER
                        else if(window_move<2 && window_move> -2) effect<=3; // NO_MOTION
                        else if(window_close>=3) effect<=4; // RETURN
                        else if(window_close<= -3) effect<=5; // AWAY
                        else effect<=6; // FLAT
                        effect_start_abs<=abs_upright_error;
                        effect_start_pos<=position_measured;
                        effect_samples<=1;
                    end else begin
                        effect_samples<=effect_samples+1'b1;
                    end
                end
            end
            // Fresh ADC angle/speed capture follows the qibai swing supervisor.
            // Stop, timeout and overrange take priority over PID handoff.
            if(running && swing_active && swing_capture && !stop &&
               (age<STALE_CYCLES || sample_valid) && !sample_otr) begin
                if(auto_swing_session) position_loop_enabled<=1;
                target_request_q8<=0;
                swing_active<=0; output_q8<=0; calc_valid<=0; quiet_pipe<=0;
                integral_acc<=0; i_term<=0; integral_sum_pending<=0;
                integral_product_reg<=0; integral_product_lo<=0; integral_product_hi<=0;
                p_product_reg<=0; d_product_reg<=0; p_term<=0; d_term<=0;
                effort<=0; boosted_effort<=0;
                angle_error_q16<=0; angle_delta_q16<=0;
                target_angle_code<=ZERO; target_offset_q8<=0;
                previous_position<=0; position_velocity_q8<=0;
                position<=0; errors_start<=encoder_errors;
                position_operand<=0; velocity_operand<=0;
                position_p_term_q8<=0; position_d_term_q8<=0; position_d_product_reg<=0;
                position_pwm_raw<=0; position_pwm_q8<=0;
                previous_error<=capture_error; control_error_code<=capture_error;
                previous_angle_measurement_q16<=$signed(capture_error[11:0])*27'sd22086;
                position_timer<=0; position_valid<=0;
                position_integral<=0; position_i_term_q8<=0; position_i_product<=0;
                position_engaged<=0; position_stable_samples<=0; ramp_step<=0;
                derivative_q8<=0; delta_code<=0;
                boost_pending<=0; boost_remaining<=0;
                effect_samples<=0; close_code<=0; move_counts<=0; effect<=1;
            end
            // On the fifth consecutive fresh in-band state4 sample, capture
            // the encoder reference and enable the outer loop on the same edge.
            // Preserve the angle PID state that is already balancing the rod.
            if(position_capture) begin
                // Target counts and trajectory are zeroed on this same edge;
                // capturing the reference therefore holds the current actual position.
                position_engaged<=1; reference<=encoder; position<=0;
                previous_position<=0; position_velocity_q8<=0; errors_start<=encoder_errors;
                position_timer<=0; position_valid<=0; position_scaled<=0; ramp_step<=0;
                position_operand<=0; velocity_operand<=0;
                position_integral<=0; position_i_term_q8<=0; position_i_product<=0;
                position_p_term_q8<=0; position_d_term_q8<=0; position_d_product_reg<=0; position_pwm_raw<=0;
            end
        end
    end
endmodule

// One shot: +180 degrees, hold, -270 degrees, hold, +180 degrees, final hold.
// Unwrapped encoder coordinates preserve the full -270 degree reverse path.
// Legacy module/instance names are retained for the existing DIP and diagnostics.
module sine_velocity_profile #(
    parameter integer PERIOD_MS=10,
    parameter integer COUNTS_PER_REV=1040,
    parameter integer SPEED_MDEG_S=10000,
    parameter integer RAMP_MS=2000,
    parameter integer HOLD_MS=3000
)(
    input clk,rst_n,ready,tick,enable,
    input signed [31:0] actual,
    output reg active,
    output signed [31:0] target,
    output reg signed [31:0] velocity_q16,
    output signed [31:0] goal
);
    localparam integer RAMP_TICKS=RAMP_MS/PERIOD_MS;
    localparam integer CRUISE_180_TICKS=180000000/(SPEED_MDEG_S*PERIOD_MS)-RAMP_TICKS;
    localparam integer CRUISE_270_TICKS=270000000/(SPEED_MDEG_S*PERIOD_MS)-RAMP_TICKS;
    localparam integer HOLD_TICKS=HOLD_MS/PERIOD_MS;
    localparam integer COUNTS_180=(COUNTS_PER_REV+1)/2;
    localparam integer COUNTS_270=(COUNTS_PER_REV*3+2)/4;
    localparam integer PEAK_Q16=(64'd65536*COUNTS_PER_REV*SPEED_MDEG_S*PERIOD_MS)/360000000;
    // Two seconds to stop from peak, with signed velocity.
    localparam integer STOP_STEP=(PEAK_Q16+RAMP_TICKS-1)/RAMP_TICKS;
    reg armed;
    reg stopping;
    reg [2:0] stage;
    reg [31:0] elapsed;
    reg signed [47:0] position_q16;
    reg signed [31:0] endpoint;
    wire moving=stage==0 || stage==2 || stage==4;
    wire [31:0] cruise_ticks=stage==2 ? CRUISE_270_TICKS : CRUISE_180_TICKS;
    wire [31:0] end_tick=2*RAMP_TICKS+cruise_ticks;
    function [16:0] ramp(input [7:0] index);
        begin
            case(index)
                8'd0: ramp=17'd0;
                8'd1: ramp=17'd5;
                8'd2: ramp=17'd20;
                8'd3: ramp=17'd44;
                8'd4: ramp=17'd78;
                8'd5: ramp=17'd121;
                8'd6: ramp=17'd173;
                8'd7: ramp=17'd235;
                8'd8: ramp=17'd306;
                8'd9: ramp=17'd386;
                8'd10: ramp=17'd475;
                8'd11: ramp=17'd573;
                8'd12: ramp=17'd679;
                8'd13: ramp=17'd795;
                8'd14: ramp=17'd918;
                8'd15: ramp=17'd1051;
                8'd16: ramp=17'd1191;
                8'd17: ramp=17'd1340;
                8'd18: ramp=17'd1497;
                8'd19: ramp=17'd1662;
                8'd20: ramp=17'd1835;
                8'd21: ramp=17'd2016;
                8'd22: ramp=17'd2204;
                8'd23: ramp=17'd2401;
                8'd24: ramp=17'd2605;
                8'd25: ramp=17'd2816;
                8'd26: ramp=17'd3035;
                8'd27: ramp=17'd3261;
                8'd28: ramp=17'd3494;
                8'd29: ramp=17'd3734;
                8'd30: ramp=17'd3981;
                8'd31: ramp=17'd4235;
                8'd32: ramp=17'd4496;
                8'd33: ramp=17'd4764;
                8'd34: ramp=17'd5038;
                8'd35: ramp=17'd5319;
                8'd36: ramp=17'd5606;
                8'd37: ramp=17'd5899;
                8'd38: ramp=17'd6199;
                8'd39: ramp=17'd6504;
                8'd40: ramp=17'd6816;
                8'd41: ramp=17'd7133;
                8'd42: ramp=17'd7457;
                8'd43: ramp=17'd7786;
                8'd44: ramp=17'd8120;
                8'd45: ramp=17'd8460;
                8'd46: ramp=17'd8806;
                8'd47: ramp=17'd9157;
                8'd48: ramp=17'd9513;
                8'd49: ramp=17'd9874;
                8'd50: ramp=17'd10240;
                8'd51: ramp=17'd10611;
                8'd52: ramp=17'd10987;
                8'd53: ramp=17'd11368;
                8'd54: ramp=17'd11753;
                8'd55: ramp=17'd12143;
                8'd56: ramp=17'd12537;
                8'd57: ramp=17'd12935;
                8'd58: ramp=17'd13338;
                8'd59: ramp=17'd13745;
                8'd60: ramp=17'd14156;
                8'd61: ramp=17'd14571;
                8'd62: ramp=17'd14989;
                8'd63: ramp=17'd15412;
                8'd64: ramp=17'd15838;
                8'd65: ramp=17'd16267;
                8'd66: ramp=17'd16700;
                8'd67: ramp=17'd17137;
                8'd68: ramp=17'd17576;
                8'd69: ramp=17'd18019;
                8'd70: ramp=17'd18465;
                8'd71: ramp=17'd18914;
                8'd72: ramp=17'd19365;
                8'd73: ramp=17'd19819;
                8'd74: ramp=17'd20276;
                8'd75: ramp=17'd20736;
                8'd76: ramp=17'd21198;
                8'd77: ramp=17'd21662;
                8'd78: ramp=17'd22129;
                8'd79: ramp=17'd22598;
                8'd80: ramp=17'd23069;
                8'd81: ramp=17'd23541;
                8'd82: ramp=17'd24016;
                8'd83: ramp=17'd24493;
                8'd84: ramp=17'd24971;
                8'd85: ramp=17'd25450;
                8'd86: ramp=17'd25932;
                8'd87: ramp=17'd26414;
                8'd88: ramp=17'd26898;
                8'd89: ramp=17'd27383;
                8'd90: ramp=17'd27869;
                8'd91: ramp=17'd28356;
                8'd92: ramp=17'd28844;
                8'd93: ramp=17'd29333;
                8'd94: ramp=17'd29822;
                8'd95: ramp=17'd30312;
                8'd96: ramp=17'd30803;
                8'd97: ramp=17'd31294;
                8'd98: ramp=17'd31785;
                8'd99: ramp=17'd32276;
                8'd100: ramp=17'd32768;
                8'd101: ramp=17'd33260;
                8'd102: ramp=17'd33751;
                8'd103: ramp=17'd34242;
                8'd104: ramp=17'd34733;
                8'd105: ramp=17'd35224;
                8'd106: ramp=17'd35714;
                8'd107: ramp=17'd36203;
                8'd108: ramp=17'd36692;
                8'd109: ramp=17'd37180;
                8'd110: ramp=17'd37667;
                8'd111: ramp=17'd38153;
                8'd112: ramp=17'd38638;
                8'd113: ramp=17'd39122;
                8'd114: ramp=17'd39604;
                8'd115: ramp=17'd40086;
                8'd116: ramp=17'd40565;
                8'd117: ramp=17'd41043;
                8'd118: ramp=17'd41520;
                8'd119: ramp=17'd41995;
                8'd120: ramp=17'd42467;
                8'd121: ramp=17'd42938;
                8'd122: ramp=17'd43407;
                8'd123: ramp=17'd43874;
                8'd124: ramp=17'd44338;
                8'd125: ramp=17'd44800;
                8'd126: ramp=17'd45260;
                8'd127: ramp=17'd45717;
                8'd128: ramp=17'd46171;
                8'd129: ramp=17'd46622;
                8'd130: ramp=17'd47071;
                8'd131: ramp=17'd47517;
                8'd132: ramp=17'd47960;
                8'd133: ramp=17'd48399;
                8'd134: ramp=17'd48836;
                8'd135: ramp=17'd49269;
                8'd136: ramp=17'd49698;
                8'd137: ramp=17'd50124;
                8'd138: ramp=17'd50547;
                8'd139: ramp=17'd50965;
                8'd140: ramp=17'd51380;
                8'd141: ramp=17'd51791;
                8'd142: ramp=17'd52198;
                8'd143: ramp=17'd52601;
                8'd144: ramp=17'd52999;
                8'd145: ramp=17'd53393;
                8'd146: ramp=17'd53783;
                8'd147: ramp=17'd54168;
                8'd148: ramp=17'd54549;
                8'd149: ramp=17'd54925;
                8'd150: ramp=17'd55296;
                8'd151: ramp=17'd55662;
                8'd152: ramp=17'd56023;
                8'd153: ramp=17'd56379;
                8'd154: ramp=17'd56730;
                8'd155: ramp=17'd57076;
                8'd156: ramp=17'd57416;
                8'd157: ramp=17'd57750;
                8'd158: ramp=17'd58079;
                8'd159: ramp=17'd58403;
                8'd160: ramp=17'd58720;
                8'd161: ramp=17'd59032;
                8'd162: ramp=17'd59337;
                8'd163: ramp=17'd59637;
                8'd164: ramp=17'd59930;
                8'd165: ramp=17'd60217;
                8'd166: ramp=17'd60498;
                8'd167: ramp=17'd60772;
                8'd168: ramp=17'd61040;
                8'd169: ramp=17'd61301;
                8'd170: ramp=17'd61555;
                8'd171: ramp=17'd61802;
                8'd172: ramp=17'd62042;
                8'd173: ramp=17'd62275;
                8'd174: ramp=17'd62501;
                8'd175: ramp=17'd62720;
                8'd176: ramp=17'd62931;
                8'd177: ramp=17'd63135;
                8'd178: ramp=17'd63332;
                8'd179: ramp=17'd63520;
                8'd180: ramp=17'd63701;
                8'd181: ramp=17'd63874;
                8'd182: ramp=17'd64039;
                8'd183: ramp=17'd64196;
                8'd184: ramp=17'd64345;
                8'd185: ramp=17'd64485;
                8'd186: ramp=17'd64618;
                8'd187: ramp=17'd64741;
                8'd188: ramp=17'd64857;
                8'd189: ramp=17'd64963;
                8'd190: ramp=17'd65061;
                8'd191: ramp=17'd65150;
                8'd192: ramp=17'd65230;
                8'd193: ramp=17'd65301;
                8'd194: ramp=17'd65363;
                8'd195: ramp=17'd65415;
                8'd196: ramp=17'd65458;
                8'd197: ramp=17'd65492;
                8'd198: ramp=17'd65516;
                8'd199: ramp=17'd65531;
                8'd200: ramp=17'd65536;
                default: ramp=17'd65536;
            endcase
        end
    endfunction
    // Timer changes at 10ms; intermediate arithmetic settles over system clocks.
    (* syn_preserve=1, syn_keep=1 *) reg [7:0] ramp_index;
    (* syn_preserve=1, syn_keep=1 *) reg [16:0] fraction;
    (* syn_preserve=1, syn_keep=1 *) reg signed [63:0] speed_product;
    (* syn_preserve=1, syn_keep=1 *) reg signed [31:0] desired_speed;
    (* syn_preserve=1, syn_keep=1 *) reg reverse_speed;
    (* syn_preserve=1, syn_keep=1 *) reg signed [31:0] next_speed;
    (* syn_preserve=1, syn_keep=1 *) reg signed [48:0] next_position;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ramp_index<=0;fraction<=0;speed_product<=0;
            desired_speed<=0;reverse_speed<=0;next_speed<=0;next_position<=0;
        end else begin
            if(!moving) ramp_index<=0;
            else if(elapsed<RAMP_TICKS) ramp_index<=RAMP_TICKS==200 ? elapsed[7:0] : (elapsed*200)/RAMP_TICKS;
            else if(elapsed<RAMP_TICKS+cruise_ticks) ramp_index<=200;
            else if(elapsed<end_tick) ramp_index<=RAMP_TICKS==200 ? end_tick-elapsed : ((end_tick-elapsed)*200)/RAMP_TICKS;
            else ramp_index<=0;
            fraction<=ramp(ramp_index[7:0]);
            speed_product<=$signed({1'b0,fraction})*$signed(PEAK_Q16);
            reverse_speed<=stage==2;
            desired_speed<=reverse_speed ? -(speed_product>>>16) : speed_product>>>16;
            next_speed<=enable && !stopping ? desired_speed :
                (velocity_q16>STOP_STEP ? velocity_q16-STOP_STEP :
                 velocity_q16< -STOP_STEP ? velocity_q16+STOP_STEP : 32'sd0);
            next_position<=$signed({position_q16[47],position_q16})+
                $signed({{17{next_speed[31]}},next_speed});
        end
    end
    assign target=position_q16[47:16];
    // Like UART steps, expose the fixed segment endpoint separately from the
    // smooth internal PID reference. An interrupted trajectory holds its stop.
    assign goal=(!enable || stopping) ? target : endpoint;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin active<=0;armed<=1;stopping<=0;stage<=0;endpoint<=0;elapsed<=0;position_q16<=0;velocity_q16<=0;end
        else if(!ready) begin
            active<=0;armed<=1;stopping<=0;stage<=0;elapsed<=0;velocity_q16<=0;position_q16<={actual,16'b0};
        end else begin
            if(!enable) begin armed<=1;if(active) stopping<=1;end
            if(!active) begin
                if(enable && armed) begin
                    active<=1;armed<=0;stopping<=0;stage<=0;elapsed<=0;
                    endpoint<=actual+COUNTS_180;
                    position_q16<={actual,16'b0};velocity_q16<=0;
                end
            end else if(tick) begin
                if(next_position>49'sh07fffffff0000) begin
                    position_q16<=48'sh7fffffff0000;velocity_q16<=0;active<=0;armed<=0;
                end else if(next_position< -49'sh0800000000000) begin
                    position_q16<=48'sh800000000000;velocity_q16<=0;active<=0;armed<=0;
                end else begin
                    position_q16<=next_position[47:0];velocity_q16<=next_speed;
                    if(!enable || stopping) begin
                        if(next_speed==0) active<=0;
                    end else if(moving) begin
                        if(elapsed>=end_tick) begin
                            // Remove accumulated fractional error at the exact waypoint.
                            position_q16<={endpoint,16'b0};velocity_q16<=0;
                            stage<=stage+1'b1;elapsed<=0;
                        end else elapsed<=elapsed+1;
                    end else if(stage==1 || stage==3) begin
                        position_q16<={endpoint,16'b0};velocity_q16<=0;
                        if(elapsed>=HOLD_TICKS-1) begin
                            elapsed<=0;stage<=stage+1'b1;
                            endpoint<=stage==1 ? endpoint-COUNTS_270 : endpoint+COUNTS_180;
                        end else elapsed<=elapsed+1;
                    end else begin
                        // Leave one full tick for the outer loop to capture the endpoint.
                        velocity_q16<=0;active<=0;
                    end
                end
            end
        end
    end
endmodule
