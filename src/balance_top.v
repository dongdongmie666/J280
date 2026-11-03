module balance_top #(
    parameter integer CLK_HZ=50000000,
    parameter integer REPORT_CYCLES=CLK_HZ*3/1000,
    parameter integer KEY_CYCLES=1000000,
    parameter integer ENCODER_FAULT_PROTECTION=0,
    parameter integer COUNTS_PER_REV=1040
) (
    input clk_50m, rst_n, input [3:0] key_sw,
    input uart_rx, output uart_tx,
    output adc_clk, adc_oe_n, input [9:0] adc_data_in, input adc_otr,
    input enc1_a, enc1_b, enc2_a, enc2_b,
    output AN1, AN2, PWMA, BN1, BN2, PWMB,
    input dip1_n, dip2_n
);
    // ADC hardware below is fixed at the official 50 MHz / 3 ms cadence.
    localparam integer PID_PERIOD_MS=3;
    reg [1:0] reset_sync;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) reset_sync<=0; else reset_sync<={reset_sync[0],1'b1};
    wire reset_n=reset_sync[1];
    wire dip_on_pulse,dip_off_pulse;
    reg sine_request;
    wire dip2_on_pulse,dip2_off_pulse;
    reg curve2_request;
    key_read #(.MCNT(KEY_CYCLES-1)) dip2_filter
        (clk_50m,reset_n,dip2_n,dip2_on_pulse,dip2_off_pulse);
    always @(posedge clk_50m or negedge reset_n)
        if(!reset_n) curve2_request<=0;
        else if(dip2_off_pulse) curve2_request<=0;
        else if(dip2_on_pulse) curve2_request<=1;
    key_read #(.MCNT(KEY_CYCLES-1)) dip_filter
        (clk_50m,reset_n,dip1_n,dip_on_pulse,dip_off_pulse);
    always @(posedge clk_50m or negedge reset_n)
        if(!reset_n) sine_request<=0;
        else if(dip_off_pulse) sine_request<=0;
        else if(dip_on_pulse) sine_request<=1;
    wire [3:0] keys,keys_released;
    genvar k;
    generate for(k=0;k<4;k=k+1) begin: buttons
        key_read #(.MCNT(KEY_CYCLES-1)) key_inst
            (clk_50m,reset_n,key_sw[k],keys[k],keys_released[k]);
    end endgenerate
    wire [7:0] rx_data,tx_data;
    wire rx_valid,tx_busy,tx_start;
    uart_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(921600)) uart_inst
        (clk_50m,reset_n,uart_rx,uart_tx,tx_data,tx_start,tx_busy,rx_data,rx_valid);
    wire running,polarity_set,polarity_reverse,boost_enabled;
    wire [5:0] run_state;
    wire [3:0] fault;
    wire [7:0] limit_percent;
    wire [15:0] kp_q8,kd_q8,ki_q8,position_kp_q12,position_kd_q8,position_ki_q12;
    wire signed [31:0] error_code,delta_code,position,enc1,enc2;
    wire signed [31:0] close_code,move_counts;
    wire [2:0] effect;
    wire signed [15:0] output_q8,applied_q8;
    wire signed [15:0] target_angle_code,target_offset_q8;
    wire signed [19:0] diag_rate_q8;
    wire signed [31:0] diag_p_q8,diag_d_q8;
    wire signed [31:0] diag_angle_pwm_q8,diag_i_q8,diag_position_pwm_q8;
    wire position_loop_enabled;
    wire position_ready;
    wire [3:0] stop_reason;
    wire [9:0] stop_adc;
    wire [31:0] stop_age;
    wire [7:0] stop_flags;
    wire signed [31:0] stop_encoder;
    wire signed [31:0] position_target_counts,position_target_encoder;
    wire signed [31:0] position_p_diag,position_d_raw_diag,position_d_limit_diag,position_i_diag;
    wire signed [15:0] diag_target_request_q8;
    wire diag_target_slew;
    wire [31:0] err1,err2;
    wire [1:0] ab1,ab2;
    wire clear=!running && (keys[3] || (rx_valid && rx_data=="z"));
    encoder_quad e1(clk_50m,reset_n,clear,enc1_a,enc1_b,enc1,err1,ab1);
    encoder_quad e2(clk_50m,reset_n,clear,enc2_a,enc2_b,enc2,err2,ab2);
    wire signed [31:0] rpm_milli;
    wire signed [31:0] target_velocity_q16,target_speed_mdeg_s;
    wire target_speed_valid;
    reg signed [34:0] actual_speed_scaled;
    reg signed [31:0] actual_speed_mdeg_s;
    // RPM * 1000 times 6 equals degrees/second * 1000.
    always @(posedge clk_50m or negedge reset_n)
        if(!reset_n) begin actual_speed_scaled<=0;actual_speed_mdeg_s<=0;end
        else begin
            actual_speed_scaled<=($signed({{3{rpm_milli[31]}},rpm_milli})<<<2)+
                                 ($signed({{3{rpm_milli[31]}},rpm_milli})<<<1);
            actual_speed_mdeg_s<=actual_speed_scaled>35'sd2147483647 ? 32'sh7fffffff :
                actual_speed_scaled< -35'sd2147483647 ? -32'sh7fffffff : actual_speed_scaled[31:0];
        end
    velocity_units #(.COUNTS_PER_REV(COUNTS_PER_REV),.PERIOD_MS(10)) target_speed_units(
        .clk(clk_50m),.rst_n(reset_n),.velocity_q16(target_velocity_q16),
        .mdeg_s(target_speed_mdeg_s),.valid(target_speed_valid));
    wire speed_valid;
    encoder_speed #(.CLK_HZ(CLK_HZ),.COUNTS_PER_REV(COUNTS_PER_REV)) speed(clk_50m,reset_n,clear,enc1,err1,rpm_milli,speed_valid);
    wire [9:0] raw,mean;
    wire valid,otr;
    adc_measure adc(clk_50m,reset_n,adc_data_in,adc_otr,adc_clk,adc_oe_n,
                    raw,mean,valid,otr);
    // SW1 (T18): idle starts swing-up; balancing steps the position by 90 deg.
    // SW2/SW3 remain immediate hardware stops. Never lose a stop to UART.
    wire stop_key_n=key_sw[1] && key_sw[2];
    wire cmd_valid=rx_valid || keys[0];
    wire [7:0] cmd=rx_valid ? rx_data : running ? "n" : "w";
    // Persistent observations distinguish missing inputs from rejected starts.
    // Counts wrap at 65536 and reset only with the FPGA reset.
    reg [3:0] key_levels_meta,key_levels;
    reg [15:0] key1_count,rx_count,swing_request_count,otr_count;
    always @(posedge clk_50m or negedge reset_n) begin
        if(!reset_n) begin
            key_levels_meta<=15;key_levels<=15;
            key1_count<=0;rx_count<=0;swing_request_count<=0;otr_count<=0;
        end else begin
            key_levels_meta<=key_sw;key_levels<=key_levels_meta;
            if(keys[0]) key1_count<=key1_count+1'b1;
            if(rx_valid) rx_count<=rx_count+1'b1;
            if(cmd_valid && cmd=="w") swing_request_count<=swing_request_count+1'b1;
            if(valid && otr) otr_count<=otr_count+1'b1;
        end
    end
    balance_control #(.CLK_HZ(CLK_HZ),.ZERO(535),.PID_PERIOD_MS(PID_PERIOD_MS),
                      .COUNTS_PER_REV(COUNTS_PER_REV),
                      .ENCODER_FAULT_PROTECTION(ENCODER_FAULT_PROTECTION)) control(
        .clk(clk_50m),.rst_n(reset_n),.stop_key_n(stop_key_n),
        .sine_enable(sine_request),
        .curve2_enable(curve2_request),
        .stop_reason(stop_reason),.stop_adc(stop_adc),.stop_age(stop_age),
        .stop_flags(stop_flags),.stop_encoder(stop_encoder),
        .target_velocity_q16(target_velocity_q16),
        .sample_valid(valid),.adc_mean(mean),.sample_otr(otr),
        .encoder(enc1),.encoder_errors(err1),.cmd_valid(cmd_valid),.cmd(cmd),
        .running(running),.fault(fault),.polarity_set(polarity_set),
        .polarity_reverse(polarity_reverse),.limit_percent(limit_percent),
        .kp_q8(kp_q8),.kd_q8(kd_q8),.error_code(error_code),
        .delta_code(delta_code),.position(position),.output_q8(output_q8),
        .close_code(close_code),.move_counts(move_counts),.effect(effect),
        .boost_enabled(boost_enabled),.ki_q8(ki_q8),
        .position_loop_enabled(position_loop_enabled),
        .position_kp_q12(position_kp_q12),.position_kd_q8(position_kd_q8),
        .position_ki_q12(position_ki_q12),
        .target_angle_code(target_angle_code),.target_offset_q8(target_offset_q8),
        .diag_rate_q8(diag_rate_q8),.diag_p_q8(diag_p_q8),.diag_d_q8(diag_d_q8),.run_state(run_state),
        .diag_angle_pwm_q8(diag_angle_pwm_q8),.diag_i_q8(diag_i_q8),
        .diag_position_pwm_q8(diag_position_pwm_q8),
        .diag_target_request_q8(diag_target_request_q8),.diag_target_slew(diag_target_slew),
        .position_target_counts(position_target_counts),.position_ready(position_ready),
        .position_target_encoder(position_target_encoder),
        .diag_position_p_q8(position_p_diag),.diag_position_d_raw_q8(position_d_raw_diag),
        .diag_position_d_limit_q8(position_d_limit_diag),.diag_position_i_q8(position_i_diag));
    // J280 XH1: B driver with encoder 1. A driver is held off.
    assign AN1=0; assign AN2=0; assign PWMA=0;
    // User-verified direction correction: positive OUT drives BN2, negative BN1.
    motor_pwm #(.CLK_HZ(CLK_HZ)) motor(clk_50m,reset_n,running,stop_key_n,
                                      output_q8,BN2,BN1,PWMB,applied_q8);
    reg [31:0] timer,seq;
    wire report_tick=timer==REPORT_CYCLES-1;
    always @(posedge clk_50m or negedge reset_n) begin
        if(!reset_n) begin timer<=0; seq<=0; end
        else if(report_tick) begin timer<=0; seq<=seq+1'b1; end
        else timer<=timer+1'b1;
    end
    wire report_busy;
    // Compare requested and applied duty in the same unit. Hard stop wins.
    wire signed [15:0] pwm_target_q8=(running && stop_key_n) ? output_q8 : 16'sd0;
    wire signed [15:0] pwm_actual_q8=(running && stop_key_n) ? applied_q8 : 16'sd0;
    balance_text #(.ZERO(535),.PID_PERIOD_MS(PID_PERIOD_MS),.COMPACT(1)) report(
        clk_50m,reset_n,report_tick,seq,raw,mean,
        error_code,position,enc1,pwm_actual_q8,limit_percent,kp_q8,kd_q8,
        running,fault,polarity_set,polarity_reverse,err1,otr,
        close_code,move_counts,effect,
        tx_busy,tx_start,tx_data,report_busy,boost_enabled,rpm_milli,speed_valid,
        pwm_target_q8,ki_q8,position_loop_enabled,target_angle_code,
        target_offset_q8,position_kp_q12,position_kd_q8,
        diag_rate_q8,diag_p_q8,diag_d_q8,run_state,
        key_levels,key1_count,rx_count,swing_request_count,otr_count,
        diag_angle_pwm_q8,diag_i_q8,diag_position_pwm_q8,position_ki_q12,diag_target_request_q8,diag_target_slew,
        position_target_counts,position_ready,position_target_encoder,
        position_p_diag,position_d_raw_diag,position_d_limit_diag,position_i_diag,
        target_speed_mdeg_s,actual_speed_mdeg_s,speed_valid,target_speed_valid,
        stop_reason,stop_adc,stop_age,stop_flags,stop_encoder);
endmodule
