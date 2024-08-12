`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/08/12 12:35:58
// Design Name: 
// Module Name: Project_Electric_Fan
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// Top module of Electric Fan
module top_module_of_electric_fan (
    input clk, reset_p,
    input [3:0] btn,
    input sw_direction_cntr,
    output [3:0] led_debug,
    output led, pwm,
    output [3:0] com,
    output [7:0] seg_7 );
    
    // Button 0 (btn_power) : ��ǳ�� �Ŀ� ���� (off, 1��, 2��, 3��)
    // Button 1 (btn_timer) : Ÿ�̸� ��� (off, 5��, 10�� 15��)
    // Button 2 (btn_led) : LED ��� ���� (off, 1��, 2��, 3��)
    // Button 3 (btn_echo) : Echo ��� ( ��ǳ�� �Ŀ��� ���� )
    // Switch (sw_direction) : ���� ����
    // Get One Cycle Pulse of buttons.
    wire btn_power_pedge, btn_timer_pedge, btn_led_pedge, btn_echo_pedge;
    button_cntr btn_power_cntr (.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_power_pedge));
    button_cntr btn_timer_cntr (.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_timer_pedge));
    button_cntr btn_led_cntr (.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_led_pedge));
    button_cntr btn_echo_cntr (.clk(clk), .reset_p(reset_p), .btn(btn[3]), .btn_pedge(btn_echo_pedge));
    
    // Declare Parameter
    parameter POWER_CONTROL = 4'b0001;
    parameter TIMER_CONTROL = 4'b0010;
    parameter LED_CONTROL = 4'b0100;
    parameter ECHO_CONTROL = 4'b1000;
    
    // Declare current state
    reg [3:0] current_state;
    
    // ������ ��ư�� ���� ��ǳ�� ��ȭ
    wire [1:0] duty;
    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) current_state = POWER_CONTROL;
        else if(btn_power_pedge)  current_state = POWER_CONTROL;
        else if(btn_timer_pedge)  current_state = TIMER_CONTROL;
        else if(btn_led_pedge) current_state = LED_CONTROL;
        else if(btn_echo_pedge) current_state = ECHO_CONTROL;
    end
    
    // Declare Instance of module
    wire [31:0] counter;
    wire [1:0] power_duty; 
    power_cntr power_cntr_0 (.clk(clk), .reset_p(reset_p), .btn_power_enable(btn_power_pedge), .duty(power_duty));
    
    // current state ���� ���� ���Ϳ� ������ duty ���� ���� 
    wire [1:0] duty;
    assign duty = (current_state == POWER_CONTROL) ? power_duty : 0;
    
    // ��ȭ�� ������ duty���� ���Ϳ� ����
    pwm_cntr #(.pwm_freq(100), .duty_step(4)) control_pwm (.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(pwm));
    
    // FND�� ������ ���� �Ŀ� ���
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value({14'b0, duty}), .com(com), .seg_7(seg_7));
    
    // LED�� ǥ��
    assign led_debug[3:0] = (duty == 2'd0)? 4'b0001 : (duty == 2'd1)? 4'b0010 :
                            (duty == 2'd2)? 4'b0100 : 4'b1000;
    
endmodule

// Timer Control Module
module timer_cntr (
    input clk, reset_p,
    input btn_timer_enable,
    output reg [31:0] counter );
    
    // Declare Parameter
    parameter SETTING_0SEC = 4'd0;
    parameter SETTING_5SEC = 4'd5;
    parameter SETTING_10SEC = 4'd10;
    parameter SETTING_15SEC = 4'd15;
    
    // Declare state, next_state value
    reg [3:0] state, next_state;
    
    // ���� ���� state�� �Ѿ�°�?
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) state = SETTING_0SEC;
        else if(btn_timer_enable) state = next_state;
    end
    
    // �� state�� ���� �� ���� state�� �����ϵ��� ����
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state = SETTING_5SEC;
            counter = SETTING_0SEC;
        end
        else begin
            case (state)
                // 0�ܰ� : ��ǳ�� ����
                SETTING_0SEC : begin
                    counter = 32'd500_;
                    next_state = SETTING_5SEC;
                end    
            
            // 1�ܰ� : ��ǳ�� ��ǳ
            FIRST_SPEED : begin
                duty = 2'd1;
                next_state = SECOND_SPPED;
            end
            
            // 2�ܰ� : ��ǳ�� ��ǳ
            SECOND_SPPED : begin
                duty = 2'd2;
                next_state = THIRD_SPEED;
            end
            
            // 3�ܰ� : ��ǳ�� ��ǳ
            THIRD_SPEED : begin
                duty = 2'd3;
                next_state = TURN_OFF;
            end
               
            // Default case 
            default : begin
                duty = duty;
                next_state = next_state;
            end  
        end
    end
    
endmodule

// Power Control Module
module power_cntr (
    input clk, reset_p,
    input btn_power_enable,
    output reg [1:0] duty );
    
    // Declare Parameter
    parameter TURN_OFF = 4'b0001;
    parameter FIRST_SPEED = 4'b0010;
    parameter SECOND_SPPED = 4'b0100;
    parameter THIRD_SPEED = 4'b1000;
    
    // Declare state, next_state value
    reg [3:0] state, next_state;
    
    // ���� ���� state�� �Ѿ�°�?
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) state = TURN_OFF;
        else if(btn_power_enable) state = next_state;
    end
    
    // �� state�� ���� �� ���� state�� �����ϵ��� ����
    always @(negedge clk or posedge reset_p) begin
        if(reset_p) begin
            next_state = FIRST_SPEED;
            duty = 2'd0;
        end
        else begin
            case (state)
            // 0�ܰ� : ��ǳ�� ����
            TURN_OFF : begin
                duty = 2'd0;
                next_state = FIRST_SPEED;
            end    
            
            // 1�ܰ� : ��ǳ�� ��ǳ
            FIRST_SPEED : begin
                duty = 2'd1;
                next_state = SECOND_SPPED;
            end
            
            // 2�ܰ� : ��ǳ�� ��ǳ
            SECOND_SPPED : begin
                duty = 2'd2;
                next_state = THIRD_SPEED;
            end
            
            // 3�ܰ� : ��ǳ�� ��ǳ
            THIRD_SPEED : begin
                duty = 2'd3;
                next_state = TURN_OFF;
            end
               
            // Default case 
            default : begin
                duty = duty;
                next_state = next_state;
            end    
            endcase
        end
    end  
endmodule

// PWM Control Module
module pwm_cntr #(
    parameter sys_clk = 100_000_000,
    parameter pwm_freq = 100,
    parameter duty_step = 4,
    
    parameter temp = sys_clk / pwm_freq / duty_step,
    parameter temp_half = temp / 2 )
    (
    input clk, reset_p,
    input [31:0] duty,
    output pwm );
    
    // pwm_freq ���ļ��� ���� PWM�� �����ϱ� ���� temp ����ȭ �۾�
    integer cnt_temp;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_temp = 0;
        else begin
            if(cnt_temp >= temp - 1) cnt_temp = 0;
            else cnt_temp = cnt_temp + 1;          
        end
    end
    
    // sys_clk / temp ���ļ��� ���� PWM��  ����
    wire temp_pwm;
    assign temp_pwm = (cnt_temp < temp_half) ? 0 : 1;
    
    // Get One Cycle Pulse of negative edge of temp_pwm.
    wire temp_pwm_nedge;
    edge_detector_p ed(.clk(clk), .reset_p(reset_p), .cp(temp_pwm), .n_edge(temp_pwm_nedge));
    
    // PWM�� Duty ratio�� duty_step �ܰ�� �����Ͽ� ��Ʈ�� �ϱ� ���� 
    // temp_pwm�� duty_step ����ȭ �۾�
    integer cnt_duty;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_duty = 0;
        else if(temp_pwm_nedge) begin
            if(cnt_duty >= duty_step-1) cnt_duty = 0;
            else cnt_duty = cnt_duty + 1;
        end
    end
    
    // Get sys_clk / temp / duty_step = pwm_freq(Hz) ���ļ��� ���� PWM ����
    assign pwm = (cnt_duty < duty) ? 1 : 0;
endmodule


