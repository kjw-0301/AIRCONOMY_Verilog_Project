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
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// Top module of Electric Fan
module top_module_of_electric_fan (
    input clk, reset_p,
    input [3:0] btn,
    input sw_direction_cntr,
    output [7:0] led_debug,
    output led, pwm,
    output [3:0] com,
    output [7:0] seg_7,
    output rotation);
    
    // Button 0 (btn_power) : 선풍기 파워 조절 (off, 1단, 2단, 3단)
    // Button 1 (btn_timer) : 타이머 모드 (off, 5초, 10초 15초)
    // Button 2 (btn_led) : LED 밝기 조절 (off, 1단, 2단, 3단)
    // Button 3 (btn_echo) : Echo 모드 ( 선풍기 파워만 조정 )
    // Switch (sw_direction) : 방향 설정
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
    parameter ROTATION_CONTROL = 4'b0001;
    
    // Declare current state
    reg [4:0] current_state;
    
    // 눌러진 버튼에 따른 선풍기 변화    
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) current_state = POWER_CONTROL;
        else if(btn_power_pedge)  current_state = POWER_CONTROL;
        else if(btn_timer_pedge)  current_state = TIMER_CONTROL;
        else if(btn_led_pedge) current_state = LED_CONTROL;
        else if(btn_echo_pedge) current_state = ECHO_CONTROL;
        else if(sw_direction_cntr) current_state = ROTATION_CONTROL;
    end
    
    // Declare Instance of module
    wire [1:0] power_duty, echo_duty;  // duty ratio of motor
    wire [3:0] left_time;              // if current state is timer mode, this variable has data of lefted time.
   wire rotation_enable;
    power_cntr power_cntr_0 (.clk(clk), .reset_p(reset_p), .btn_power_enable(btn_power_pedge), .btn_timer_enable(btn_timer_pedge), .duty(power_duty), .left_time(left_time), .rotation_enable(rotation_enable));
    
    // current state 값에 따라 모터에 적용한 duty 값을 선택 
    wire [1:0] duty;
    assign duty = (current_state == ECHO_CONTROL) ? echo_duty : power_duty;
    dht11_duty(.clk(clk),.reset_p(reset_p), .dht11_data(dht11_data),.echo_btn_enable(btn_echo_pedge),.duty(echo_duty));
    
    // 변화된 모터의 duty값을 모터에 적용
    pwm_cntr #(.pwm_freq(100), .duty_step(4)) control_pwm (.clk(clk), .reset_p(reset_p), .duty(duty), .pwm(pwm));

    //led_mode0
    wire led_blue;
    wire [7:0]led_led_debug;
    fan_led blue_led(.clk(clk), .reset_p(reset_p), .btn_led(btn_led_pedge),
                                              .led_blue(led_blue), .led_debug(led_led_debug[7:4]));
    
    // FND로 현재의 모터 파워 출력
    wire [15:0] bcd_left_time;
    bin_to_dec convert_bin_to_dec_for_left_time (.bin(left_time), .bcd(bcd_left_time));
    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value({bcd_left_time[7:0], 6'b0, duty}), .com(com), .seg_7(seg_7));
    
    // LED로 표시
    assign led_debug[3:0] = (duty == 2'd0)? 4'b0001 : (duty == 2'd1)? 4'b0010 :
                            (duty == 2'd2)? 4'b0100 : 4'b1000;
    assign led = led_blue;
    assign led_debug[7:4] = led_led_debug[7:4];
    
    
    // rotation instance
    rotation_cntr( .clk(clk), .reset_p(reset_p), .sw_direction(sw_direction_cntr),.rotation(rotation), .rotation_enable(rotation_enable));
endmodule












///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Power Control Module
module power_cntr (
    input clk, reset_p,
    input btn_power_enable,
    input btn_timer_enable,
    output [3:0] left_time,
    output reg [1:0] duty,
    output reg rotation_enable );
    
    // Declare Parameter
    parameter TURN_OFF = 4'b0001;
    parameter FIRST_SPEED = 4'b0010;
    parameter SECOND_SPEED = 4'b0100;
    parameter THIRD_SPEED = 4'b1000;
    
    // Declare state, next_state value
    reg duty_enable;
    wire reset_time_out;
    reg [3:0] speed_state, speed_next_state;
    
    // 언제 다음 state로 넘어가는가?
    always @(posedge clk or posedge reset_p) begin
        if(reset_p | reset_time_out) speed_state = TURN_OFF;
        else if(btn_power_enable) speed_state = speed_next_state;
    end

    
    // 각 state의 동작 및 다음 state로 동작하도록 설계
    always @(negedge clk or posedge reset_p) begin
        if(reset_p | reset_time_out) begin
            speed_next_state = FIRST_SPEED;
            duty = 2'd0;
            rotation_enable =0;
        end 
        else begin
            case (speed_state)
            // 0단계 : 선풍기 끄기
            TURN_OFF : begin
                duty = 2'd0;
                speed_next_state = FIRST_SPEED;
                rotation_enable =0;
            end    
            
            // 1단계 : 선풍기 약풍
            FIRST_SPEED : begin
                duty = 2'd1;
                speed_next_state = SECOND_SPEED;
                rotation_enable =1;
            end
            
            // 2단계 : 선풍기 중풍
            SECOND_SPEED : begin
                duty = 2'd2;
                speed_next_state = THIRD_SPEED;
                rotation_enable =1;
            end
            
            // 3단계 : 선풍기 강풍
            THIRD_SPEED : begin
                duty = 2'd3;
                speed_next_state = TURN_OFF;
                rotation_enable =1;
            end
               
            // Default case 
            default : begin
                duty = duty;
                speed_next_state = speed_next_state;
                rotation_enable =0;
            end    
            endcase
        end
    end  
    
    //// Timer Setting

    // Prescaling operation to create a counter in seconds
    wire clk_1usec, clk_1msec, clk_1sec;
    clock_div_100 usec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_1usec));
    clock_div_1000 msec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_1usec), .clk_div_1000(clk_1msec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_1msec), .clk_div_1000_nedge(clk_1sec));

    // Motor Run Enable variable Setting
    reg timer_enable; 

    // Declare Parameter
    parameter SETTING_0SEC = 4'd0;
    parameter SETTING_5SEC = 4'd5;
    parameter SETTING_10SEC = 4'd10;
    parameter SETTING_15SEC = 4'd15;
    
    // Declare state, next_state value
    reg [3:0] timer_state, timer_next_state;
    
    // 언제 다음 state로 넘어가는가?
    always @(posedge clk or posedge reset_p) begin
        if(reset_p | reset_time_out) timer_state = SETTING_0SEC;  
        else if(btn_timer_enable) timer_state = timer_next_state;
    end
    
    // 각 state의 동작 및 다음 state로 동작하도록 설계
    always @(negedge clk or posedge reset_p) begin
        if(reset_p | reset_time_out) begin
            timer_next_state = SETTING_5SEC;
            timer_enable = 0;
        end
        else begin
            case (timer_state)
                // 0단계 : Turn off electric fan
                SETTING_0SEC : begin
                    timer_next_state = SETTING_5SEC;
                    timer_enable = 0;
                end    
            
                // 1단계 : Setting Timer 5sec
                SETTING_5SEC : begin
                    timer_next_state = SETTING_10SEC;
                    timer_enable = 1;
                end
            
                // 2단계 : Setting Timer 10sec
                SETTING_10SEC : begin
                    timer_next_state = SETTING_15SEC;
                    timer_enable = 1;
                end
            
                // 3단계 : Setting Timer 15sec
                SETTING_15SEC : begin
                    timer_next_state = SETTING_0SEC;
                    timer_enable = 1;
                end
               
                // Default case 
                default : begin
                    timer_next_state = timer_next_state;
                    timer_enable = timer_enable;
                end  
            endcase
         end
    end
    
    
    // Down Counting of Timer
    reg [3:0] timer;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p | reset_time_out) begin timer = 0; duty_enable = 1; end
        else if(btn_timer_enable) timer = timer_next_state;
        else if(clk_1sec && timer_enable && timer >= 0) begin
            if(timer <= 0 && timer_enable) duty_enable = 0; 
            else if(duty_enable) timer = timer - 1;
        end
    end
    
    // Print lefted time to FND.
    assign left_time = timer;
    
    // When the duty_enable variable is 0 and the btn_power_enable variable is activated, the reset for Time out is enabled.
    assign reset_time_out = timer_enable && (timer == 0) &&  duty_enable;
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


    // pwm_freq 주파수를 갖는 PWM을 생성하기 위한 temp 분주화 작업
    integer cnt_temp;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_temp = 0;
        else begin
            if(cnt_temp >= temp - 1) cnt_temp = 0;
            else cnt_temp = cnt_temp + 1;          
        end
    end

    // sys_clk / temp 주파수를 갖는 PWM를  생성
    wire temp_pwm;
    assign temp_pwm = (cnt_temp < temp_half) ? 0 : 1;

    // Get One Cycle Pulse of negative edge of temp_pwm.
    wire temp_pwm_nedge;
    edge_detector_p ed(.clk(clk), .reset_p(reset_p), .cp(temp_pwm), .n_edge(temp_pwm_nedge));

    // PWM의 Duty ratio를 duty_step 단계로 구분하여 컨트롤 하기 위해 
    // temp_pwm을 duty_step 분주화 작업
    integer cnt_duty;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) cnt_duty = 0;
        else if(temp_pwm_nedge) begin
            if(cnt_duty >= duty_step-1) cnt_duty = 0;
            else cnt_duty = cnt_duty + 1;
        end
    end

    // Get sys_clk / temp / duty_step = pwm_freq(Hz) 주파수를 갖는 PWM 생성
    assign pwm = (cnt_duty < duty) ? 1 : 0;
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module rotation_cntr(
        input clk, reset_p,
        input sw_direction,    // rotation switch
        input rotation_enable,
        output rotation              //  surbo moter output
        );
        
        integer clk_div;
        always @(posedge clk)  clk_div =clk_div +1;   //clock divider
        
        wire clk_div_24_nedge;
        edge_detector_n ed( .clk(clk), .reset_p(reset_p), . cp(clk_div[24]), .n_edge(clk_div_24_nedge));  
        
        reg [6:0] duty, duty_max, duty_min;
        reg down_up;
        always @(posedge clk or posedge reset_p) begin
                    if(reset_p) begin
                            duty =45;
                            duty_max = 75;     //  12.5% of  pwm
                            duty_min = 15;       // 2.5% of pwm
                    end
                    
                    else if(sw_direction&&rotation_enable) begin   // rotation switch 'on' -> start
                             if (clk_div_24_nedge) begin
                                    if(down_up)begin    // duty goes down until reach duty_min
                                            if(duty>= duty_min) duty = duty-1;
                                            else down_up =0;
                                    end
                                    else if(!down_up) begin // duty goes up until reach duty_max
                                            if(duty<=duty_max)duty = duty+1;
                                            else down_up =1;
                                    end                              
                            end
                      end
         end
         
         pwm_Nstep_freq     #(.duty_step(600), .pwm_freq(50))  
                 pwm_rotation       (.clk(clk), .reset_p(reset_p),  .duty(duty), .pwm(rotation));   //pwm pulse = rotation
          
endmodule

//===================================================================
module fan_led(
    input clk,reset_p,
    input btn_led,
    output led_blue,
    output [7:4] led_debug,
    output [3:0] com,
    output [7:0] seg_7); 
    
    /*
    //btn_mode
    wire btn_mode;
    button_cntr change(.clk(clk), .reset_p(reset_p), .btn(btn_led) , .btn_pedge(btn_mode));
    */
    
    
    //duty_code
    reg [5:0] duty;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) duty = 0;
        else if(btn_led) begin
            if(duty>=90) duty =0;
            else duty = duty + 30;
        end
     end
     
   //PWM_inst
   pwm_100step_solo led_b(.clk(clk), .reset_p(reset_p) , .duty(duty), .pwm(led_blue));   

   //led
   assign led_debug[4] = (duty == 0);
   assign led_debug[5] = (duty == 20);
   assign led_debug[6] = (duty == 40);
   assign led_debug[7] = (duty == 60);
   
   
   // use 3color
   /*
   //3color_led
   assign led_r  = (duty == 30);
   assign led_g = (duty == 60);
   assign led_b = (duty == 90); 
   */

   //convert_bcd
  wire [6:0] duty_bcd;
  bin_to_dec duty_bcds(.bin(duty),  .bcd(duty_bcd));
  
  //FND
  fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(duty_bcd), 
                                                .com(com), .seg_7(seg_7));   
                              
endmodule


//========================================================
module pwm_100step_solo(       
    input clk, reset_p,
    input [6:0] duty,   //100단계
    output pwm);
 
    parameter sys_clk_freq = 100_000_000;       //100MHZ(10ns주기)
    parameter pwm_freq = 10_000; //100US주기
    parameter duty_step = 100;
    parameter temp = (sys_clk_freq/pwm_freq )/ duty_step;
    parameter temp_half = temp/2;
    //parameter는 미리 계산되서 나가기 떄문에 
    
   reg[6:0] cnt_sysclk;      
      // integer로 선언하면 편하긴함 , 비트수 계싼안해도 괜찮아서
     wire pwm_freqx100;
     wire pwm_freqx100_nedge;
     
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) cnt_sysclk =0;
            else  begin
               if(cnt_sysclk >= temp-1) cnt_sysclk = 0;   //공식화 가능
              else cnt_sysclk = cnt_sysclk +1;
            end  
       end   
                assign pwm_freqx100 = (cnt_sysclk < temp_half) ? 1 : 0;    
                    //ex) duty가 90이라면 90% 듀티
            
     edge_detector_nedge ed_n1(.clk(clk), .reset_p(reset_p),
                                                   .cp(pwm_freqx100), .p_edge(pwm_freqx100_nedge));
        
        
                    
       reg  [6:0] count_duty;       //10-6
       
       always @(negedge clk or posedge reset_p)begin
            if(reset_p) count_duty =0;
            else if(pwm_freqx100_nedge) begin
                if(count_duty >=99) count_duty =0;
                else  count_duty = count_duty + 1;    //128이 최대이고 7비트니까 128넘으면 자동 0    
            end  
       end   
                assign pwm = (count_duty < duty) ? 1 : 0;    
endmodule

//===========================================================================
module edge_detector_nedge(
    input clk, reset_p,
    input cp, // 버튼
    output p_edge, n_edge);
    
    reg ff_cur, ff_old;
    always @(negedge clk or posedge reset_p) begin
        if (reset_p) begin
            ff_cur <= 0;
            ff_old <= 0;
        end
        else begin
            ff_old <= ff_cur; // 0
            ff_cur <= cp; // 1
            
        end
    end
    
    assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1 : 0;
    assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1 : 0;
    
endmodule

//=================================================
module button_cntr(
        input clk, reset_p,
        input btn,
        output btn_pedge, btn_nedge);
        
         reg[20:0] clk_div = 0; 
         always @(posedge clk)clk_div = clk_div +1;
       
       
         //채터링 과정 스타트
         wire clk_div_nedge;  
    edge_detector_positive ed(.clk(clk), .reset_p(reset_p),
                                               .cp(clk_div[16]), .n_edge(clk_div_nedge)); 
                      
                      
         //채터링 잡는 과정                         
         reg debounced_btn;
         always @(posedge clk or posedge reset_p)begin
            if(reset_p) debounced_btn =0;
            else if(clk_div_nedge) debounced_btn = btn;
         end
         
         

    edge_detector_positive btn1(.clk(clk), .reset_p(reset_p),
                               .cp(debounced_btn), .n_edge(btn_nedge), .p_edge(btn_pedge)); 
                                            //debounced_btn이아니라 btn이 들어가면 눌렀을때 동작하나 채터링발생
                                             //손으로 버튼ㅇ르 눌렀을때 하강엣지, 누르던 손을  똇을댸 상승엣지 
endmodule

//===================================================
module edge_detector_positive(
    input clk, reset_p,
    input cp, // 버튼
    output p_edge, n_edge
);
    
    reg ff_cur, ff_old;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin // reset이 1일 때 cur, old에 각각 0값을 대입
            ff_cur <= 0;
            ff_old <= 0;
        end
        else begin
            ff_old <= ff_cur; // 0
            ff_cur <= cp; // 1 -----> 버튼을 누르면 cur는 1로 변환
            

        end
    end
    
    assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1 : 0; 
    assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1 : 0;
    
endmodule

//=================================================
// 16진수를 bcd코드로 바꿔주는 코드(2진화 10진수) 
module bin_to_dec(
        input [11:0] bin,
        output reg [15:0] bcd
    );

    reg [3:0] i;

    always @(bin) begin
        bcd = 0;
        for (i=0;i<12;i=i+1)begin
            bcd = {bcd[14:0], bin[11-i]};
            if(i < 11 && bcd[3:0] > 4) bcd[3:0] = bcd[3:0] + 3;
            if(i < 11 && bcd[7:4] > 4) bcd[7:4] = bcd[7:4] + 3;
            if(i < 11 && bcd[11:8] > 4) bcd[11:8] = bcd[11:8] + 3;
            if(i < 11 && bcd[15:12] > 4) bcd[15:12] = bcd[15:12] + 3;
        end
    end
endmodule

//=====================================================
module fnd_cntr(     //컨트롤러  //fnd 0일때 켜진다.  
        input clk, reset_p,
        input [15:0] value,
        output [3:0] com,       //공통단자  //LED의 전원 담당
        output [7:0] seg_7);
        
        ring_counter_fnd rc(clk, reset_p, com);
         // 모듈명         인스턴스명
        reg [3:0] hex_value;     //decoder_7seg에 들어갈 reg
        always @(posedge clk) begin
                case(com)  
                    4'b1110 : hex_value = value[3:0];   
                    4'b1101 : hex_value = value[7:4];   
                    4'b1011 : hex_value = value[11:8];   
                    4'b0111 : hex_value = value[15:12];   
            endcase
        end
        
        decoder_7seg sub7seg(.hex_value(hex_value), .seg_7(seg_7));
        //sub7seg 이름 설정
endmodule

//dht11 module
module dht11_duty(
    input clk, reset_p, 
    inout dht11_data,
    input echo_btn_enable,
    output reg[1:0]duty);
    
    wire [7:0] humidity, temperature; 
    dht11_cntrl dth11(.clk(clk), .reset_p(reset_p), .dht11_data(dht11_data), .humidity(humidity), .temperature(temperature));
    
    wire [15:0] humidity_bcd, temperature_bcd;
    bin_to_dec bcd_humi(.bin({4'b0, humidity}),  .bcd(humidity_bcd));
    bin_to_dec bcd_temp(.bin({4'b0, temperature}),  .bcd(temperature_bcd));
    
    wire[15:0] t_data = temperature_bcd;
    wire[15:0] h_data = humidity_bcd;
    reg echo_enable;
    always@(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            duty <= 2'd0;
            echo_enable = 0;
        end
        else if(echo_btn_enable) begin 
            if(echo_enable)begin
                if(t_data[7:0] >= 15'd32 && h_data[7:0] <= 15'd15) duty = 2'd0;
                else if(t_data[7:0] >= 15'd31 && h_data[7:0] >= 15'd25) duty = 2'd3;
                else if(t_data[7:0] >= 15'd28 && h_data[7:0] >= 15'd25) duty = 2'd2;
                else if(t_data[7:0] >= 15'd25 && h_data[7:0] >= 15'd25) duty = 2'd1;
            end
        end
    end
    
endmodule





