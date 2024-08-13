`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module fan_led(
    input clk,reset_p,
    input btn_led,
    output led_blue,
    output [7:4] led_debug,
    output [3:0] com,
    output [7:0] seg_7); 
    
    
    //btn_mode
    wire btn_mode;
    button_cntr change(.clk(clk), .reset_p(reset_p), .btn(btn_led) , .btn_pedge(btn_mode));
    
    //duty_code
    reg [5:0] duty;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) duty = 0;
        else if(btn_mode) begin
            if(duty>=60) duty =0;
            else duty = duty + 20;
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
   assign led_r  = (duty == 20);
   assign led_g = (duty == 40);
   assign led_b = (duty == 60); 
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
