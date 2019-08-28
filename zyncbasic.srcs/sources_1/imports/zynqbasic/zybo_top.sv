/*
 (C) Tomonori Izumi <izumi@ieee.org>, Sep. 2016. All rigts reserved.
 for xc7z020clg484-1 on Digilent ZYBO
 					Ver.2016.09.13a
*/

/*

 memo: ZYBOT-R connection

 Camera Up-Down    ... CON3 PWM P3 = jd9
 Camera Left-Right ... CON3 PWM P4 = jd10
 Wheel Right       ... HB5 (DIR,EN,SA,SB) = (je1,je2,je3,je4)
 Wheel Left        ... HB5 (DIR,EN,SA,SB) = (je7,je8,je9,je10)
 Character Display ... UART TX = jb10

 */

//`timescale 1ns / 1ps

`default_nettype none

module ZYBO_top
 (
  input wire CLK125M,
  input wire [3:0] btn,
  input wire [3:0] sw,
  output wire [3:0] led,
  inout wire ja1,ja2,ja3,ja4,ja7,ja8,ja9,ja10,
  inout wire jb1,jb2,jb3,jb4,jb7,jb8,jb9,jb10,
  inout wire jc1,jc2,jc3,jc4,jc7,jc8,jc9,jc10,
  inout wire jd1,jd2,jd3,jd4,jd7,jd8,jd9,jd10,
  inout wire je1,je2,je3,je4,je7,je8,je9,je10,
  inout wire [14:0] DDR_addr,
  inout wire [2:0] DDR_ba,
  inout wire DDR_cas_n, DDR_ck_n, DDR_ck_p, DDR_cke, DDR_cs_n,
  inout wire [3:0] DDR_dm,
  inout wire [31:0] DDR_dq,
  inout wire [3:0] DDR_dqs_n,
  inout wire [3:0] DDR_dqs_p,
  inout wire DDR_odt, DDR_ras_n, DDR_reset_n, DDR_we_n, FIXED_IO_ddr_vrn, FIXED_IO_ddr_vrp,
  inout wire [53:0] FIXED_IO_mio,
  inout wire FIXED_IO_ps_clk, FIXED_IO_ps_porb, FIXED_IO_ps_srstb
 );

 wire   CLK125M_ibuf,clk0;
 IBUFG ibuf_clk(.I(CLK125M), .O(CLK125M_ibuf));
 BUFG  buf_clk(.I(CLK125M_ibuf),.O(clk0));

 wire clk;
 wire rstn;

 //----------------------------------------------------------------
 // Zynq Processing System

 wire 	arm_clko;
 wire 	arm_rstno;
 wire [31:0] 	arm_gpi01, arm_gpi02, arm_gpo01, arm_gpo02;

 assign 	clk=arm_clko;
 assign 	rstn=arm_rstno;
 //assign 	clk=clk0;
 //assign 	rstn=1'b1;

 Zynq_PS Zynq_PS_i
         (.DDR_addr(DDR_addr),
          .DDR_ba(DDR_ba),
          .DDR_cas_n(DDR_cas_n),
          .DDR_ck_n(DDR_ck_n),
          .DDR_ck_p(DDR_ck_p),
          .DDR_cke(DDR_cke),
          .DDR_cs_n(DDR_cs_n),
          .DDR_dm(DDR_dm),
          .DDR_dq(DDR_dq),
          .DDR_dqs_n(DDR_dqs_n),
          .DDR_dqs_p(DDR_dqs_p),
          .DDR_odt(DDR_odt),
          .DDR_ras_n(DDR_ras_n),
          .DDR_reset_n(DDR_reset_n),
          .DDR_we_n(DDR_we_n),
          .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
          .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
          .FIXED_IO_mio(FIXED_IO_mio),
          .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
          .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
          .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
          .clko(arm_clko),
          .gpi01(arm_gpi01),
          .gpi02(arm_gpi02),
          .gpo01(arm_gpo01),
          .gpo02(arm_gpo02),
          .rstno(arm_rstno));

 //----------------------------------------------------------------
 // setup : 汎用カウンタと入力チャタリング対策、出力
 // 注…クロックが PL系clk0とPS系clkの２系統

 reg [31:0] 	counter0=0;
 always @(posedge clk0)
     counter0<=counter0+1;

 reg [31:0] 	counter=0;
 always @(posedge clk)
     if (!rstn) counter<=0;
     else counter<=counter+1;

 reg [3:0] 	led_r;
 reg [3:0] 	sw_r;
 reg [3:0] 	btn_r;

 always @(posedge clk)
     if (counter[15:0]==0) begin // anti-chattering
      sw_r  <= sw;
      btn_r <= btn;
     end

 assign led=led_r;

 //----------------------------------------------------------------
 // setup : debug UART out at BOARD CLOCK

 wire [31:0] debug1,debug2,debug3,debug4;
 wire       UART0TX;

 defparam   disp.PULSEW  =10416; // 100MHz/9600bps=10416
 defparam   disp.PULSEW2 = 5208; // PULSEW/2
 //defparam   disp.PULSEW  =13021; // 125MHz/9600bps=13021
 //defparam   disp.PULSEW2 = 6510; // PULSEW/2
 HEX2OLED_module disp(debug1,debug2,debug3,debug4,UART0TX,clk,rstn);

 //----------------------------------------------------------------
 // PWM motor control

 // servo: period 20.0ms, width 1.9-2.1ms
 //PWM_module servo_v_pwm(servo_v_period,servo_v_width,servo_v_pwm,clk,rstn);
 //PWM_module servo_h_pwm(servo_h_period,servo_h_width,servo_h_pwm,clk,rstn);

 // motor: period 40us, width 0-40us
 //PWM_module motor_r_pwm(motor_r_period,motor_r_width,motor_r_pwm,clk,rstn);
 //PWM_module motor_l_pwm(motor_l_period,motor_l_width,motor_l_pwm,clk,rstn);

 //----------------------------------------------------------------
 // setup : Zynq_PS GPIO への入力

 assign  arm_gpi01={12'b0,sw_r,12'b0,btn_r};
 assign  arm_gpi02=counter;

 //----------------------------------------------------------------
 // setup : DEBUG用 OLED 表示設定

 assign  debug1=arm_gpo01;
 assign  debug2=arm_gpo02;
 assign  debug3={3'b0, sw_r[3],3'b0, sw_r[2],3'b0, sw_r[1],3'b0, sw_r[0],
     3'b0,btn_r[3],3'b0,btn_r[2],3'b0,btn_r[1],3'b0,btn_r[0]};
 assign  debug4=counter;

 //----------------------------------------------------------------
 // setup : LED 表示設定

 always @(posedge clk)
     case (btn_r)
      4'b0001: led_r<=sw_r;
      4'b0010: led_r<=counter[23:20];
      4'b0100: led_r<=counter[27:24];
      4'b1000: led_r<=counter[31:28];
      default: led_r<=arm_gpo01;
     endcase

 //----------------------------------------------------------------
 // setup : 汎用入出力ピンの接続

 assign      ja1 =1'bz;
 assign      ja2 =1'bz;
 assign      ja3 =1'bz;
 assign      ja4 =1'bz;
 assign      ja7 =1'bz;
 assign      ja8 =1'bz;
 assign      ja9 =1'bz;
 assign      ja10=1'bz;

 assign      jb1 =1'bz;
 assign      jb2 =1'bz;
 assign      jb3 =1'bz;
 assign      jb4 =1'bz;
 assign      jb7 =1'bz;
 assign      jb8 =1'bz;
 assign      jb9 =1'bz;
 assign      jb10=UART0TX;

 assign      jc1 =1'bz;
 assign      jc2 =1'bz;
 assign      jc3 =1'bz;
 assign      jc4 =1'bz;
 assign      jc7 =1'bz;
 assign      jc8 =1'bz;
 assign      jc9 =1'bz;
 assign      jc10=1'bz;

 assign      jd1 =1'bz;
 assign      jd2 =1'bz;
 assign      jd3 =1'bz;
 assign      jd4 =1'bz;
 assign      jd7 =1'bz;
 assign      jd8 =1'bz;
 assign      jd9 =1'b0;
 assign      jd10=1'b0;

 assign      je1 =1'b1;
 assign      je2 =1'b0;
 assign      je3 =1'bz;
 assign      je4 =1'bz;
 assign      je7 =1'b0;
 assign      je8 =1'b0;
 assign      je9 =1'bz;
 assign      je10=1'bz;

endmodule : ZYBO_top

`default_nettype wire

// [Synth 8-3331] unconnected port
// [Synth 8-3848] does not have driver
// [Synth 8-3295] tying undriven pin to constant
