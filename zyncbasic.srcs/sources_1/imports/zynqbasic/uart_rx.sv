// (C) Tomonori Izumi <izumi@ieee.org>, Mar. 2014. All rigts reserved.o

// UART I/F

// speed      : 115200 bps
// stop bit   :    1 bit
// parity bit :   none
// data width :    8 bit

// 115200bps -> 434.02cycles@50MHz

`default_nettype none

//================================================================

// jb10 = UART0TX = waito
module UART_RX_module(
	input wire uarti,
	output wire[7:0] datao,
	output wire valido,
	input wire waito,
	input wire clk,
	input wire resetn
);

	parameter PULSEW  = 434;
	parameter PULSEW2 = 217;

	parameter STT_WAIT = 9;
	parameter STT_STRT =10;
	parameter STT_RCV0 = 0;
	parameter STT_RCV1 = 1;
	parameter STT_RCV2 = 2;
	parameter STT_RCV3 = 3;
	parameter STT_RCV4 = 4;
	parameter STT_RCV5 = 5;
	parameter STT_RCV6 = 6;
	parameter STT_RCV7 = 7;
	parameter STT_STOP = 8;

	reg [7:0] 	datai_reg=0;
	reg [7:0] 	datao_reg=0;
	reg 		valido_reg=0;

	assign 	datao = datao_reg;
	assign 	valido = valido_reg;

	//----------------------------------------------------------------
	// input anti-noize filter

	function majority5(input [4:0] d);
		case (d)
			5'b00000: majority5=0;
			5'b00001: majority5=0;
			5'b00010: majority5=0;
			5'b00100: majority5=0;
			5'b01000: majority5=0;
			5'b10000: majority5=0;
			5'b00011: majority5=0;
			5'b00101: majority5=0;
			5'b01001: majority5=0;
			5'b10001: majority5=0;
			5'b00110: majority5=0;
			5'b01010: majority5=0;
			5'b10010: majority5=0;
			5'b01100: majority5=0;
			5'b10100: majority5=0;
			5'b11000: majority5=0;
			default:  majority5=1;
		endcase
	endfunction


	reg [1:0] 	count=0;
	reg [4:0] 	uartiq=5'b11111;
	reg 		uarti2=1;
	always @(posedge clk)
		if (!resetn) begin
			count<=0;
			uartiq<=5'b11111;
		end else begin
			if (count==0) begin
				uartiq<={uarti,uartiq[4:1]};
			end
			uarti2<=majority5(uartiq);
			count<=count+1;
		end

	//----------------------------------------------------------------
	// receive data

	(* mark_debug = "true" *) reg [15:0] counter=0;
	(* mark_debug = "true" *) reg [3:0] 	state=STT_WAIT;

	wire 	rx_done;
	assign 	rx_done = (state==STT_STOP) && (counter==0);

	always @(posedge clk)
		if (!resetn) begin
			state<=STT_WAIT;
			counter<=0;
			datai_reg<=0;
		end else
			case (state)
				STT_WAIT:
					if (uarti2==0) begin
						counter<=PULSEW+PULSEW2; // 1.5 * pulse width
						state<=STT_RCV0;
					end
				STT_RCV0:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW;
						state<=STT_RCV1;
					end
				STT_RCV1:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW;
						state<=STT_RCV2;
					end
				STT_RCV2:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW;
						state<=STT_RCV3;
					end
				STT_RCV3:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW;
						state<=STT_RCV4;
					end
				STT_RCV4:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW;
						state<=STT_RCV5;
					end
				STT_RCV5:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW;
						state<=STT_RCV6;
					end
				STT_RCV6:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW;
						state<=STT_RCV7;
					end
				STT_RCV7:
					if (counter>0) begin
						counter<=counter-1;
					end else begin
						datai_reg<={uarti2,datai_reg[7:1]};
						counter<=PULSEW; // 1.5 -> 1.0 * pulse width shorter for safety
						state<=STT_STOP;
					end
				STT_STOP:
					if (counter>0) begin // wait next pulse-center
						counter<=counter-1;
					end else if (uarti2) begin // wait longer for safety
						state<=STT_WAIT;
					end
				default:
					state<=STT_WAIT;
			endcase

	//----------------------------------------------------------------
	// output I/F

	wire 	 oexec;
	assign 	 oexec = valido_reg & ~waito;

	always @(posedge clk)
		if (!resetn) begin
			valido_reg<=0;
			datao_reg<=0;
		end else if (rx_done && !valido_reg) begin
			valido_reg<=1;
			datao_reg<=datai_reg;
		end else if (valido_reg && !waito) begin
			valido_reg<=0;
		end

endmodule : UART_RX_module

`default_nettype wire