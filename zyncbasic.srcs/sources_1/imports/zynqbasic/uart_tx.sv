`default_nettype none

// uarto txd
// data<=datai
// uarto_reg<=data[0]
// assign      uarto = uarto_reg;
module UART_TX_module(
    input wire [7:0] datai,
    input wire validi,
    output wire waiti,
    output wire uarto,
    input wire clk,
    input wire resetn
);

    parameter PULSEW  = 434;
    parameter PULSEW2 = 217;

    parameter STT_WAIT = 9;
    parameter STT_STRT =10;
    parameter STT_SND0 = 0;
    parameter STT_SND1 = 1;
    parameter STT_SND2 = 2;
    parameter STT_SND3 = 3;
    parameter STT_SND4 = 4;
    parameter STT_SND5 = 5;
    parameter STT_SND6 = 6;
    parameter STT_SND7 = 7;
    parameter STT_STOP = 8;

    reg [7:0]   data=0;
    reg 	       waiti_reg=0;
    reg 	       uarto_reg=1;

    assign      waiti = waiti_reg;
    assign      uarto = uarto_reg;

    (* mark_debug = "true" *) reg [15:0]  counter=0;
    (* mark_debug = "true" *) reg [3:0]   state=STT_WAIT;

    always @(posedge clk)
        if (!resetn) begin
            state<=STT_WAIT;
            counter<=0;
            uarto_reg<=1;
            waiti_reg<=1;
        end else
            case (state)
                STT_WAIT:
                    if (counter>0) begin // wait stop bit
                        counter<=counter-1;
                    end else if (waiti_reg) begin // clear wait signal
                        waiti_reg<=0;
                    end else if (validi) begin // wait valid signal
                        data<=datai;
                        waiti_reg<=1;
                        uarto_reg<=0; // start bit
                        counter<=PULSEW;
                        state<=STT_SND0;
                    end
                STT_SND0:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_SND1;
                    end
                STT_SND1:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_SND2;
                    end
                STT_SND2:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_SND3;
                    end
                STT_SND3:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_SND4;
                    end
                STT_SND4:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_SND5;
                    end
                STT_SND5:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_SND6;
                    end
                STT_SND6:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_SND7;
                    end
                STT_SND7:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        data<={1'b0,data[7:1]};
                        uarto_reg<=data[0];
                        counter<=PULSEW;
                        state<=STT_STOP;
                    end
                STT_STOP:
                    if (counter>0) begin
                        counter<=counter-1;
                    end else begin
                        uarto_reg<=1; // stop bit
                        counter<=PULSEW+PULSEW2; // 1.0 -> 1.5 pulse width for safety
                        state<=STT_WAIT;
                    end
                default:
                    state<=STT_WAIT;
            endcase

endmodule : UART_TX_module

`default_nettype wire