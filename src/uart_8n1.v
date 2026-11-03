// One system clock domain; 115200 baud, 8 data bits, no parity, 1 stop bit.
module uart_8n1 #(parameter integer CLK_HZ=50000000, BAUD=115200) (
    input clk, input rst_n, input rx, output tx,
    input [7:0] tx_data, input tx_start, output reg tx_busy,
    output reg [7:0] rx_data, output reg rx_valid
);
    localparam integer DIV = (CLK_HZ + BAUD/2)/BAUD;
    reg [15:0] tx_timer, rx_timer;
    reg [9:0] tx_shift;
    reg [3:0] tx_bit, rx_bit;
    reg [1:0] rx_sync, rx_state;
    reg [7:0] rx_shift;
    assign tx = tx_busy ? tx_shift[0] : 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_timer<=0; tx_shift<=10'h3ff; tx_bit<=0; tx_busy<=0;
        end else if (!tx_busy) begin
            if (tx_start) begin
                tx_shift<={1'b1,tx_data,1'b0}; tx_busy<=1;
                tx_bit<=0; tx_timer<=DIV-1;
            end
        end else if (tx_timer==0) begin
            tx_timer<=DIV-1;
            tx_shift<={1'b1,tx_shift[9:1]};
            if (tx_bit==9) tx_busy<=0;
            else tx_bit<=tx_bit+1'b1;
        end else tx_timer<=tx_timer-1'b1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync<=3; rx_state<=0; rx_timer<=0; rx_bit<=0;
            rx_shift<=0; rx_data<=0; rx_valid<=0;
        end else begin
            rx_sync<={rx_sync[0],rx}; rx_valid<=0;
            case(rx_state)
                0: if (!rx_sync[1]) begin rx_timer<=DIV/2-1; rx_state<=1; end
                1: if(rx_timer!=0) rx_timer<=rx_timer-1'b1;
                   else if(rx_sync[1]) rx_state<=0;
                   else begin rx_timer<=DIV-1; rx_bit<=0; rx_state<=2; end
                2: if(rx_timer!=0) rx_timer<=rx_timer-1'b1;
                   else begin
                       rx_shift[rx_bit]<=rx_sync[1]; rx_timer<=DIV-1;
                       if(rx_bit==7) rx_state<=3;
                       else rx_bit<=rx_bit+1'b1;
                   end
                3: if(rx_timer!=0) rx_timer<=rx_timer-1'b1;
                   else begin
                       if(rx_sync[1]) begin rx_data<=rx_shift; rx_valid<=1; end
                       rx_state<=0;
                   end
            endcase
        end
    end
endmodule
