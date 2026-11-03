// Adapted from the user's key_read example: active-low, press/release FSM.
// MCNT=999999 at 50 MHz gives 20 ms of stable-level filtering.
module key_read #(parameter integer MCNT=1000000-1) (
    input clk, rst_n, key,
    output reg key_p_flag, key_r_flag
);
    reg sync_d0_key, sync_d1_key, r_key;
    reg [2:0] state;
    reg [31:0] cnt;
    localparam IDLE=0, P_FILTER=1, WAIT_R=2, R_FILTER=3;
    wire pedge_key=sync_d1_key && !r_key;
    wire nedge_key=!sync_d1_key && r_key;
    wire time_20ms_reach=cnt>=MCNT;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin sync_d0_key<=1; sync_d1_key<=1; r_key<=1; end
        else begin sync_d0_key<=key; sync_d1_key<=sync_d0_key; r_key<=sync_d1_key; end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin state<=IDLE; cnt<=0; key_p_flag<=0; key_r_flag<=0; end
        else begin
            key_p_flag<=0; key_r_flag<=0;
            case(state)
                IDLE: begin cnt<=0; if(nedge_key) state<=P_FILTER; end
                P_FILTER: begin
                    // A bounce wins even on the terminal count.
                    if(pedge_key || sync_d1_key) begin state<=IDLE; cnt<=0; end
                    else if(time_20ms_reach) begin state<=WAIT_R; cnt<=0; key_p_flag<=1; end
                    else cnt<=cnt+1'b1;
                end
                WAIT_R: begin cnt<=0; if(pedge_key) state<=R_FILTER; end
                R_FILTER: begin
                    if(nedge_key || !sync_d1_key) begin state<=WAIT_R; cnt<=0; end
                    else if(time_20ms_reach) begin state<=IDLE; cnt<=0; key_r_flag<=1; end
                    else cnt<=cnt+1'b1;
                end
                default: begin state<=IDLE; cnt<=0; end
            endcase
        end
    end
endmodule
