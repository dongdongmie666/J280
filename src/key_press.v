module key_press #(parameter integer DEBOUNCE_CYCLES=1000000) (
    input clk, input rst_n, input key_n, output reg press
);
    reg [1:0] sync;
    reg stable;
    reg [19:0] timer;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin sync<=3; stable<=1; timer<=0; press<=0; end
        else begin
            sync<={sync[0],key_n}; press<=0;
            if(sync[1]==stable) timer<=0;
            else if(timer==DEBOUNCE_CYCLES-1) begin
                stable<=sync[1]; timer<=0; press<=!sync[1];
            end else timer<=timer+1'b1;
        end
    end
endmodule
