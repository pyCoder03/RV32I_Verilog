`timescale 1ns / 1ps

module Shift_Reg #(parameter shift_width=8)(
    input D,
    input Rst,
    input clk,
    output shift_out
    );
    if(shift_width>0) begin
        reg[shift_width-1:0] shift_buf;
        always @(posedge clk or negedge Rst) begin
            if(Rst==1'b0)
                shift_buf<={shift_width{1'b0}};
            else
                shift_buf<={shift_buf,D};
        end
        assign shift_out=shift_buf[shift_width-1];       
    end
    else
        assign shift_out=D;
endmodule

module Pipeline_ShiftReg #(parameter WIDTH=8)(
    input[WIDTH-1:0] D,
    input clk,rst,
    output[WIDTH-1:0] sr_out
    );
    generate
    genvar i;
    for(i=0;i<WIDTH;i=i+1) begin
        Shift_Reg#(i) sr(.D(D[i]),
                         .clk(clk),
                         .Rst(rst),
                         .shift_out(sr_out[i]));
    end
    endgenerate
endmodule

module Reg_Queue#(parameter shift_width=8,parameter size=1)(
    input [size-1:0] shift_in,
    input Rst,Clk,
    output [size-1:0] shift_out
);
    generate
    genvar i;
        for(i=0;i<size;i=i+1) begin
            Shift_Reg sr(
                .D(shift_in[i]),
                .Rst(Rst),
                .clk(Clk),
                .shift_out(shift_out[i])
            );
        end
    endgenerate
endmodule
