`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.02.2024 15:32:52
// Design Name: 
// Module Name: Clock_gen
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
`include "C:/Users/sbala/Course_assignments/Course_assignments.srcs/sources_1/new/Pipeline_MUL_4x4.v"
`include "C:/Users/sbala/Course_assignments/Course_assignments.srcs/sources_1/new/Pipelined_Proc.v"
 
module Clock_gen(
    input Clk,
    input Rst,
    output[2:0] t_state,
    output clkdiv4
    );
    
    wire t2,t1,t0;
    
    Dual_Edge_Count #(3)(.cnt(1'b1),
                         .clk(Clk),
                         .rst(Rst),
                         .Q(t_state));
    
    assign {t2,t1,t0}=t_state,clkdiv4=t2|t1|t0;
            
endmodule
