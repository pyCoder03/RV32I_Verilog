`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01.06.2024 20:10:49
// Design Name: 
// Module Name: Gate_stages_tb
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


module Gate_stages_tb;
    reg mem_stall_,exe_stall_,jmp_flush_,misprediction_,clk,rst;
    wire stall_,flush_,IFBufEn,IDBufEn,EXEBufEn,MEMBufEn,WBEn;
    
    assign stall_=mem_stall_&exe_stall_;
    assign flush_=jmp_flush_&misprediction_;
    
    Gate_stages uut(
//        .mem_stall_(mem_stall_),
        .exe_stall_(exe_stall_),
        .stall_(stall_),
        .mispred_flush_(misprediction_),
        .flush_(flush_),
        .clk(clk),
        .rst(rst),
        .IFBufEn(IFBufEn),
        .IDBufEn(IDBufEn),
        .EXEBufEn(EXEBufEn),
        .MEMBufEn(MEMBufEn),
        .WBBufEn(WBEn)
    );
    
    initial begin
        mem_stall_=1'b1;
        exe_stall_=1'b1;
        jmp_flush_=1'b1;
        misprediction_=1'b1;
        clk=1'b1;
        rst=1'b0;
        
        #5
        rst=1'b1;
        
        #55
        mem_stall_=1'b0;
        
        #10
        mem_stall_=1'b1;
        
        #60
        jmp_flush_=1'b0;
        
        #10
        jmp_flush_=1'b1;
        
        #60
        mem_stall_=1'b0;
        
        #10
        mem_stall_=1'b1;
        
        #60
        misprediction_=1'b0;
        
        #10
        misprediction_=1'b1;
        
        #60
        $finish();
    end
    
//    initial begin
//        $monitor();
//    end
    
    always #5 clk<=~clk;
endmodule
