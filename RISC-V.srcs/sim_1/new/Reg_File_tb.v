`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.05.2024 22:51:27
// Design Name: 
// Module Name: Reg_File_tb
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


module Reg_File_tb;
    reg[4:0] rd_enc,rs1_enc,rs2_enc;
    reg[31:0] reg_w;
    reg rst,clk,w_en;
    wire[31:0] rs1,rs2;
    
    Reg_File registers(
        .reg_w(reg_w),
        .write_enable(w_en),
        .clk(clk),
        .rst(rst),
        .rs1_enc(rs1_enc),
        .rs2_enc(rs2_enc),
        .reg_enc_write(rd_enc),
        .rs1(rs1),
        .rs2(rs2)
    );
    
    task Reg_write;
        begin
            repeat(5) begin
                #10
                rd_enc=$urandom;
                reg_w=$urandom;
            end
            w_en=~w_en;
        end
    endtask
    
    initial begin
        rst=1'b0;
        clk=1'b1;
        w_en=1'b0;
        rd_enc=5'b0;
        rs1_enc=5'b0;
        rs2_enc=5'b0;
        reg_w=31'b0;
        #10
        rst=1'b1;
        repeat(6)
            Reg_write();
        $finish();
    end
    
    initial begin
        repeat(61) begin
            #10
            rs1_enc=$urandom;
            rs2_enc=$urandom;
        end
    end
    
    always #5 clk=~clk;
endmodule
