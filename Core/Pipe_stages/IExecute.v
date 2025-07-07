`timescale 1ns / 1ps

`ifndef XLEN
`define XLEN 32
`endif
// `define XLEN 32

module IExecute(
    input clk,
    input rst,

    input EXEBufEn,

    // Signals related to ALU
    input [`XLEN-1:0] ALUopr1,
    input [`XLEN-1:0] ALUopr2,
    input [7:0] ALUOp,
    input sgn,

    input isJump,
    input [3:0] branch_type,

    input [`XLEN-1:0] PC_curr,
    input [`XLEN-1:0] Store_src,

    input pc_ins_type,

    output EXEHold,

    output reg [`XLEN-1:0] branch_resolution_address,
    output reg misprediction,
    output reg isJump_reg,
    
    output reg [`XLEN-1:0] EXE_Result,
);
    // Simulation Purpose
    // parameter XLEN=32;
    parameter SRA=8'd6;

    localparam SHAMT_WIDTH=$clog2(`XLEN);
    genvar g;

    wire [`XLEN:0] Adder;
    wire [`XLEN-1:0] AND_Unit;
    wire [`XLEN-1:0] OR_Unit;
    wire [`XLEN-1:0] XOR_Unit;
    wire [`XLEN-1:0] shifted_left_all[`XLEN-1:0];
    wire [`XLEN-1:0] shifted_right_all[`XLEN-1:0];
    wire [`XLEN-1:0] shift_left;
    wire extend, shift_overflow;

    wire isBranch;
    wire compare;
    wire misprediction_w;

    reg compare_LT;

    assign Adder={sgn&ALUopr1[`XLEN-1],ALUopr1}+{(sgn&ALUopr2[`XLEN-1])^subtract,ALUopr2^{`XLEN{subtract}}}+subtract;
    assign AND_Unit=ALUopr1&ALUopr2;
    assign OR_Unit=ALUopr1|ALUopr2;
    assign XOR_Unit=ALUopr1^ALUopr2;

    // Shifter Units
    // Left Shifter
    
    generate
        for(g=0;g<`XLEN;g=g+1) begin
            assign shifted_left_all[g]={ALUopr1[`XLEN-g-1:0],{g{1'b0}}};
            assign shifted_right_all[g]={{g{sgn}},ALUopr1[`XLEN-1:g]};
        end
    endgenerate

    assign subtract=ALUOp[1];
    assign extend=(ALUOp==SRA)&ALUopr2[`XLEN-1];
    assign shift_overflow=|ALUopr2[`XLEN-1:SHAMT_WIDTH];

    assign shift_left={`XLEN{~shift_overflow}}&shifted_left_all[ALUopr2[SHAMT_WIDTH-1:0]];

    always @(*) begin
        case({ALUopr1[`XLEN-1],ALUopr2[`XLEN-1]})
            2'b01:
                compare_LT=1'b1;
            2'b10:
                compare_LT=1'b0; 
            default: 
                compare_LT=Adder[`XLEN-1];
        endcase
    end

    assign isBranch=~(branch_type[3])&(|branch_type[2:0]);
    assign compare=((branch_type[0]&(|XOR_Unit))|(branch_type[1]&compare_LT))^branch_type[2];

    `ifdef BRANCH_PRED_EN
        assign misprediction_w=compare^branch_pred;
    `else
        assign misprediction_w=compare; 
    `endif 

    always @(posedge clk or negedge rst) begin
        if(rst==1'b0) begin
            misprediction<=1'b0;
            branch_resolution_address<={`XLEN{1'b0}};
            isJump_reg<=1'b0;
        end
        else begin
            misprediction<=misprediction_w;
            branch_resolution_address<=PC_curr+ALUopr2;
            isJump_reg<=isJump;
        end
    end

    // Operation Select

    always @(posedge clk or negedge rst) begin
        if(rst==1'b0)
            EXE_Result<={`XLEN{1'b0}};
        else if(EXEBufEn) begin
            (*parallel_case,full_case*)
            casez({branch_type[3],ALUOp})
                {1'b0,7'b00000?1}:
                    EXE_Result<=Adder;
                {1'b1,7'b00000?1}:
                    EXE_Result<={{`XLEN-1{1'b0}},compare_LT};
                {1'b?,7'b0000100}:
                    EXE_Result<=AND_Unit;
                {1'b?,7'b0001000}:
                    EXE_Result<=OR_Unit;
                {1'b?,7'b0010000}:
                    EXE_Result<=XOR_Unit;
                {1'b?,7'b0100000}:
                    EXE_Result<=shifted_left_all[ALUopr2[SHAMT_WIDTH-1:0]];
                {1'b?,7'b1000000}:
                    EXE_Result<=shifted_right_all[ALUopr2[SHAMT_WIDTH-1:0]];
            endcase
        end
    end
endmodule