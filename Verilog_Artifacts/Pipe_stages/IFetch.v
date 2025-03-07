`timescale 1ns / 1ps

`include "C:/Users/sbala/Course_assignments/Course_assignments.srcs/sources_1/new/Shift_Reg.v"

module IFetch#(parameter XLEN=32)(
    input clk,rst,
//    input rs1,                                          // rs1 value needed for calculating jump target addressws
    input is_jump,
    input [XLEN-1:0] jump_target,
    input is_branch,direction,misprediction_,
    input [XLEN-1:0] pred_target,BTB_target,
    input[2:0] t_state,
    input stall_,stall_syn_,                            // Stall Enable (Active Low)
    output[XLEN-1:0] IAddr,I_BUF,
    output den_,rd_,wr_,dtr_syn,dtr_
    );
    
    // PC - Program Counter, I_BUF - Instruction Buffer
    reg[XLEN-1:0] PC;
        
    reg[XLEN-1:0] PC_w;
    wire[XLEN-1:0] PC_next;    
        
    BIU_Non_Multiplexed Signals(
        .ADDR(IAddr),
        .t_state(t_state),
        .den_(stall_),
        .rd_(rd_),
        .wr_(wr_),
        .dtr_syn(dtr_syn),
        .dtr_(1'b1),
        .busint(stall_)
    );
    
    Bus_Interface_Port I_Fetch(
        .cs_(den_),
        .rd_(rd_),
        .dir(dtr_syn),
        .data_out(0),
        .data_in(I_BUF)
    );

    always @(*) begin
        case({is_jump,is_branch,direction,misprediction_})
            4'b1001,4'b1011:
                PC_w<=jump_target;
            4'b0101:
                PC_w<=pred_target;
            4'b0001,4'b0011:
                PC_w<=PC_next;
            default:
                PC_w<=BTB_target;
        endcase
    end
        
    always @(posedge clk) begin
        if(rst)
            PC<={XLEN{1'b0}};
        else if(stall_syn_) begin
            PC<=PC_w;
        end
    end
    
    assign IAddr=PC,PC_next=PC+4;
endmodule
