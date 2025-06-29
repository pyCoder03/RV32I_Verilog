`timescale 1ns / 1ps

module Fetch_Reg(                       // Module to select between operand forwarded value and register file value
    input [4:0] reg_enc,
    input [31:0] reg_in,
    input [31:0] op_fwd,
    input [31:0] outdated,
    output [31:0] reg_val
);
    reg od;
    
    always @(*) begin
        case(reg_enc)
            5'd0:	od<=outdated[0];
            5'd1:	od<=outdated[1];
            5'd2:	od<=outdated[2];
            5'd3:	od<=outdated[3];
            5'd4:	od<=outdated[4];
            5'd5:	od<=outdated[5];
            5'd6:	od<=outdated[6];
            5'd7:	od<=outdated[7];
            5'd8:	od<=outdated[8];
            5'd9:	od<=outdated[9];
            5'd10:	od<=outdated[10];
            5'd11:	od<=outdated[11];
            5'd12:	od<=outdated[12];
            5'd13:	od<=outdated[13];
            5'd14:	od<=outdated[14];
            5'd15:	od<=outdated[15];
            5'd16:	od<=outdated[16];
            5'd17:	od<=outdated[17];
            5'd18:	od<=outdated[18];
            5'd19:	od<=outdated[19];
            5'd20:	od<=outdated[20];
            5'd21:	od<=outdated[21];
            5'd22:	od<=outdated[22];
            5'd23:	od<=outdated[23];
            5'd24:	od<=outdated[24];
            5'd25:	od<=outdated[25];
            5'd26:	od<=outdated[26];
            5'd27:	od<=outdated[27];
            5'd28:	od<=outdated[28];
            5'd29:	od<=outdated[29];
            5'd30:	od<=outdated[30];
            5'd31:	od<=outdated[31];
        endcase
    end
    
    assign reg_val=(od)?op_fwd:reg_in;
endmodule

module IDecode(
    input [31:0] Ins,           // Instruction Fetch stage buffer
    input [31:0] PC_curr,       // Current PC value sent through the pipeline
    input rst,
    input clk_stage,
    input clk_stage_en,
    
    // Inputs from Register File
    input [31:0] regfile1,      // Source register rs1 fetched from register file
    input [31:0] regfile2,      // Source register rs2 fetched from register file

    input [4:0] outdated_EXE,outdated_MEM,outdated_WB,  // Destination register record from successive stages of the pipeline
                                                        // outdated_EXE is the register value taken from the EXE stage buffer and so on
    input IDBufEn,
    
    // Operand Forward values
    input [31:0] EXEBuf,MEMBuf, // EXE Unit and MEM Unit Buffer values
    
    // Tells at which stage the destination register value is actually acquired, so to generate stall according to dependencies
    // ins_class={WB_stage_signal,MEM_stage_signal,EXE_stage_signal};
    // 0 - The value is acquired in EXE stage
    // 1 - The value is acquired in MEM stage
    input [2:0] ins_class,

    // PC Redirection Instruction type
    // 00 : No jump, no branch
    // 01 : Jump
    // 10 : Non Taken Branch (Predicted to be taken)
    // 11 : Taken Branch (Predicted not to be taken)
    input [1:0] pc_ins_type,

    // Mem_rd value from MEM stage
    input Mem_rd_in,

    // Output ports that go to register file
    output [4:0] rs1_enc,
    output [4:0] rs2_enc,
    
    // Sending current PC to next stage
    output reg [31:0] PC_curr_reg,

    // Sending pc_ins_type to next stage
    output reg [1:0] pc_ins_type_reg,

    // ALU Control Signals
    output reg [7:0] ALUOp,     // ALU operation select (Decoded in this stage itself so can be directly used by ALU)
    output reg sgn,             // Indicates whether the operation is signed or unsigned
    output reg isBranch,        // Indicates whether the current instruction is a BRANCH, enabling the EXE Unit to send misprediction_ signal whenever applicable
    output reg branch_type,     // Indicates the type of BRANCH instruction (Decoded funct3 argument)
    output reg [31:0] ALUopr1,  // ALU Operands
    output reg [31:0] ALUopr2,
    
    output reg [31:0] Store_src,

    output IDHold               // Hold the pipeline (For resolving dependencies)
    );
    
    wire[6:0] opcode=Ins[6:0];
    reg[10:0] op_minterms;      // Minterms generated on decoding the opcode field
    wire[5:0] ins_type;         // Type of the instruction (R,I,S,B,U,J)
    
    wire[4:0] rd_enc;           // Destination register encoding for the current instruction cycle
    
    // Immediate Field
    
    wire[31:0] immediate;
    wire imm31,imm11,imm0;
    wire[10:0] imm30_20;
    wire[7:0] imm19_12;
    wire[5:0] imm10_5;
    wire[3:0] imm4_1;
    
    wire[5:0] cmp;               // Compare current Destination register with outdated registers
    reg[1:0] Mem_rd;
    
    wire[4:0] rs_enc[1:0];      // regfile[1:0];
    wire[31:0] regfile[1:0];
    
    assign regfile[0]=regfile1,regfile[1]=regfile2;
    assign rs1_enc=rs_enc[0],rs2_enc=rs_enc[1],regfile[0]=regfile1,regfile[1]=regfile2;
    
    // Outdated registers
    // If the Destination Register of previous Instruction cycle is required as an operand in current cycle
    // If previous instruction was a LOAD instruction then its true value will only be found after the next cycle (In MEM Unit Buffer), hence stall required
    // Else, it can be retrieved from EXE Unit Buffer without the need of stall
    // In the other case, where the Destination Register of the previous to previous Instruction cycle is required, no stall is needed
    // As the Register value is present at the MEM Unit Buffer, irrespective of that Instruction type
    reg[4:0] outdated;          // Record of Destination Register for past two Instruction cycles (Will be passed in the pipeline till Write-Back Unit)  
    
    // Stall signal for ID unit
    reg[1:0] RAW_stall;
    
    wire[2:0] funct3;
    wire[6:0] funct7;

    wire[14:0] outdated_in={outdated_WB,outdated_MEM,outdated_EXE};
    
    // Opcode type
    localparam OP=6'd0,OP_IMM=6'd1,JAL=6'd2,JALR=6'd3,BRANCH=6'd4,LOAD=6'd5,STORE=6'd6,LUI=6'd7,AUIPC=6'd8;
    
    // Instruction type
    localparam R=0,I=1,S=2,B=3,U=4,J=5;  
    
    assign funct3=Ins[14:12],funct7=Ins[31:25];
    assign rd_enc=Ins[11:7]&{5{~{op_minterms[BRANCH]|op_minterms[STORE]}}};
    assign rs_enc[0]=Ins[19:15],rs_enc[1]=Ins[24:20];
    
    // OPCODE Chart (Has to be updated)
    // OP-IMM: 0
    // LUI: 1
    // AUIPC: 2
    // OP (Integer Register-Register Operations): 3
    // JAL: 4
    // JALR: 5
    // BRANCH: 6
    // LOAD: 7
    // STORE: 8
    
    // Opcode Decoder
    
    always @(*) begin
        case(opcode)
            OP:	    op_minterms<=10'd1;
            OP_IMM:	op_minterms<=10'd2;
            JAL:	op_minterms<=10'd4;
            JALR:	op_minterms<=10'd8;
            BRANCH:	op_minterms<=10'd16;
            LOAD:	op_minterms<=10'd32;
            STORE:	op_minterms<=10'd64;
            LUI:	op_minterms<=10'd128;
            AUIPC:	op_minterms<=10'd256;
            default:    op_minterms<=10'd0;    
        endcase
    end
    
    // Instruction type classifier
    
    assign ins_type[R]=op_minterms[OP];
    assign ins_type[I]=|{op_minterms[OP_IMM],op_minterms[JALR],op_minterms[LOAD]};
    assign ins_type[S]=op_minterms[STORE];
    assign ins_type[B]=op_minterms[BRANCH];
    assign ins_type[U]=op_minterms[LUI]|op_minterms[AUIPC];
    assign ins_type[J]=op_minterms[JAL];
    
    // Pushing Destination register number into pipeline
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0) 
            outdated<=5'b0;
        else // if(stall_) begin
            outdated<=rd_enc&{5{IDBufEn}};
    end  
            
    // cmp[0] => Stores (outdated_EXE==rs1_enc)
    // cmp[1] => Stores (outdated_MEM==rs1_enc)
    // cmp[2] => Stores (outdated_WB==rs1_enc)
    // cmp[3] => Stores (outdated_EXE==rs2_enc)
    // cmp[4] => Stores (outdated_MEM==rs2_enc)
    // cmp[5] => Stores (outdated_WB==rs2_enc)

    generate
    genvar i1,j1;
        for(i1=0;i1<2;i1=i1+1) begin        
            for(j1=0;j1<3;j1=j1+1) begin
                assign cmp[3*i1+j1]=(outdated_in[5*j1+:5]==rs_enc[i1]);
            end
        end
    endgenerate    
    
    // Immediate field fetcher
    
    wire w1=ins_type[I]|ins_type[S]|ins_type[B],w2=w1|ins_type[J];
    
    assign imm31=Ins[31];
    assign imm30_20=(w2)?{11{Ins[31]}}:Ins[30:20];
    assign imm19_12=(w1)?{8{Ins[31]}}:Ins[19:12];
    assign imm11=|{(Ins[31]&(ins_type[I]|ins_type[S])),(Ins[7]&ins_type[B]),(Ins[20]&ins_type[J])};
    assign imm10_5=Ins[30:25]&{6{w2}};
    assign imm4_1=(Ins[24:21]&{4{ins_type[I]|ins_type[J]}})|(Ins[11:8]&{4{ins_type[S]|ins_type[B]}});
    assign imm0=(Ins[20]&ins_type[I])|(Ins[7]&ins_type[S]);
    
    assign immediate={imm31,imm30_20,imm19_12,imm11,imm10_5,imm4_1,imm0};
    
    // Operand update
    
    wire r_w=ins_type[R]|ins_type[B];
    
    wire[1:0] neq={(rs2_enc!=5'b0),(rs1_enc!=5'b0)};

    reg[31:0] rs2_updated;

    // Setting ALU Operand 1

    // Opcodes that have rs1: OP, OP_IMM, JALR, BRANCH, LOAD and STORE
    // Only JAL and BRANCH target calculations are done in dedicated adders
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0) begin
            ALUopr1<=32'b0;
        end
        else if(IDBufEn) begin
            casez({opcode,neq[0],cmp[1:0]})
                {OP,1'b1,2'b?1},{OP_IMM,1'b1,2'b?1},{JALR,1'b1,2'b?1},{BRANCH,1'b1,2'b?1},{LOAD,1'b1,2'b?1},{STORE,1'b1,2'b?1}:
                    ALUopr1<=EXEBuf;
                {OP,1'b1,2'b10},{OP_IMM,1'b1,2'b10},{JALR,1'b1,2'b10},{BRANCH,1'b1,2'b10},{LOAD,1'b1,2'b10},{STORE,1'b1,2'b10}:
                    ALUopr1<=MEMBuf;
                {OP,1'b1,2'b00},{OP_IMM,1'b1,2'b00},{JALR,1'b1,2'b00},{BRANCH,1'b1,2'b00},{LOAD,1'b1,2'b00},{STORE,1'b1,2'b00}:
                    ALUopr1<=regfile1;
                // LUI is carried out by placing 0 in operand 1 and immediate in operand 2
                {OP,1'b0,2'b??},{OP_IMM,1'b0,2'b??},{JALR,1'b0,2'b??},{BRANCH,1'b0,2'b??},{LOAD,1'b0,2'b??},{STORE,1'b0,2'b??},{LUI,1'b?,2'b??}:
                    ALUopr1<=32'b0;     // x0 selected
//                {AUIPC,1'b?,2'b??}:
                default:
                    ALUopr1<=PC_curr;
            endcase
        end    
    end
    
    // Setting ALU Operand 2

    // For Branch Instruction, there is immediate field, but the immediate field is sent directly to the PC-Gen stage for Next PC calculation
    // Hence in this case, ALU Operands are just rs1 and rs2
    // For Load, rs1 (base) and immediate (offset) are the operands, because the calculated address is needed in memory stage
    
    always @(*) begin                // Have to select immediate field when applicable
        casez({opcode,neq[1],cmp[3:2]})
            {OP,1'b1,2'b?1},{BRANCH,1'b1,2'b?1},{STORE,1'b1,2'b?1}:
                rs2_updated<=EXEBuf;
            {OP,1'b1,2'b10},{BRANCH,1'b1,2'b10},{STORE,1'b1,2'b10}:
                rs2_updated<=MEMBuf;
            {OP,1'b1,2'b00},{BRANCH,1'b1,2'b00},{STORE,1'b1,2'b00}:
                rs2_updated<=regfile2;
            // {OP,1'b0,2'b??},{BRANCH,1'b0,2'b??},{STORE,1'b0,2'b??}:
            default:    
                rs2_updated<=32'b0;
        endcase
    end

    always @(posedge clk_stage or negedge rst) begin                // Have to select immediate field when applicable
        if(rst==1'b0)
            ALUopr2<=32'b0;
        else if(IDBufEn)
            ALUopr2<=(|{op_minterms[OP_IMM],op_minterms[LUI],op_minterms[AUIPC],op_minterms[JAL],op_minterms[JALR],op_minterms[LOAD],op_minterms[STORE]})?immediate:rs2_updated;
    end

    // Source register for STORE operation                                   
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0)
            Store_src<=32'b0;
        else if(IDBufEn&ins_type[S])
            Store_src<=rs2_updated;
    end

    // Passing Current PC, Mem_rd and pc_ins_type in the pipeline
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==0) begin
            PC_curr_reg<=32'b0;
            Mem_rd<=1'b0;
            pc_ins_type_reg<=2'b00;
        end
        else if(IDBufEn) begin
            PC_curr_reg<=PC_curr;
            Mem_rd<=op_minterms[LOAD];
            pc_ins_type_reg<=pc_ins_type;
        end
    end
    
    // Source Register fetchers
    
    // ALU Operation Select

    localparam ADD=8'd1,SUB=8'd3,AND=8'd4,OR=8'd8,XOR=8'd16,SLL=8'd32,SRL=8'd64,SRA=8'd128;
    
    localparam BEQ=3'd0,BNE=3'd1,BLTU=3'd2,BGEU=3'd3;
    
    //
    localparam ADD_=10'd0,SLT_=10'd1,SLTU_=10'd2,AND_=10'd3,OR_=10'd4,XOR_=10'd5,SLL_=10'd6,SRL_=10'd7,SUB_=10'd256,SRA_=10'd257;
    
    localparam ADDI=8'd0,SLTIU=8'd1,ANDI=8'd2,ORI=8'd3,XORI=8'd4,SLLI=8'd5,SRLI=8'd6,SRAI=8'd7;
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0)
            ALUOp<=8'b0;
        else if(IDBufEn) begin
            casez({opcode,funct7,funct3})
                {OP,{7{1'b?}},ADDI}:
                    ALUOp<=ADD;
                {OP,{7{1'b?}},SLTIU}:
                    ALUOp<=SUB;
                {OP,{7{1'b?}},ANDI}:
                    ALUOp<=AND;
                {OP,{7{1'b?}},ORI}:
                    ALUOp<=OR;
                {OP,{7{1'b?}},XORI}:
                    ALUOp<=XOR;
                {OP,{7{1'b?}},SLL}:
                    ALUOp<=SLL;
                {OP,{7{1'b?}},SRL}:
                    ALUOp<=SRL;
                {OP,{7{1'b?}},SRA}:
                    ALUOp<=SRA;
                {OP_IMM,ADD_}:
                    ALUOp<=ADD;
                {OP_IMM,SLT_},{OP_IMM,SLTU_},{OP_IMM,SUB_}:
                    ALUOp<=SUB;
                {OP_IMM,AND_}:
                    ALUOp<=AND;
                {OP_IMM,OR_}:
                    ALUOp<=OR;
                {OP_IMM,XOR_}:
                    ALUOp<=XOR;
                {OP_IMM,SLL_}:
                    ALUOp<=SLL;
                {OP_IMM,SRL_}:
                    ALUOp<=SRL;
                {OP_IMM,SRA_}:
                    ALUOp<=SRA;
                {LUI,{9{1'b?}}},{AUIPC,{9{1'b?}}}:
                    ALUOp<=ADD;
                {BRANCH,{7'b?},BEQ},{BRANCH,{7'b?},BNE}:
                    ALUOp<=XOR;
                {BRANCH,{7'b?},BGEU},{BRANCH,{7'b?},BLTU}:
                    ALUOp<=SUB;
                default:
                    ALUOp<=ADD;         // Can be used for JAL and JALR
            endcase            
        end
    end
      
    // Sending SGN signal to discrminate between signed and unsigned operations
    
    // SRAI (Arithmetic Right Shift) is also considered a "signed operation"
    // LUI, AUIPC are also "signed" instructions but the extra (33rd) sign bit is not needed, so to simplify hardware, they are not considered
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==0)
            sgn<=1'b0;
        else if(IDBufEn) begin
            casez({opcode,funct7,funct3})
                {OP_IMM,{10{1'b?}}},{OP,SLT_},{OP,SRA_},{BRANCH,{7{1'b?}},BLTU},{BRANCH,{7{1'b?}},BGEU}:
                    sgn<=1'b1;
                default:
                    sgn<=1'b0;
            endcase
        end
    end
    
    // Updating isBranch
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0)
            isBranch<=1'b0;
        else if(IDBufEn)
            isBranch<=op_minterms[BRANCH];
    end
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0)
            branch_type<=4'b0;
        else if(IDBufEn) begin
            case(funct3)
                BEQ:
                    branch_type<=4'b0001;
                BNE:
                    branch_type<=4'b0010;
                BLTU:
                    branch_type<=4'b0100;
                default:
                    branch_type<=4'b1000;
            endcase
        end
    end
                              
    // Stall and flush signals
    
    generate
        for(i1=0;i1<2;i1=i1+1) begin
            always @(*) begin
                casez({neq[i1],cmp[3*i1+:3],Mem_rd_in})
                    // {3'b?10,3'b?0?} means, one operand register which is the destination register of an alu instruction is at MEM stage
                    //      (result ready in EXE buffer, so no stall required)
                    // {3'b100,3'b???} means, one operand register which is the destination register at WB stage
                    //      (result ready in MEM buffer, so no stall required, may it be alu or LOAD instruction)
                    // {3'b000,3'b???} means no matching, no dependency, no stall
                    {1'b1,3'b?10,1'b0},{1'b1,3'b100,1'b?},{1'b1,3'b000,1'b?}:
                        RAW_stall[i1]=1'b0; 
                    default: 
                        RAW_stall[i1]=1'b1;
                endcase
            end        
        end
    endgenerate

    assign IDHold=RAW_stall[0]|RAW_stall[1];
endmodule

module SR_Reg(
    input d,s,r,clk,g,
    output reg q
    );
    always @(posedge clk or negedge r or negedge s) begin
        if(~r) q<=1'b0;
        else if(~s) q<=1'b1;
        else if(g) q<=d;
    end
endmodule

module Pipeline_Handler(
    input clk,
    input rst,
    input flush,
    input IFHold,IDHold,EXEHold,MEMHold,WBHold,                 // All hold signals are active high
    output PCGenEn,IFBufEn,IDBufEn,EXEBufEn,MEMBufEn,WBBufEn            // flush_ is jmp_flush_&mispred_flush_
);
    reg[5:1] Enable_Reg;
    reg[5:0] Buf_Enable;                                        // 0 - PCGen, 1 - Instruction Fetch, ... , 5 - Write Back

    wire[4:0] stall={IFHold,IDHold,EXEHold,MEMHold,WBHold};

    integer i;
    genvar g;

    always @(*) begin
        Buf_Enable[5]=~WBHold;
        for(i=4;i>=1;i=i-1)  
            Buf_Enable[i]=~(stall[i]&Buf_Enable[i+1]&Enable_Reg[i]);
        Buf_Enable[0]=flush|Buf_Enable[1];                      // PCGen may be stopped due to traffic, but in case of misprediction flush, PCGen still needs to work
    end

    generate
        for(g=1;g<5;g=g+1) begin
            always @(posedge clk or negedge rst or negedge Buf_Enable[g]) begin
                if(rst==1'b0)
                    Enable_Reg[g]<=1'b1;
                else if(Buf_Enable[g]==1'b0)
                    Enable_Reg[g]<=1'b0;
                else
                    Enable_Reg[g]<=Enable_Reg[g+1];
            end
        end
    endgenerate
    
    assign {WBBufEn,MEMBufEn,EXEBufEn,IDBufEn,IFBufEn,PCGenEn}={Enable_Reg[5:4],Enable_Reg[3:1]&{3{~flush}},Buf_Enable[0]};
endmodule
