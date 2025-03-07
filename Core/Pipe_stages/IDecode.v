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
    // Dummy inputs (verification purpose)
//    input jump_stall_,mem_stall_,
    
    input [31:0] Ins,           // Instruction Fetch stage buffer
    input [31:0] PC_curr,       // Current PC value sent thourgh the pipeline
    input rst,
    input clk_stage,
    input clk_stage_en,
    
    // Input from EXE stage
    input misprediction_,       // Indicates that previous instruction was a mispredicted branch so pipeline flush has to be done
    
    // Inputs from Register File
    input [31:0] regfile1,      // Source register rs1 fetched from register file
    input [31:0] regfile2,      // Source register rs2 fetched from register file
    
    // Write port to Register File
    
    input [31:0] WB_val,
    
    // Operand Forward values
//    input [31:0] EXEBuf,        // EXE Unit Buffer value (in reality they are the buffer inputs, not the buffer values)
//    input [31:0] MEMBuf,        // MEM Unit Buffer value
    
    // Output ports that go to register file
    output reg [4:0] rs1_enc,   
//    output [31:0] opr1,
    output reg [4:0] rs2_enc,
    output reg [31:0] PC_curr_reg,
    
    // Jump Target output
//    output reg [31:0] jmp_target,
    
    // Output ports that go to EXE Unit
//    output reg ALUEn,           // ALU Enable (Data Bypasses processing by ALU and directly reaches EXE Buffer)
    output EXEBufEn,            // Clock gate for EXE Unit Buffer
    output reg [7:0] ALUOp,     // ALU operation select (Decoded in this stage itself so can be directly used by ALU)
//    output reg ALUOp_w
    output reg sgn,             // Indicates whether the operation is signed or unsigned
    output reg isBranch,        // Indicates whether the current instruction is a BRANCH, enabling the EXE Unit to send misprediction_ signal whenever applicable
    output reg branch_type,     // Indicates the type of BRANCH instruction (Decoded funct3 argument)
    output reg [31:0] ALUopr1,  // ALU Operands
    output reg [31:0] ALUopr2,
    output reg [3:0] Opr_src1,  // Indicates from where the true operand has to be fetched (one-hot encoded)
    output reg [3:0] Opr_src2,
    output reg [31:0] Store_src,
    
    output [31:0] rs1,          // IF stage also requires register values for JALR instruction
    output [31:0] rs2,
    
    output [1:0] MEMEn,         // Enables or Bypasses MEM unit
    output MEMBufEn,            // Clock gate for MEM unit
    output IFBufEn,             // Pipeline stall enable (Active low)
    output WBEn
    );
    
    wire[6:0] opcode=Ins[6:0];
    reg[10:0] op_minterms;      // Minterms generated on decoding the opcode field
    wire[5:0] ins_type;         // Type of the instruction (R,I,S,B,U,J)
    
    wire[4:0] rd_enc;           // Destination register encoding for the current instruction cycle
    reg[31:0] rs[1:0];
    
    // Immediate Field
    
    wire[31:0] immediate;
    wire imm31,imm11,imm0;
    wire[10:0] imm30_20;
    wire[7:0] imm19_12;
    wire[5:0] imm10_5;
    wire[3:0] imm4_1;
    
    reg[31:0] PC_reg;
    
    reg[3:0] cmp;               // Compare current Destination register with outdated registers
    wire[4:0] rs_enc[1:0];      // regfile[1:0];
    wire[31:0] regfile[1:0];
    
    assign regfile[0]=regfile1,regfile[1]=regfile2;
    assign rs_enc[0]=rs1_enc,rs_enc[1]=rs2_enc,regfile[0]=regfile1,regfile[1]=regfile2;
    
    // Outdated registers
    // If the Destination Register of previous Instruction cycle is required as an operand in current cycle
    // If previous instruction was a LOAD instruction then its true value will only be found after the next cycle (In MEM Unit Buffer), hence stall required
    // Else, it can be retrieved from EXE Unit Buffer without the need of stall
    // In the other case, where the Destination Register of the previous to previous Instruction cycle is required, no stall is needed
    // As the Register value is present at the MEM Unit Buffer, irrespective of that Instruction type
    reg[4:0] outdated[2:0];     // Record of Destination Register for past two Instruction cycles
    reg MEM_rd;                 // Tells that the previous instruction is a Memory Access instruction and hence operand cannot be forwarded
                                // in the current cycle (from EXE stage) and hence a stall of 1 cycle is needed to fetch it from MEM stage
    reg[1:0] MEMEnBuf[1:0];
    reg mem_stall_;
    wire exe_stall_,stall_;                 
    
    // Stall_ signals for IF, ID, EXE, MEM and WB units
    wire IDBufEn,full_;
    
    wire[2:0] funct3;
    wire[6:0] funct7;
    
    // Opcode type
    localparam OP=6'd0,OP_IMM=6'd1,JAL=6'd2,JALR=6'd3,BRANCH=6'd4,LOAD=6'd5,STORE=6'd6,LUI=6'd7,AUIPC=6'd8;
    
    // Instruction type
    localparam R=0,I=1,S=2,B=3,U=4,J=5;  
    
    assign funct3=Ins[14:12],funct7=Ins[31:25];
    assign rd_enc=Ins[11:7]&{5{~{op_minterms[BRANCH]|op_minterms[STORE]}}};
    assign rs_enc[0]=Ins[19:15],rs_enc[1]=Ins[24:20];
    assign rs1=regfile1,rs2=regfile2;
//    assign rs1=rs[0],rs2=rs[1];
    
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
    assign ins_type[I]=op_minterms[OP_IMM]|op_minterms[JALR]|op_minterms[LOAD];
    assign ins_type[S]=op_minterms[STORE];
    assign ins_type[B]=op_minterms[BRANCH];
    assign ins_type[U]=op_minterms[LUI]|op_minterms[AUIPC];
    assign ins_type[J]=op_minterms[JAL];
    
    // Destination register record
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0) begin
            outdated[0]<=5'b0;
            outdated[1]<=5'b0;
            outdated[2]<=5'b0;
        end
        else begin // if(stall_) begin
            outdated[0]<=rd_enc&{5{IDBufEn}};   
            outdated[1]<=outdated[0];
            outdated[2]<=outdated[1];
        end
    end  
            
    generate
    genvar i1,j1;
    for(i1=0;i1<2;i1=i1+1) begin        
        for(j1=0;j1<2;j1=j1+1) begin
            always @(*) begin
                case(opcode)
                    OP,OP_IMM,JALR,LOAD,LUI,AUIPC,JAL:
                        cmp[2*i1+j1]<=(outdated[j1]==rs_enc[i1]);
                    default:
                        cmp[2*i1+j1]<=5'b0;
                endcase
            end
        end
    end
    endgenerate    
    
//    generate
//        genvar i;
//        for(i=0;i<2;i=i+1) begin
//            always @(*) rs[i]<=({32{cmp[2*i]}}&EXEBuf)|({32{cmp[2*i+1]}}&MEMBuf)|(~({32{cmp[2*i]|cmp[2*i+1]}})&regfile[i]);
//        end
//    endgenerate
    
    // Immediate field fetcher
    
    wire w1=ins_type[I]|ins_type[S]|ins_type[B],w2=w1|ins_type[J];
    
    assign imm31=Ins[31];
    assign imm30_20=(w2)?{11{Ins[31]}}:Ins[30:20];
    assign imm19_12=(w1)?{8{Ins[31]}}:Ins[19:12];
    assign imm11=(Ins[31]&(ins_type[I]|ins_type[S]))|(Ins[7]&ins_type[B])|(Ins[20]&ins_type[J]);
    assign imm10_5=Ins[30:25]&{6{w2}};
    assign imm4_1=(Ins[24:21]&{4{ins_type[I]|ins_type[J]}})|(Ins[11:8]&{4{ins_type[S]|ins_type[B]}});
    assign imm0=(Ins[20]&ins_type[I])|(Ins[7]&ins_type[S]);
    
    assign immediate={imm31,imm30_20,imm19_12,imm11,imm10_5,imm4_1,imm0};
    
    // Operand update
    
    wire r_w=ins_type[R]|ins_type[B];
    
    wire[1:0] neq={(rs2_enc!=5'b0),(rs1_enc!=5'b0)};
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==0) begin
            ALUopr1<=32'b0;
//            ALUopr2<=32'b0;
        end
        else if(IDBufEn) begin                                      // cmp[2]: Carries the destination reg encoding of an instruction three cycles ago
            ALUopr1<=(neq[0]&(rs1_enc==outdated[2]))?WB_val:regfile1;    // So, forwarding is done from the write port to the Register File, if needed
//            ALUopr2<=(neq[1]&(rs2_enc==outdated[2]))?WB_val:regfile2;    // This is becuase the current instruction is in Decode stage, while the actual value of the
//            ALUopr1<=(r_w|ins_type[I]|ins_type[S])?rs1:PC_curr;   // required register is being written to the Regitsr File in the WB stage
//            ALUopr2<=(r_w)?rs2:immediate;                         // Strictly speaking, it will be written at the end of the current cycle
        end    
    end
    
    always @(posedge clk_stage or negedge rst) begin                // Have to select immediate field when applicable
        if(rst==0)
            ALUopr2<=32'b0;
        else if(IDBufEn) begin
            case({opcode,neq[0]&(rs1_enc==outdated[2])})
                {OP,1'b1},{BRANCH,1'b1},{STORE,1'b1}:
                    ALUopr2<=WB_val;
                {OP,1'b0},{BRANCH,1'b0},{STORE,1'b0}:
                    ALUopr2<=regfile2;                    
                default:
                    ALUopr2<=immediate;
            endcase
        end
    end
//                OP_IMM,LUI,AUIPC,LOAD,STORE:                                   
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==0)
            Store_src<=32'b0;
        else if(IDBufEn&ins_type[S])
            Store_src<=rs2;
    end
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==0)
            PC_curr_reg<=32'b0;
        else if(IDBufEn&ins_type[B])
            PC_curr_reg<=PC_curr;
    end
    
    // Source Register fetchers
    
    // Op_src encoding
    
    // 3'b100: No forwarding
    // 3'b010: Forwarding from EXE Stage Buffer
    // 3'b001: Forwarding from MEM Stage Buffer
    
    // Difference in processing between Jump and Branch instructions
    // For Branch instructions, the operands carried into the Execute stage are the ones to be evaluated on
    // For Jumps, the PC and immediate are carried as operands, no dedicated adder for jump target calculation
    
    always @(posedge clk_stage or negedge rst) begin        // Opr1 is a register (rs1) when instruction is of type R,I,S,B so forwarding may be required
        if(rst==1'b0)                                       // In case if type U,J, forwarding is not required as operand is immediate
            Opr_src1<=4'b0;
        else if(IDBufEn) begin
            case(ins_type[U])                                          
                1'b1:
                    Opr_src1<=4'b1000;                      // Indicates that operand is current PC (LUI or AUIPC)
                default:
                    Opr_src1<=(cmp[0]&(~MEM_rd)&neq[0])?4'b0010:(cmp[1])?4'b0001:4'b0100;   // Priority of new instruction over older one
            endcase
        end
    end
    
    always @(posedge clk_stage or negedge rst) begin        // Opr2 is a register (rs1) when instruction is of type R,I,S,B so forwarding may be required
        if(rst==1'b0)                                       // In case if type I,U,J, forwarding is not required as operand is immediate
            Opr_src2<=3'b0;
        else if(IDBufEn) begin
            case(ins_type)
                I,U,J:
                    Opr_src2<=3'b100;                       // Indicates forarding not required
                default:
                    Opr_src2<=(cmp[2]&(~MEM_rd)&neq[1])?3'b010:(cmp[3])?3'b001:3'b100;   // Priority of new instruction over older one
            endcase
        end
    end      
    
    // ALU Operation Select

    localparam ADD=8'd1,SUB=8'd3,AND=8'd4,OR=8'd8,XOR=8'd16,SLL=8'd32,SRL=8'd64,SRA=8'd128;
    
    localparam BEQ=3'd0,BNE=3'd1,BLTU=3'd2,BGEU=3'd3;
    
    //
    localparam ADD_=10'd0,SLT_=10'd1,SLTU_=10'd2,AND_=10'd3,OR_=10'd4,XOR_=10'd5,SLL_=10'd6,SRL_=10'd7,SUB_=10'd256,SRA_=10'd257;
    
    localparam ADDI=8'd0,SLTIU=8'd1,ANDI=8'd2,ORI=8'd3,XORI=8'd4,SLLI=8'd5,SRLI=8'd6,SRAI=8'd7;
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==0)
            ALUOp<=8'b0;
        else if(IDBufEn) begin
            case(opcode)
                OP:
                    case(funct3)
                        ADDI:
                            ALUOp<=ADD;
                        SLTIU:
                            ALUOp<=SUB;
                        ANDI:
                            ALUOp<=AND;
                        ORI:
                            ALUOp<=OR;
                        XORI:
                            ALUOp<=XOR;
                        SLLI:
                            ALUOp<=SLL;
                        SRLI:
                            ALUOp<=SRL;
                        default:
                            ALUOp<=SRA;
                    endcase     
                OP_IMM:
                    case({funct7,funct3})
                        ADD_:
                            ALUOp<=ADD;
                        SLT_,SLTU_,SUB_:
                            ALUOp<=SUB;
                        AND_:
                            ALUOp<=AND;
                        OR_:
                            ALUOp<=OR;
                        XOR_:
                            ALUOp<=XOR;
                        SLL_:
                            ALUOp<=SLL;
                        SRL_:
                            ALUOp<=SRL;
                        default:
                            ALUOp<=SRA;
                    endcase    
                // LUI,AUIPC,JAL,JALR:
                LUI, AUIPC:    
                    ALUOp<=ADD;
                BRANCH:
                    case(funct3)
                        BEQ,BNE:
                            ALUOp<=XOR;
                        default:
                            ALUOp<=SUB;
                    endcase
                default:
                    ALUOp<=ADD;                         // Can be used for JAL and JALR            
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
            case(opcode)
                OP_IMM:
                    sgn<=(funct3==SRAI);
                OP:
                    case({funct7,funct3})
                        SLT_,SRA_:
                            sgn<=1'b1;
                        default:
                            sgn<=1'b0;
                    endcase
                BRANCH:
                    case(funct3)
                        BLTU,BGEU:
                            sgn<=1'b1;
                        default:
                            sgn<=1'b0;
                    endcase
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
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==1'b0)
            MEM_rd<=1'b0;
        else
            MEM_rd<=op_minterms[LOAD];
    end
    
    always @(*) begin
        case(rd_enc)
            5'b0:
                mem_stall_<=1'b1;
            default:
                mem_stall_<=~((cmp[0]|cmp[2])&MEM_rd);
        endcase
    end    
    
    assign exe_stall_=1'b1;                         // Functionality to be added in future
    assign stall_=mem_stall_&exe_stall_;
    
    assign jmp_flush_=op_minterms[JAL]|op_minterms[JALR];
    assign flush_=jmp_flush_&misprediction_;
    
    // Enable signals for all pipeline stage buffers
    
    assign MEMEn_w={op_minterms[LOAD]|op_minterms[STORE],op_minterms[STORE]};
    assign MEMEn=MEMEnBuf[1];
    
    always @(posedge clk_stage or negedge rst) begin
        if(rst==0) begin
            MEMEnBuf[0]<=2'b0;
            MEMEnBuf[1]<=2'b0;
        end
        else if(stall_) begin
            MEMEnBuf[0]<=MEMEn_w;
            MEMEnBuf[1]<=MEMEnBuf[0];
        end
    end
     
    Gate_stages gs(
        .exe_stall_(exe_stall_),
        .stall_(stall_),
        .mispred_flush_(misprediction_),
        .flush_(flush_),
        .clk(clk_stage_en),
        .rst(rst),
        .IFBufEn(IFBufEn),
        .IDBufEn(IDBufEn),
        .EXEBufEn(EXEBufEn),
        .MEMBufEn(MEMBufEn),
        .WBBufEn(WBEn)
    );
    
    // Adder for Jump target calculation
    
//    assign jmp_target=PC_Curr+((op_minterms[JAL])?immediate:
    
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

module Gate_stages(
    input exe_stall_,stall_,mispred_flush_,flush_,clk,rst,         // Here stall_ is the total AND of all possible stall signals
    output IFBufEn,IDBufEn,EXEBufEn,MEMBufEn,WBBufEn               // flush_ is jmp_flush_&mispred_flush_
);
    reg r_mem_stall_,r_stall_,full_;
    
    reg[4:0] Enable_Reg;
    wire[4:0] Enable_w; 
    
    generate
    genvar i;
        for(i=0;i<5;i=i+1) begin
            always @(posedge clk or negedge rst) begin
                if(rst==1'b0)
                    Enable_Reg[i]<=1'b1;
                else if(full_)
                    Enable_Reg[i]<=Enable_w[i];
            end
        end
    endgenerate

    always @(posedge clk or negedge rst) begin
        if(rst==1'b0) r_stall_<=1'b1;
        else if(full_) r_stall_<=stall_;
    end
    
    always @(*) begin
        case({Enable_Reg[4:2],Enable_w[2:0]})
            6'b111111:
                full_<=1'b0;
            default:
                full_<=1'b1;
        endcase
    end
    
    assign Enable_w[2:0]={exe_stall_&Enable_Reg[1],stall_&mispred_flush_&Enable_Reg[0],flush_};
    assign Enable_w[4:3]=Enable_Reg[3:2];
    
    assign {WBBufEn,MEMBufEn,EXEBufEn,IDBufEn,IFBufEn}={Enable_Reg[4:2],Enable_Reg[1:0]&{2{r_stall_}}};
endmodule
