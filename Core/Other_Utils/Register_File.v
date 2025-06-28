`timescale 1ns / 1ps

module Reg_File(
    input [31:0] reg_w,
//    input update_enable,                    // Signalled by the Encode Unit when an instruction has a specific register as the destination operand
//                                            // Thus indicating that the specific register value stored in the register file is outdated until written in the corresponding Write-Back cycle
    input write_enable,
    input clk,
    input rst,
    input [4:0] rs1_enc,
    input [4:0] rs2_enc,
    input [4:0] reg_enc_write,
//    input [4:0] reg_dest_enc,               // Destination register rd, coupled with update_enable for updating "outdated" register
    output [31:0] rs1,
    output [31:0] rs2
//    output reg[31:0] outdated               // Register to indicate whether the specific register value is updated, or to direct to operand forwarding
    );
    
//    write_gate    -> Clock gate for the updation of registers
//    status_gate   -> Clock gate for "outdated" status of the registers
    
//    wire [31:0] write_route,status_route,reg_decoder_wr,reg_decoder_dest;
        
    reg [31:0] x[31:0];                     // Register File
                
    reg [31:0] rs[1:0];
    wire [4:0] rs_enc[1:0];
    integer i;
    genvar j;
    
//    // Register decoders for source operand registers rs1 and rs2
//    Decoder32 d1(.i_5(rs1_enc),
//                 .reg_decoder(reg_decoder_rs1));
//    Decoder32 d2(.i_5(rs2_enc),
//                 .reg_decoder(reg_decoder_rs2));
    
    // Register decoder for routing Write-Back
//    Decoder32 d3(.i_5(reg_enc_write),
//                 .reg_decoder(reg_decoder_wr));
                 
//    // Register decoder for updating corresponding "outdated" register for destination operand
//    Decoder32 d4(.i_5(reg_dest_enc),
//                 .reg_decoder(reg_decoder_dest));
    
    // Write port
    always @(posedge clk or negedge rst) begin
        if(rst==1'b0)
            for(i=0;i<32;i=i+1) x[i]<=31'b0;
        else begin
            case({write_enable,reg_enc_write})
                6'd32:	x[0]<=reg_w;
                6'd33:	x[1]<=reg_w;
                6'd34:	x[2]<=reg_w;
                6'd35:	x[3]<=reg_w;
                6'd36:	x[4]<=reg_w;
                6'd37:	x[5]<=reg_w;
                6'd38:	x[6]<=reg_w;
                6'd39:	x[7]<=reg_w;
                6'd40:	x[8]<=reg_w;
                6'd41:	x[9]<=reg_w;
                6'd42:	x[10]<=reg_w;
                6'd43:	x[11]<=reg_w;
                6'd44:	x[12]<=reg_w;
                6'd45:	x[13]<=reg_w;
                6'd46:	x[14]<=reg_w;
                6'd47:	x[15]<=reg_w;
                6'd48:	x[16]<=reg_w;
                6'd49:	x[17]<=reg_w;
                6'd50:	x[18]<=reg_w;
                6'd51:	x[19]<=reg_w;
                6'd52:	x[20]<=reg_w;
                6'd53:	x[21]<=reg_w;
                6'd54:	x[22]<=reg_w;
                6'd55:	x[23]<=reg_w;
                6'd56:	x[24]<=reg_w;
                6'd57:	x[25]<=reg_w;
                6'd58:	x[26]<=reg_w;
                6'd59:	x[27]<=reg_w;
                6'd60:	x[28]<=reg_w;
                6'd61:	x[29]<=reg_w;
                6'd62:	x[30]<=reg_w;
                6'd63:	x[31]<=reg_w;
            endcase
        end
    end
    
//    always @(posedge clk or negedge rst) begin
//        if(rst==1'b0)
//            for(i=0;i<32;i=i+1) x[i]<=31'b0;
//        else begin
//            if(write_route[0])	x[0]<=reg_w;
//            if(write_route[1])	x[1]<=reg_w;
//            if(write_route[2])	x[2]<=reg_w;
//            if(write_route[3])	x[3]<=reg_w;
//            if(write_route[4])	x[4]<=reg_w;
//            if(write_route[5])	x[5]<=reg_w;
//            if(write_route[6])	x[6]<=reg_w;
//            if(write_route[7])	x[7]<=reg_w;
//            if(write_route[8])	x[8]<=reg_w;
//            if(write_route[9])	x[9]<=reg_w;
//            if(write_route[10])	x[10]<=reg_w;
//            if(write_route[11])	x[11]<=reg_w;
//            if(write_route[12])	x[12]<=reg_w;
//            if(write_route[13])	x[13]<=reg_w;
//            if(write_route[14])	x[14]<=reg_w;
//            if(write_route[15])	x[15]<=reg_w;
//            if(write_route[16])	x[16]<=reg_w;
//            if(write_route[17])	x[17]<=reg_w;
//            if(write_route[18])	x[18]<=reg_w;
//            if(write_route[19])	x[19]<=reg_w;
//            if(write_route[20])	x[20]<=reg_w;
//            if(write_route[21])	x[21]<=reg_w;
//            if(write_route[22])	x[22]<=reg_w;
//            if(write_route[23])	x[23]<=reg_w;
//            if(write_route[24])	x[24]<=reg_w;
//            if(write_route[25])	x[25]<=reg_w;
//            if(write_route[26])	x[26]<=reg_w;
//            if(write_route[27])	x[27]<=reg_w;
//            if(write_route[28])	x[28]<=reg_w;
//            if(write_route[29])	x[29]<=reg_w;
//            if(write_route[30])	x[30]<=reg_w;
//            if(write_route[31])	x[31]<=reg_w;
//        end
//    end
    
//    always @(posedge clk or negedge rst) begin
//        if(rst==1'b0)   outdated<=32'b0;
//        else    outdated<=(outdated&(~write_route))|((~outdated)&status_route);
//    end    
    
    // Read ports (rs1 and rs2 operands)     
    generate
        for(j=0;j<2;j=j+1) begin
            always @(*) begin
                case(rs_enc[j])
                    5'd0:	rs[j]=x[0];
                    5'd1:	rs[j]=x[1];
                    5'd2:	rs[j]=x[2];
                    5'd3:	rs[j]=x[3];
                    5'd4:	rs[j]=x[4];
                    5'd5:	rs[j]=x[5];
                    5'd6:	rs[j]=x[6];
                    5'd7:	rs[j]=x[7];
                    5'd8:	rs[j]=x[8];
                    5'd9:	rs[j]=x[9];
                    5'd10:	rs[j]=x[10];
                    5'd11:	rs[j]=x[11];
                    5'd12:	rs[j]=x[12];
                    5'd13:	rs[j]=x[13];
                    5'd14:	rs[j]=x[14];
                    5'd15:	rs[j]=x[15];
                    5'd16:	rs[j]=x[16];
                    5'd17:	rs[j]=x[17];
                    5'd18:	rs[j]=x[18];
                    5'd19:	rs[j]=x[19];
                    5'd20:	rs[j]=x[20];
                    5'd21:	rs[j]=x[21];
                    5'd22:	rs[j]=x[22];
                    5'd23:	rs[j]=x[23];
                    5'd24:	rs[j]=x[24];
                    5'd25:	rs[j]=x[25];
                    5'd26:	rs[j]=x[26];
                    5'd27:	rs[j]=x[27];
                    5'd28:	rs[j]=x[28];
                    5'd29:	rs[j]=x[29];
                    5'd30:	rs[j]=x[30];
                    5'd31:	rs[j]=x[31];
                endcase
            end
        end
    endgenerate
    
//    assign status_route=reg_decoder_dest&{32{update_enable}};
//    assign write_route=reg_decoder_wr&{32{write_enable}};
    assign rs_enc[0]=rs1_enc,rs_enc[1]=rs2_enc,rs1=rs[0],rs2=rs[1];
    
endmodule

module Decoder32(
    input [4:0] i_5,
    output reg[31:0] reg_decoder
);
    always @(*) begin
        case(i_5)
            5'd0:	reg_decoder<=32'd1;
            5'd1:	reg_decoder<=32'd2;
            5'd2:	reg_decoder<=32'd4;
            5'd3:	reg_decoder<=32'd8;
            5'd4:	reg_decoder<=32'd16;
            5'd5:	reg_decoder<=32'd32;
            5'd6:	reg_decoder<=32'd64;
            5'd7:	reg_decoder<=32'd128;
            5'd8:	reg_decoder<=32'd256;
            5'd9:	reg_decoder<=32'd512;
            5'd10:	reg_decoder<=32'd1024;
            5'd11:	reg_decoder<=32'd2048;
            5'd12:	reg_decoder<=32'd4096;
            5'd13:	reg_decoder<=32'd8192;
            5'd14:	reg_decoder<=32'd16384;
            5'd15:	reg_decoder<=32'd32768;
            5'd16:	reg_decoder<=32'd65536;
            5'd17:	reg_decoder<=32'd131072;
            5'd18:	reg_decoder<=32'd262144;
            5'd19:	reg_decoder<=32'd524288;
            5'd20:	reg_decoder<=32'd1048576;
            5'd21:	reg_decoder<=32'd2097152;
            5'd22:	reg_decoder<=32'd4194304;
            5'd23:	reg_decoder<=32'd8388608;
            5'd24:	reg_decoder<=32'd16777216;
            5'd25:	reg_decoder<=32'd33554432;
            5'd26:	reg_decoder<=32'd67108864;
            5'd27:	reg_decoder<=32'd134217728;
            5'd28:	reg_decoder<=32'd268435456;
            5'd29:	reg_decoder<=32'd536870912;
            5'd30:	reg_decoder<=32'd1073741824;
            5'd31:	reg_decoder<=32'd2147483648;
        endcase
    end
endmodule        
