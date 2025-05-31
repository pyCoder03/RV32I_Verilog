// `include "C:/Users/sbala/CISC_16bit/CISC_16bit.srcs/sources_1/new/Dual_edge_triggered_DFF.v"

// Bus Interface Handshaking for Multiplexing of Address and Data Bus
module Bus_Interface_Unit(ale,den_,rd_,wr_,dtr_syn,dtr_,busint,t_state);
	input dtr_,busint;
	input[2:0] t_state=3'b000;
	output rd_,wr_,ale,den_;
	output reg dtr_syn;

	reg rd_=1'b1,wr_=1'b1;
	reg busint_syn=0;
	wire sig,den_,a,b,ta,ale;

	wire t2,t1,t0;
	assign {t2,t1,t0}=t_state;
	//assign ta=t2&t1&t0;		Might not be a good practice
	//assign rd_=rd_,wr_=wr_,ale=ale_,den_=den_,sig=sig;

	//assign rd_=(
	always @(*) begin
	    if(busint_syn==0) begin
		rd_=1'b1;
		wr_=1'b1;
		end
	    else if(dtr_syn) begin
		wr_=sig;
		rd_=1'b1;
		end
	    else begin
		rd_=sig;
		wr_=1'b1;
		end
	end

	always @(negedge t2) begin
		dtr_syn<=dtr_;
		busint_syn<=busint;
	end

	assign a=t2&t1,b=~(t2|t1),sig=(~busint_syn)|a|b,den_=(~busint_syn)|(a&t0)|b,ale=busint_syn&~(t2|t1);		// sig is the actual handshaking signal, demultiplexed to rd_ or wr_
															// based on requirement, sig is high on 0 and 7 in the 8 t-states per clock

endmodule

// Bus Interface Handshaking for Separate Address and Data Communication
module BIU_Non_Multiplexed#(parameter WIDTH=32)(ADDR,den_,rd_,wr_,stall_,dtr_,busint,t_state);
	input[WIDTH-1:0] ADDR;
	input dtr_,busint,stall_;
	input[2:0] t_state=3'b000;
	output rd_,wr_,den_;
	//    output reg dtr_syn;
	    
	//    reg busint_syn=0;
	wire t2,t1,t0,sig,den_,a,b,ta,rw_gate;
	    
	assign rw_gate=(~busint)|~(stall_)|sig;
	assign rd_=rw_gate|(~dtr_),wr_=rw_gate|dtr_;
	    
	//    always @(negedge ta) begin
	//        dtr_syn<=dtr_;
	//        busint_syn<=busint;
	//    end
	    
	assign {t2,t1,t0}=t_state,ta=t2&t1&t0;
	assign sig=(~busint)|ta;
endmodule 

//module Data_Bus(En,clk,ADBus);
//input En,clk;
//output reg[31:0] ADBus;
//always @(posedge clk)

//endmodule
