`timescale 1ns / 1ps

module LRU_test;
    reg clk;
    reg rst;
    reg select;             // Module Select input
    reg [TAG_WIDTH-1:0] input_tag;
    reg [VALUE_WIDTH-1:0] new_value;
    reg RD_;                // Read enable            
    reg WR_;                // Write enable
    reg is_new;
    
    reg [TAG_WIDTH-1:0] input_tag_syn;
    reg [VALUE_WIDTH-1:0] new_value_syn;
    reg RD_syn;
    reg WR_syn;
    reg is_new_syn;
    
    wire cache_miss;
    wire memwrite;          // Writing back dirty cache contents to lower memory hierarchy in case of eviction
    wire [TAG_WIDTH-1:0] tag_write;
    wire [VALUE_WIDTH-1:0] value_write;

    localparam NUM_LINES=4; 
    localparam VALUE_WIDTH=32;
    localparam TAG_WIDTH=4;

    LRU_Cache DUT(
        .clk(clk),
        .rst(rst),
        .select(select),
        .input_tag(input_tag_syn),
        .new_value(new_value_syn),
        .RD_(RD_syn),
        .WR_(WR_syn),
        .is_new(is_new_syn),
        .cache_miss(cache_miss),
        .memwrite(memwrite),
        .tag_write(tag_write),
        .value_write(value_write)
    );

    always @(posedge clk or negedge rst) begin
        if(rst==1'b0) begin
            input_tag_syn<={TAG_WIDTH{1'b0}};
            new_value_syn<={VALUE_WIDTH{1'b0}};
            RD_syn<=1'b1;
            WR_syn<=1'b1;
            is_new_syn<=1'b0;
        end
        else begin
            input_tag_syn<=input_tag;
            new_value_syn<=new_value;
            RD_syn<=RD_;
            WR_syn<=WR_;
            is_new_syn<=is_new;
        end
    end

    task Read(
        input [TAG_WIDTH-1:0] input_tag_arg
    );
        begin
            @(negedge clk);
            #1
            RD_=1'b0;
            input_tag=input_tag_arg;      
            @(negedge clk);
            #1 
            RD_= 1'b1; 
        end
    endtask

    task Write(
        input [TAG_WIDTH-1:0] input_tag_arg,
        input [VALUE_WIDTH-1:0] new_value_arg
    );
        begin
            @(negedge clk);
            #1
            WR_=1'b0;
            input_tag=input_tag_arg;
            new_value=new_value_arg;    
            @(negedge clk); 
            #1
            WR_= 1'b1;
        end
    endtask

    initial begin
        $dumpfile("./dumpfiles/LRU_dump.vcd");
        $dumpvars(0);

        clk=1'b0;
        rst=1'b0;
        select=1'b1;
        input_tag=4'b01;
        new_value=$urandom;
        RD_=1'b1;
        WR_=1'b1;
        is_new=1'b0;

        #10 
        rst=1'b1;

        #49 
        Write(4'd5,$urandom);

        for(integer i=6;i<12;i=i+1) begin
            #35
            Write(i,$urandom);
        end

        for(integer i=9;i<12;i=i+1) begin
            #35
            Read(i);
        end
        
        for(integer i=13;i<15;i=i+1) begin
            #35
            Read(i);
            #35
            Write(i,)
        end

        #20 $finish();
    end

    always #5 clk=~clk;

endmodule