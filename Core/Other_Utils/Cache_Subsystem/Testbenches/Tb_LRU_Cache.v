`timescale 1ns / 1ps
module LRU_test;
    reg clk,
    reg rst,
    reg select,             // Module Select input
    reg [TAG_WIDTH-1:0] input_tag,
    reg [VALUE_WIDTH-1:0] new_value,
    reg RD_,                // Read enable            
    reg WR_,                // Write enable
    
    wire cache_miss,
    wire memwrite,          // Writing back dirty cache contents to lower memory hierarchy in case of eviction
    wire [TAG_WIDTH-1:0] tag_write,
    wire [VALUE_WIDTH-1:0] value_write

    LRU_Cache DUT(
        .clk(clk),
        .rst(rst),
        .select(select),
        .input_tag(input_tag),
        .new_value(new_value),
        .RD_(RD_),
        .WR_(WR_),
        .cache_miss(cache_miss),
        .memwrite(memwrite),
        .tag_write(tag_write),
        .value_write(value_write)
    )

    initial begin
        clk=1'b0;
        rst=1'b0;
        select=1'b1;
        input_tag=$random;
        new_value=$random;
        RD_=1'b1;
        WR_=1'b1;

        
    end

    forever #5 clk=~clk;

endmodule