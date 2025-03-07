`timescale 1ns / 1ps

module LRU_Cache #(parameter VALUE_WIDTH=8,TAG_WIDTH=8,NUM_PAIRS=8)(
    input clk,
    input rst,
    input [TAG_WIDTH-1:0] search_tag,
    input [TAG_WIDTH-1:0] new_tag,
    input [VALUE_WIDTH-1:0] new_value,
    input mem_RW,           // 1 - Write, 0 - Read
    input invalidate,       // When memory location is written, old value must be deleted
    output cache_miss
    );
    
    // WORKING
    
    // New write: Add to top and right-shift rest all
    // Overwrite: Bring the key value pair to top and alter the value, right shift till old position (inclusive)
    // Read: If hit occurs, bring to top and right shift till old position (inclusive)
    //      If cache miss occurs, it's the driving system (IF or MEM stage)'s job to store the item in cache
    
    reg[TAG_WIDTH-1:0] tags[NUM_PAIRS-1:0];
    reg[VALUE_WIDTH-1:0] values[NUM_PAIRS-1:0];
    reg[NUM_PAIRS-1:0] valid;
    
    wire[NUM_PAIRS-1:0] right_shift_en;     // When 
    wire[NUM_PAIRS-1:0] left_shift_en;      // To delete one entry
    wire[NUM_PAIRS-1:0] write_en;           // Clock gates
    
    reg[TAG_WIDTH-1:0] next_tag[NUM_PAIRS-1:0];
    reg[VALUE_WIDTH-1:0] next_value[NUM_PAIRS-1:0];
    
    wire[NUM_PAIRS-1:0] compare;
    
    integer i;
    
    // Comparing the input search tag with each and every valid tag stored in the cache
    
    generate
    genvar g;
        for(g=0;g<NUM_PAIRS;g=g+1) begin
            assign compare[g]=(search_tag==tags[g]);    
        end
    endgenerate
    
    //
    
    assign left_shift_en[0]=compare[0];
    
    for(g=1;g<NUM_PAIRS;g=g+1) begin
        assign left_shift_en[g]=compare[g]|left_shift_en[g-1];
    end
    
    assign right_shift_en=~left_shift_en;
    
    //
    
    always @(*) begin
        for(i=1;i<NUM_PAIRS;i=i+1) begin
            next_value[i]=values[i-1];
            next_tag[i]=tags[i-1];
        end
    end
    
    always @(*) begin
        case({cache_miss,mem_RW,compare})
            10'd1: begin
                next_value[0]=values[0];
                next_tag[0]=tags[0];
            end
            10'd2: begin
                next_value[0]=values[1];
                next_tag[0]=tags[1];
            end
            10'd4: begin
                next_value[0]=values[2];
                next_tag[0]=tags[2];
            end
            10'd8: begin
                next_value[0]=values[3];
                next_tag[0]=tags[3];
            end
            10'd16: begin
                next_value[0]=values[4];
                next_tag[0]=tags[4];
            end
            10'd32: begin
                next_value[0]=values[5];
                next_tag[0]=tags[5];
            end
            10'd64: begin
                next_value[0]=values[6];
                next_tag[0]=tags[6];
            end
            10'd128: begin
                next_value[0]=values[7];
                next_tag[0]=tags[7];
            end
            default: begin                   // Cache hit + Write and all Cache Misses
                next_value[0]=new_value;
                next_tag[0]=new_tag; 
            end
        endcase
    end        
    
    always @(posedge clk or negedge rst) begin
        if(rst==1'b0) begin
            for(i=0;i<NUM_PAIRS;i=i+1) begin
                tags[i]<={TAG_WIDTH{1'b0}};
                values[i]<={VALUE_WIDTH{1'b0}};
                valid[i]<={NUM_PAIRS{1'b0}};
            end
        end
        else begin
            for(i=0;i<NUM_PAIRS;i=i+1) begin
                if(write_en[i]) begin
                    tags[i]<=next_tag[i];
                    values[i]<=next_value[i];
                end
            end
        end
    end
endmodule

