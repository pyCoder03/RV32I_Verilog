`timescale 1ns / 1ps

// Implementation of Least Recently Used (LRU) replacement policy
// Fully Associative Cache with 4 lines
module LRU_Cache#(
    parameter NUM_LINES=4,
    parameter VALUE_WIDTH=32,
    parameter TAG_WIDTH=4
    )(           
    input clk,
    input rst,
    input select,           // Module Select input
    input [TAG_WIDTH-1:0] input_tag,
    input [VALUE_WIDTH-1:0] new_value,
    input RD_,              // Read enable            
    input WR_,              // Write enable
    input is_new,           // Indicates if the block is freshly loaded from memory and written to cache (in that case the block is not dirty) 
    output cache_miss,
    output memwrite,        // Writing back dirty cache contents to lower memory hierarchy in case of eviction
    output [TAG_WIDTH-1:0] tag_write,
    output [VALUE_WIDTH-1:0] value_write
    );

    // WORKING
    
    // New write: Add to top and right-shift rest all
    // Overwrite: Bring the key value pair to top and alter the value, right shift till old position (inclusive)
    // Read: If hit occurs, bring to top and right shift till old position (inclusive)
    //      If cache miss occurs during read, it's the driving system (IF or MEM stage)'s job to store the item in cache
    
    reg[TAG_WIDTH-1:0] tags[NUM_LINES-1:0];
    reg[VALUE_WIDTH-1:0] values[NUM_LINES-1:0];
    reg[VALUE_WIDTH-1:0] read_buf;
    reg[NUM_LINES-1:0] valid;               // Indicates whether the value is valid (is it a reset value or a properly filled value)
    reg[NUM_LINES-1:0] dirty;               // In a write-back cache, dirty bit is used to indicate that upon eviction, the value must be updated to the next level cache/memory 

    wire[NUM_LINES-1:0] right_shift_en_hit;     // When
    wire[NUM_LINES-1:0] write_en;           // Clock gates
    
    reg[TAG_WIDTH-1:0] next_tag[NUM_LINES-1:0];
    reg[VALUE_WIDTH-1:0] next_value[NUM_LINES-1:0];
    reg next_dirty[NUM_LINES-1:0];

    reg cache_miss;
    
    wire[NUM_LINES-1:0] compare;

    // Simulation purpose

    wire[TAG_WIDTH-1:0] tags0=tags[0];
    wire[TAG_WIDTH-1:0] tags1=tags[1];
    wire[TAG_WIDTH-1:0] tags2=tags[2];
    wire[TAG_WIDTH-1:0] tags3=tags[3];

    wire[VALUE_WIDTH-1:0] values0=values[0];
    wire[VALUE_WIDTH-1:0] values1=values[1];
    wire[VALUE_WIDTH-1:0] values2=values[2];
    wire[VALUE_WIDTH-1:0] values3=values[3];
    
    integer i;
    genvar g;
    
    // REGISTER CONVENTION
    // LSB is in the left, MSB in the right
    // Data fed through LSB 

    // Comparing the input search tag with each and every valid tag stored in the cache
    
    generate
        for(g=0;g<NUM_LINES;g=g+1) begin
            assign compare[g]=(input_tag==tags[g])&valid[g];    
        end
    endgenerate
    
    //  Connecting the right shift enable signal by rippling with compare values (only the bits left to the hit position to be right shifted)
    
    assign right_shift_en_hit[NUM_LINES-1]=compare[NUM_LINES-1];
    
    generate
        for(g=0;g<NUM_LINES-1;g=g+1) begin
            assign right_shift_en_hit[g]=compare[g]|right_shift_en_hit[g+1];
        end
    endgenerate

    // Cache miss condition

    always @(*) begin
        case(valid&compare)
            4'd0:       cache_miss=1'b1;
            default:    cache_miss=1'b0;
        endcase
    end

    // Cache entries modification cases (Write-back cache)

    // -> There is a write miss: All entries to be shifted to allow new entry into the cache (last entry to be evicted if valid)
    // -> There is a read miss: No change in cache entries
    // -> Other cases: Partial shifting has to be done according to right_shift_en_hit signal
    // Given below is a logic-minimized circuit satisfying the above conditions

    assign write_en=({NUM_LINES{(~WR_)|(~RD_)}})&(right_shift_en_hit|{NUM_LINES{cache_miss}})&({NUM_LINES{(~cache_miss)|(~WR_)}});

    // Setting the input multiplexer for various cache hit conditions and the miss condition
    
    always @(*) begin
        case({WR_,compare})
            5'd17:  next_value[0]=values[0];
            5'd18:  next_value[0]=values[1];
            5'd20:	next_value[0]=values[2];
            5'd24:	next_value[0]=values[3];
            default:    next_value[0]=new_value;
        endcase
    end

    // Tag value at 0 (leftmost position) can be directly taken from input tag, irrespective of whether there is a hit or miss, a read or write

    always @(*) begin   
        next_tag[0]=input_tag;
    end

    always @(*) begin
        for(i=1;i<NUM_LINES;i=i+1) begin
            next_value[i]=values[i-1];
            next_tag[i]=tags[i-1];
        end
    end

    // Setting the multiplexer for LSB dirty bit (dirty bit is made 1 if a write is made)

    always @(*) begin
        case({WR_,compare})
            6'd17:  next_dirty[0]=dirty[0];
            6'd18:  next_dirty[0]=dirty[1];
            6'd20:  next_dirty[0]=dirty[2];
            6'd24:  next_dirty[0]=dirty[3];
            default:    next_dirty[0]=~is_new;
        endcase
    end

    always @(*) begin
        for(i=1;i<NUM_LINES;i=i+1) begin
            next_dirty[i]=dirty[i-1];
        end
    end

    // Read buffer update (NO_CHANGE policy)

    always @(posedge clk or negedge rst) begin
        if(rst==1'b0)   read_buf<={VALUE_WIDTH{1'b0}};
        else begin
            if((~RD_)&(~cache_miss)) begin
                case(compare)
                    4'd1:   read_buf<=values[0];
                    4'd2:   read_buf<=values[1];
                    4'd4:   read_buf<=values[2];
                    4'd8:   read_buf<=values[3];
                endcase
            end
        end
    end

    // Updating tag and value registers

    always @(posedge clk or negedge rst) begin
        if(rst==1'b0) begin
            for(i=0;i<NUM_LINES;i=i+1) begin
                tags[i]<={TAG_WIDTH{1'b0}};
                values[i]<={VALUE_WIDTH{1'b0}};
                dirty[i]<={NUM_LINES{1'b0}};
            end
        end
        else begin
            for(i=0;i<NUM_LINES;i=i+1) begin
                if(write_en[i]) begin
                    tags[i]<=next_tag[i];
                    values[i]<=next_value[i];
                    dirty[i]<=next_dirty[i];
                end
            end
        end
    end

    always @(posedge clk or negedge rst) begin
        if(rst==1'b0) begin
            valid<={NUM_LINES{1'b0}};
        end
        else begin
            if((~WR_)&select&cache_miss)
                valid<={valid,1'b1};
        end
    end

    // To indicate eviction and to initiate write back to lower memory hierarchy incase the entry is dirty

    assign tag_write=tags[NUM_LINES-1];
    assign value_write=values[NUM_LINES-1];
    assign memwrite=dirty[NUM_LINES-1]&(~WR_)&select&cache_miss;
endmodule

