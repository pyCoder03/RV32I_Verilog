`timescale 1ns / 1ps
`define NUM_LINES 4
`define VALUE_WIDTH 32
`define TAG_WIDTH $clog2(NUM_LINES)     // This amount of LSBs of address must be used to encode cache lines, and stored with values

// Implementation of Least Recently Used (LRU) replacement policy
// Fully Associative Cache with 4 lines
module LRU_Cache(           // 
    input clk,
    input rst,
    input [TAG_WIDTH-1:0] input_tag,
    input [VALUE_WIDTH-1:0] new_value,
    input RD_,              // Read strobe              
    input WR_,              // Write strobe
    input invalidate,       // When memory location is written, old value must be deleted
    output cache_miss
    );

    // WORKING
    
    // New write: Add to top and right-shift rest all
    // Overwrite: Bring the key value pair to top and alter the value, right shift till old position (inclusive)
    // Read: If hit occurs, bring to top and right shift till old position (inclusive)
    //      If cache miss occurs, it's the driving system (IF or MEM stage)'s job to store the item in cache
    
    reg[TAG_WIDTH-1:0] tags[NUM_LINES-1:0];
    reg[VALUE_WIDTH-1:0] values[NUM_LINES-1:0];
    reg[VALUE_WIDTH-1:0] read_buf;
    reg[NUM_LINES-1:0] valid;
    reg[NUM_LINES-1:0] dirty;

    wire[NUM_PAIRS-1:0] right_shift_en_hit;     // When
    wire[NUM_PAIRS-1:0] left_shift_en;      // To delete one entry
    wire[NUM_PAIRS-1:0] write_en;           // Clock gates
    
    reg[TAG_WIDTH-1:0] next_tag[NUM_LINES-1:0];
    reg[VALUE_WIDTH-1:0] next_value[NUM_LINES-1:0];
    
    wire[NUM_PAIRS-1:0] compare;
    
    integer i;
    
    // Comparing the input search tag with each and every valid tag stored in the cache
    
    generate
    genvar g;
        for(g=0;g<NUM_LINES;g=g+1) begin
            assign compare[g]=(input_tag==tags[g])&valid[g];    
        end
    endgenerate
    
    //
    
    assign right_shift_en_hit[NUM_LINES-1]=compare[NUM_LINES-1];
    
    generate
        for(g=0;g<NUM_LINES-1;g=g+1) begin
            assign right_shift_en_hit[g]=compare[g]|right_shift_en[g+1];
        end
    endgenerate

    assign write_en=right_shift_en_hit|(~right_shift_en_hit[0]);

    //

    assign left_shift_en[0]=compare[0];
    
    for(g=1;g<NUM_PAIRS;g=g+1) begin
        assign left_shift_en[g]=compare[g]|left_shift_en[g-1];
    end
    
    assign right_shift_en=~left_shift_en;
    
    //
    
    always @(*) begin
        next_value[0]=case({WR_,compare})
            5'd17:  values[0];
            5'd18:	values[1];
            5'd20:	values[2];
            5'd24:	values[3];
            default:    new_value;
        endcase
    end

    always @(*) begin   
        next_tag[0]=input_tag;
    end

    always @(*) begin
        for(i=1;i<NUM_LINES;i=i+1) begin
            next_value[i]=values[i-1];
            next_tag[i]=tags[i-1];
        end
    end

    // Read buffer update (NO_CHANGE)

    always @(negedge RD_ or negedge rst) begin
        if(rst==1'b0)   read_buf<={VALUE_WIDTH{1'b0}};
        else begin
            read_buf<=case(compare)
                4'd1:   values[0];
                4'd2:   values[1];
                4'd4:   values[2];
                4'd8:   values[3];
            endcase
        end
    end

    
    always @(posedge clk or negedge rst) begin
        if(rst==1'b0) begin
            for(i=0;i<NUM_LINES;i=i+1) begin
                tags[i]<={TAG_WIDTH{1'b0}};
                values[i]<={VALUE_WIDTH{1'b0}};
                valid[i]<={NUM_LINES{1'b0}};
                dirty[i]<={NUM_LINES{1'b0}};
            end
        end
        else begin
            for(i=0;i<NUM_LINES;i=i+1) begin
                if(write_en[i]) begin
                    tags[i]<=next_tag[i];
                    values[i]<=next_value[i];
                end
            end
        end
    end
endmodule

