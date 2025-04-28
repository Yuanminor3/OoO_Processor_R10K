`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ps.sv"

module SQ_tb;

    // Parameters
    parameter CLK_PERIOD = 10;

    // Clock and reset
    logic clock;
    logic reset;

    // Inputs
    logic [2:0] dispatch;
    logic [2:0] retire;
    logic [2:0] exe_valid;
    SQ_ENTRY_PACKET [2:0] exe_store;
    logic [2:0][`LSQ-1:0] exe_idx;
    LOAD_SQ_PACKET [1:0] load_lookup;

    // Outputs
    logic [2:0] stall;
    logic [2:0][`LSQ-1:0] tail_pos;
    logic [2**`LSQ-1:0] load_tail_ready;
    SQ_LOAD_PACKET [1:0] load_forward;
    SQ_ENTRY_PACKET [2:0] cache_wb;
    SQ_ENTRY_PACKET [2:0] sq_head;
    SQ_ENTRY_PACKET [0:2**`LSQ-1] sq_display;
    logic [`LSQ-1:0] head_dis, tail_dis;
    logic [`LSQ:0] filled_num_dis;
    SQ_ENTRY_PACKET [2**`LSQ-1:0] older_stores_display;
    logic [2**`LSQ-1:0] older_stores_valid_display;

    // Instantiate the DUT
    SQ dut (
        .clock(clock),
        .reset(reset),
        .dispatch(dispatch),
        .stall(stall),
        .tail_pos(tail_pos),
        .load_tail_ready(load_tail_ready),
        .exe_valid(exe_valid),
        .exe_store(exe_store),
        .exe_idx(exe_idx),
        .load_lookup(load_lookup),
        .load_forward(load_forward),
        .retire(retire),
        .cache_wb(cache_wb),
        .sq_head(sq_head),
        .sq_display(sq_display),
        .head_dis(head_dis),
        .tail_dis(tail_dis),
        .filled_num_dis(filled_num_dis),
        .older_stores_display(older_stores_display),
        .older_stores_valid_display(older_stores_valid_display)
    );

    // Clock generation
    initial begin
        clock = 0;
        forever #(CLK_PERIOD/2) clock = ~clock;
    end

    // Test sequence
    initial begin
        // Initialize
        reset = 1;
        dispatch = 0;
        retire = 0;
        exe_valid = 0;
        exe_store = '{default: '0};
        exe_idx = '{default: '0};
        load_lookup = '{default: '0};
        #(2*CLK_PERIOD);

        reset = 0;
        #(CLK_PERIOD);

        // Dispatch 2 entries
        dispatch = 3'b010; // dispatch 1 entry
        #(CLK_PERIOD);
        dispatch = 3'b001; // dispatch 1 more
        #(CLK_PERIOD);
        dispatch = 3'b000;
        #(CLK_PERIOD);

        // Simulate EXE writing to the entries
        exe_valid = 3'b001;
        exe_idx[0] = tail_pos[0];
        exe_store[0].ready = 1;
        exe_store[0].addr = 32'h12345678;
        exe_store[0].data = 32'hDEADBEEF;
        exe_store[0].usebytes = 4'b1111;
        #(CLK_PERIOD);

        exe_valid = 0;
        #(CLK_PERIOD);

        // Check internal states
        if (filled_num_dis !== 2) begin
            $display("\nFAILED: filled_num should be 2, got %0d", filled_num_dis);
            $finish;
        end

        // Now retire 1 entry
        retire = 3'b001;
        #(CLK_PERIOD);

        retire = 0;
        #(CLK_PERIOD);

        // Check filled_num again
        if (filled_num_dis !== 1) begin
            $display("\nFAILED: filled_num should be 1 after retire, got %0d", filled_num_dis);
            $finish;
        end

        // Final check passed
        $display("\nPASSED");
        $finish;
    end

endmodule

