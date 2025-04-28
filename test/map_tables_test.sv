`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module tb_map_table;

    // Testbench signals
    logic clock;
    logic reset;
    logic [31:0][`PR-1:0] archi_maptable;
    logic BPRecoverEN;
    logic [31:0] cdb_t_in;
    logic [2:0][`PR-1:0] maptable_new_pr;
    logic [2:0][4:0] maptable_new_ar;
    logic [2:0][4:0] reg1_ar;
    logic [2:0][4:0] reg2_ar;
    logic [2:0][`PR-1:0] reg1_tag;
    logic [2:0][`PR-1:0] reg2_tag;
    logic [2:0] reg1_ready;
    logic [2:0] reg2_ready;
    logic [2:0][`PR-1:0] Told_out;
    
    // Debug signals for TEST_MODE
    logic [31:0][`PR-1:0] map_array_disp;
    logic [31:0] ready_array_disp;
    
    // Instantiate the map_table module
    map_table uut (
        .clock(clock),
        .reset(reset),
        .archi_maptable(archi_maptable),
        .BPRecoverEN(BPRecoverEN),
        .cdb_t_in(cdb_t_in),
        .maptable_new_pr(maptable_new_pr),
        .maptable_new_ar(maptable_new_ar),
        .reg1_ar(reg1_ar),
        .reg2_ar(reg2_ar),
        .reg1_tag(reg1_tag),
        .reg2_tag(reg2_tag),
        .reg1_ready(reg1_ready),
        .reg2_ready(reg2_ready),
        .Told_out(Told_out),
        .map_array_disp(map_array_disp),
        .ready_array_disp(ready_array_disp)
    );
    
    // Clock generation
    always #5 clock = ~clock;

    // Test stimulus
    initial begin
        // Initialize signals
        clock = 0;
        reset = 0;
        BPRecoverEN = 0;
        archi_maptable = 32'h00000000;  // Example map table
        cdb_t_in = 0; // CDB packet input
        maptable_new_pr = 3'b000; // Example new physical registers
        maptable_new_ar = 3'b000; // Example new architectural registers
        reg1_ar = 3'b000; // Example reg1
        reg2_ar = 3'b001; // Example reg2

        // Apply reset
        reset = 1;
        #10;
        reset = 0;
        
        // Test 1: Normal operation with no recovery
        archi_maptable = {32{1'b0}}; // Some initial map table
        BPRecoverEN = 0;
        #10;

        // Test 2: With Branch Prediction Recovery enabled
        BPRecoverEN = 1;
        archi_maptable = {32{1'b1}}; // New map table for recovery
        #10;

        // Test 3: Update the mapping table with new physical register allocation
        maptable_new_pr = 3'b101; // New physical register allocations
        maptable_new_ar = 3'b010; // Corresponding architectural registers
        #10;

        // Test 4: Broadcast CDB packet and check ready state
        cdb_t_in = 32'h00000001; // Example CDB packet (this should trigger some updates)
        #10;

        // End of simulation
        $display("\nPASSED");
        $finish;
    end

endmodule

