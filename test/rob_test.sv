`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module tb_ROB;

// Declare testbench signals
reg clock;
reg reset;
reg [2:0] rob_in;
reg [2:0] complete_valid;
reg [2:0][`ROB-1:0] complete_entry;
reg [2:0] precise_state_valid;
reg [2:0][`XLEN-1:0] target_pc;
reg BPRecoverEN;
reg [2:0] sq_stall;
wire [2:0][`ROB-1:0] dispatch_index;
wire [2:0] struct_stall;
wire [2:0] retire_entry;

// Instantiate the ROB module
ROB uut (
    .clock(clock), 
    .reset(reset),
    .rob_in(rob_in),
    .complete_valid(complete_valid),
    .complete_entry(complete_entry),
    .precise_state_valid(precise_state_valid),
    .target_pc(target_pc),
    .BPRecoverEN(BPRecoverEN),
    .sq_stall(sq_stall),
    .dispatch_index(dispatch_index),
    .struct_stall(struct_stall),
    .retire_entry(retire_entry)
);  


// Clock generation
always begin
    #5 clock = ~clock;
end

// Test sequence
initial begin
    // Initialize signals
    clock = 0;
    reset = 0;
    rob_in = 3'b000;
    complete_valid = 3'b000;
    complete_entry = 3'b000;
    precise_state_valid = 3'b000;
    target_pc = 3'b000;
    BPRecoverEN = 0;
    sq_stall = 3'b000;

    // Apply reset
    reset = 1;
    #10;
    reset = 0;
    
    // Test 1: Dispatch entries into the ROB
    rob_in = 3'b111; // Simulate valid ROB entries
    #10;

    // Test 2: Complete an instruction
    complete_valid = 3'b001; // One entry completed
    complete_entry = 3'b001; // The first entry completed
    precise_state_valid = 3'b000;
    #10;

    // Test 3: Branch misprediction recovery
    precise_state_valid = 3'b001; // Simulate a branch misprediction recovery
    target_pc = 3'b101; // Recovery target PC
    #10;

    // Test 4: Store Queue stall (simulate store issue)
    sq_stall = 3'b111; // All store queues stalled
    #10;

    // Test 5: Reset the ROB and check if it clears everything
    reset = 1;
    #10;
    reset = 0;

    // Check if all tests passed
    if (rob_in == 3'b111 && complete_valid == 3'b001 && complete_entry == 3'b001 && 
        precise_state_valid == 3'b001 && target_pc == 3'b101 && sq_stall == 3'b111) 
    begin
        $display("\nPASSED");
    end else begin
        $display("Test failed");
    end

    // End simulation
    $finish;
end


endmodule

