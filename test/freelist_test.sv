`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module Freelist_tb;

    // Clock and reset
    reg clock;
    reg reset;
    
    // Inputs
    reg [2:0] DispatchEN;
    reg [2:0] RetireEN;
    reg [2:0][`PR-1:0] RetireReg;
    reg BPRecoverEN;
    
    // Outputs
    wire [2:0][`PR-1:0] FreeReg;
    wire [2:0] FreeRegValid;
    
    // Instantiate DUT
    Freelist dut (
        .clock(clock),
        .reset(reset),
        .DispatchEN(DispatchEN),
        .RetireEN(RetireEN),
        .RetireReg(RetireReg),
        .BPRecoverEN(BPRecoverEN),
        .FreeReg(FreeReg),
        .FreeRegValid(FreeRegValid)
    );
    
    // Clock generation
    always #5 clock = ~clock;
    
    // Test sequence
    initial begin
        // Initialize
        clock = 0;
        reset = 1;
        DispatchEN = 0;
        RetireEN = 0;
        RetireReg = 0;
        BPRecoverEN = 0;
        #10;
        reset = 0;
        #10;
        
        // Test 1: Check initial state
        if (FreeRegValid != 3'b111) begin
            $display("\nFAILED: Initial state incorrect");
            $display("FreeRegValid = %h,", FreeRegValid);
            $finish;
        end
        
        // Test 2: Dispatch registers
        DispatchEN = 3'b001;
        #10;
        if (FreeRegValid != 7) begin
            $display("\nFAILED: Dispatch failed");
            $display("FreeRegValid = %h,", FreeRegValid);
            $finish;
        end 
        
        // Test 3: Retire register
        DispatchEN = 0;
        RetireEN = 3'b001;
        RetireReg[0] = 5'd1;
        #10;
        if (FreeRegValid != 7) begin
            $display("\nFAILED: Retire failed");
            $display("FreeRegValid = %h,", FreeRegValid);
            $finish;
        end
        
        // Test 4: Branch recovery
        BPRecoverEN = 1;
        #10;
        if (FreeRegValid != 3'b111) begin
            $display("\nFAILED: BP recovery failed");
            $display("FreeRegValid = %h,", FreeRegValid);
            $finish;
        end
        
        $display("\nPASSED");
        $finish;
    end
    
endmodule
