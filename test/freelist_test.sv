`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module Freelist_tb;

    // Clock and rst
    reg clk;
    reg rst;
    
    // Inputs
    reg [2:0] fl_dispatch_en_mask;
    reg [2:0] fl_retire_en_mask;
    reg [2:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] fl_retired_pr_list;
    reg fch_rec_enable;
    
    // Outputs
    wire [2:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] fl_allocated_pr_list;
    wire [2:0] fl_alloc_valid_mask;
    
    // Instantiate DUT
    Freelist dut (
        .clk(clk),
        .rst(rst),
        .fl_dispatch_en_mask(fl_dispatch_en_mask),
        .fl_retire_en_mask(fl_retire_en_mask),
        .fl_retired_pr_list(fl_retired_pr_list),
        .fch_rec_enable(fch_rec_enable),
        .fl_allocated_pr_list(fl_allocated_pr_list),
        .fl_alloc_valid_mask(fl_alloc_valid_mask)
    );
    
    // Clock generation
    always #5 clk = ~clk;
    
    // Test sequence
    initial begin
        // Initialize
        clk = 0;
        rst = 1;
        fl_dispatch_en_mask = 0;
        fl_retire_en_mask = 0;
        fl_retired_pr_list = 0;
        fch_rec_enable = 0;
        #10;
        rst = 0;
        #10;
        
        // Test 1: Check initial state
        if (fl_alloc_valid_mask != 3'b111) begin
            $display("\nFAILED: Initial state incorrect");
            $display("fl_alloc_valid_mask = %h,", fl_alloc_valid_mask);
            $finish;
        end
        
        // Test 2: Dispatch registers
        fl_dispatch_en_mask = 3'b001;
        #10;
        if (fl_alloc_valid_mask != 7) begin
            $display("\nFAILED: Dispatch failed");
            $display("fl_alloc_valid_mask = %h,", fl_alloc_valid_mask);
            $finish;
        end 
        
        // Test 3: Retire register
        fl_dispatch_en_mask = 0;
        fl_retire_en_mask = 3'b001;
        fl_retired_pr_list[0] = 5'd1;
        #10;
        if (fl_alloc_valid_mask != 7) begin
            $display("\nFAILED: Retire failed");
            $display("fl_alloc_valid_mask = %h,", fl_alloc_valid_mask);
            $finish;
        end
        
        // Test 4: Branch recovery
        fch_rec_enable = 1;
        #10;
        if (fl_alloc_valid_mask != 3'b111) begin
            $display("\nFAILED: SYS_BRANCH_PREDICTION recovery failed");
            $display("fl_alloc_valid_mask = %h,", fl_alloc_valid_mask);
            $finish;
        end
        
        $display("\nPASSED");
        $finish;
    end
    
endmodule
