

`define TEST_MODE
// `define RS_ALLOCATE_DEBUG
`ifndef __FREELIST_V__
`define __FREELIST_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module Freelist(
    input logic                     clock,
    input logic                     reset,

    input logic [2:0]               DispatchEN,    // Dispatch enable signals for 3 instructions
    input logic [2:0]               RetireEN,      // Retire enable signals for 3 instructions
    input logic [2:0][`PR-1:0]       RetireReg,     // Retired physical registers to be returned
    input logic                     BPRecoverEN,   // Branch recovery signal
    input logic [`ROB-1:0]           BPRecoverHead, // Not used

    output logic [2:0][`PR-1:0]      FreeReg,       // Allocated free physical registers
    output logic [`ROB-1:0]          Head,          // Free list head pointer (not used)
    output logic [2:0]               FreeRegValid,  // Free register availability for each dispatch slot
    output logic [4:0]               fl_distance    // Not used

    `ifdef TEST_MODE
        , output logic [31:0][`PR-1:0] array_display // For debug: free list contents
        , output logic [4:0]           head_display  // For debug: head pointer
        , output logic [4:0]           tail_display  // For debug: tail pointer
        , output logic                 empty_display // Not implemented yet
    `endif
);

    // Internal registers
    logic [`ROB-1:0] head, head_next;
    logic [`ROB-1:0] tail, tail_next;
    logic [`ROB-1:0] head_inc1, head_inc2, head_inc3;
    logic [`ROB-1:0] tail_inc1, tail_inc2, tail_inc3;
    logic [31:0][`PR-1:0] array, array_next;
    logic full, full_next;

    // Compute pointer increments
    assign head_inc1 = head + 1;
    assign head_inc2 = head + 2;
    assign head_inc3 = head + 3;
    assign tail_inc1 = tail + 1;
    assign tail_inc2 = tail + 2;
    assign tail_inc3 = tail + 3;

    // Compute available free register slots
    logic [5:0] available_num;
    logic [4:0] available_num_temp;
    assign available_num_temp = tail - head;
    assign available_num = full ? 32 : available_num_temp;

    // Determine FreeRegValid based on number of free slots
    always_comb begin
        if (available_num >= 3) FreeRegValid = 3'b111;
        else if (available_num == 2) FreeRegValid = 3'b110;
        else if (available_num == 1) FreeRegValid = 3'b100;
        else FreeRegValid = 3'b000;
    end

    // Compute new head pointer after dispatch
    assign head_next = head + DispatchEN[0] + DispatchEN[1] + DispatchEN[2];

    // Allocate free registers based on DispatchEN
    always_comb begin
        FreeReg = '0;
        case (DispatchEN)
            3'b000: ; // No allocation
            3'b001: FreeReg[0] = array[head];
            3'b010: FreeReg[1] = array[head];
            3'b011: begin
                FreeReg[0] = array[head_inc1];
                FreeReg[1] = array[head];
            end
            3'b100: FreeReg[2] = array[head];
            3'b101: begin
                FreeReg[2] = array[head];
                FreeReg[0] = array[head_inc1];
            end
            3'b110: begin
                FreeReg[2] = array[head];
                FreeReg[1] = array[head_inc1];
            end
            3'b111: begin
                FreeReg[2] = array[head];
                FreeReg[1] = array[head_inc1];
                FreeReg[0] = array[head_inc2];
            end
        endcase
    end

    // Compute next full flag and tail pointer after retire
    assign tail_next = RetireEN[0] + RetireEN[1] + RetireEN[2] + tail;
    assign full_next = (RetireEN != 0 || full) && (tail_next == head_next);

    // Add retired registers back to the freelist
    always_comb begin
        array_next = array;
        case (RetireEN)
            3'b000: ; // No retire
            3'b001: array_next[tail] = RetireReg[0];
            3'b010: array_next[tail] = RetireReg[1];
            3'b011: begin
                array_next[tail]     = RetireReg[1];
                array_next[tail_inc1] = RetireReg[0];
            end
            3'b100: array_next[tail] = RetireReg[2];
            3'b101: begin
                array_next[tail]     = RetireReg[2];
                array_next[tail_inc1] = RetireReg[0];
            end
            3'b110: begin
                array_next[tail]     = RetireReg[2];
                array_next[tail_inc1] = RetireReg[1];
            end
            3'b111: begin
                array_next[tail]     = RetireReg[2];
                array_next[tail_inc1] = RetireReg[1];
                array_next[tail_inc2] = RetireReg[0];
            end
        endcase
    end

    // Sequential update of free list state
    always_ff @(posedge clock) begin
        if (reset) begin
            head <= `SD 0;
            tail <= `SD 0;
            full <= `SD 1;
            for (int i = 0; i < 32; i++) begin
                array[i] <= `SD i + 32; // Initialize physical registers
            end
        end
        else if (BPRecoverEN) begin
            array <= `SD array_next;
            head <= `SD tail_next;
            tail <= `SD tail_next;
            full <= `SD 1; // Full again after recovery
        end
        else begin
            array <= `SD array_next;
            head <= `SD head_next;
            tail <= `SD tail_next;
            full <= `SD full_next;
        end
    end

    // Debug outputs
    assign array_display = array;
    assign head_display = head;
    assign tail_display = tail;

endmodule

`endif // __FREELIST_V__
