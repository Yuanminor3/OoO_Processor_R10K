
`ifndef __FREELIST_V__
`define __FREELIST_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
module Freelist(
    input logic                     clock,
    input logic                     reset,

    input logic [2:0]               DispatchEN,    // Dispatch enable signals for 3 instructions
    input logic [2:0]               RetireEN,      // Retire enable signals for 3 instructions
    input logic [2:0][`PR-1:0]      RetireReg,     // Retired physical registers to be returned
    input logic                     BPRecoverEN,   // Branch recovery signal

    output logic [2:0][`PR-1:0]      FreeReg,       // Allocated free physical registers
    output logic [`ROB-1:0]          Head,          // Free list head pointer (used in retire)
    output logic [2:0]               FreeRegValid,  // Free register availability for each dispatch slot
    output logic [`ROB-1:0]          fl_distance    // distance from tail to head, i.e. ava number

);

    // Internal registers
    logic [`ROB-1:0] head, head_next;
    logic [`ROB-1:0] tail, tail_next;
    logic [2**`ROB-1:0][`PR-1:0] array, array_next;
    logic full, full_next;
    logic [`ROB-1:0] head_offset[3];

    always_comb begin
    // Precompute head offsets
    for (int i = 0; i < 3; i++) begin
          head_offset[i] = head + i;
    end
        FreeReg = '0;
        case (DispatchEN)
            3'b000: ; // No allocation
            3'b001: FreeReg[0] = array[head];
            3'b010: FreeReg[1] = array[head];
            3'b011: begin
                FreeReg[0] = array[head_offset[1]];
                FreeReg[1] = array[head];
            end
            3'b100: FreeReg[2] = array[head];
            3'b101: begin
                FreeReg[2] = array[head];
                FreeReg[0] = array[head_offset[1]];
            end
            3'b110: begin
                FreeReg[2] = array[head];
                FreeReg[1] = array[head_offset[1]];
            end
            3'b111: begin
                FreeReg[2] = array[head];
                FreeReg[1] = array[head_offset[1]];
                FreeReg[0] = array[head_offset[2]];
            end
        endcase
    end

    // Compute available free register slots
    logic [`ROB:0] available_num;
    assign available_num = full ? 2**`ROB : (tail - head) & {`ROB{1'b1}};

// Determine FreeRegValid based on number of free slots
always_comb begin
    case (available_num)
        0: FreeRegValid = 3'b000;
        1: FreeRegValid = 3'b100;
        2: FreeRegValid = 3'b110;
        default: FreeRegValid = 3'b111; // for 3 or more
    endcase
end

    // Compute new head pointer after dispatch
    assign head_next = head + DispatchEN[0] + DispatchEN[1] + DispatchEN[2];

    // Compute next full flag and tail pointer after retire
    assign tail_next = RetireEN[0] + RetireEN[1] + RetireEN[2] + tail;
    assign full_next = (RetireEN != 0 || full) && (tail_next == head_next);

    // Add retired registers back to the freelist
    logic [`ROB-1:0] tail_offset[3];
    always_comb begin

    for (int i = 0; i < 3; i++) begin
          tail_offset[i] = tail + i;
    end

        array_next = array;
        case (RetireEN)
            3'b000: ; // No retire
            3'b001: array_next[tail] = RetireReg[0];
            3'b010: array_next[tail] = RetireReg[1];
            3'b011: begin
                array_next[tail]     = RetireReg[1];
                array_next[tail_offset[1]] = RetireReg[0];
            end
            3'b100: array_next[tail] = RetireReg[2];
            3'b101: begin
                array_next[tail]     = RetireReg[2];
                array_next[tail_offset[1]] = RetireReg[0];
            end
            3'b110: begin
                array_next[tail]     = RetireReg[2];
                array_next[tail_offset[1]] = RetireReg[1];
            end
            3'b111: begin
                array_next[tail]     = RetireReg[2];
                array_next[tail_offset[1]] = RetireReg[1];
                array_next[tail_offset[2]] = RetireReg[0];
            end
        endcase
    end

    // Sequential update of free list state
    always_ff @(posedge clock) begin
        if (reset) begin
            head <= `SD 0;
            tail <= `SD 0;
            full <= `SD 1;
            for (int i = 0; i < 2**`ROB; i++) begin
                array[i] <= `SD i + 2**`ROB; // Initialize physical registers
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

    assign fl_distance = available_num[`ROB-1:0];
    assign Head = head;
    // Debug outputs
    assign array_display = array;
    assign head_display = head;
    assign tail_display = tail; 

endmodule 

`endif // __FREELIST_V__


