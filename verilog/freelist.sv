
`ifndef __FREELIST_V__
`define __FREELIST_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
module Freelist(
    input logic                     clk,
    input logic                     rst,

    input logic [2:0]               fl_dispatch_en_mask,    // Dispatch enable signals for 3 instructions
    input logic [2:0]               fl_retire_en_mask,      // Retire enable signals for 3 instructions
    input logic [2:0][`SYS_PHYS_REG-1:0]      fl_retired_pr_list,     // Retired physical registers to be returned
    input logic                     fch_rec_enable,   // Branch recovery signal

    output logic [2:0][`SYS_PHYS_REG-1:0]      fl_allocated_pr_list,       // Allocated free physical registers
    output logic [`SYS_ROB_ADDR_WIDTH-1:0]          fl_head_ptr,          // Free list fl_head_reg pointer (used in lsq_retire_mask)
    output logic [2:0]               fl_alloc_valid_mask,  // Free register availability for each lsq_disp_mask slot
    output logic [`SYS_ROB_ADDR_WIDTH-1:0]          fl_free_count    // distance from fl_tail_reg to fl_head_reg, i.e. ava number

);

    // Internal registers
    logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_head_reg, fl_head_nxt;
    logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_tail_reg, fl_tail_nxt;
    logic [2**`SYS_ROB_ADDR_WIDTH-1:0][`SYS_PHYS_REG-1:0] fl_pr_stack, fl_pr_stack_nxt;
    logic fl_buffer_full, fl_buffer_full_nxt;
    logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_head_offsets[3];

    always_comb begin
    // Precompute fl_head_reg offsets
    for (int i = 0; i < 3; i++) begin
          fl_head_offsets[i] = fl_head_reg + i;
    end
        fl_allocated_pr_list = '0;
        case (fl_dispatch_en_mask)
            3'b000: ; // No allocation
            3'b001: fl_allocated_pr_list[0] = fl_pr_stack[fl_head_reg];
            3'b010: fl_allocated_pr_list[1] = fl_pr_stack[fl_head_reg];
            3'b011: begin
                fl_allocated_pr_list[0] = fl_pr_stack[fl_head_offsets[1]];
                fl_allocated_pr_list[1] = fl_pr_stack[fl_head_reg];
            end
            3'b100: fl_allocated_pr_list[2] = fl_pr_stack[fl_head_reg];
            3'b101: begin
                fl_allocated_pr_list[2] = fl_pr_stack[fl_head_reg];
                fl_allocated_pr_list[0] = fl_pr_stack[fl_head_offsets[1]];
            end
            3'b110: begin
                fl_allocated_pr_list[2] = fl_pr_stack[fl_head_reg];
                fl_allocated_pr_list[1] = fl_pr_stack[fl_head_offsets[1]];
            end
            3'b111: begin
                fl_allocated_pr_list[2] = fl_pr_stack[fl_head_reg];
                fl_allocated_pr_list[1] = fl_pr_stack[fl_head_offsets[1]];
                fl_allocated_pr_list[0] = fl_pr_stack[fl_head_offsets[2]];
            end
        endcase
    end

    // Compute available free register slots
    logic [`SYS_ROB_ADDR_WIDTH:0] fl_avai_cnt_wide;
    assign fl_avai_cnt_wide = fl_buffer_full ? 2**`SYS_ROB_ADDR_WIDTH : (fl_tail_reg - fl_head_reg) & {`SYS_ROB_ADDR_WIDTH{1'b1}};

// Determine fl_alloc_valid_mask based on number of free slots
always_comb begin
    case (fl_avai_cnt_wide)
        0: fl_alloc_valid_mask = 3'b000;
        1: fl_alloc_valid_mask = 3'b100;
        2: fl_alloc_valid_mask = 3'b110;
        default: fl_alloc_valid_mask = 3'b111; // for 3 or more
    endcase
end

    // Compute new fl_head_reg pointer after lsq_disp_mask
    assign fl_head_nxt = fl_head_reg + fl_dispatch_en_mask[0] + fl_dispatch_en_mask[1] + fl_dispatch_en_mask[2];

    // Compute next fl_buffer_full flag and fl_tail_reg pointer after lsq_retire_mask
    assign fl_tail_nxt = fl_retire_en_mask[0] + fl_retire_en_mask[1] + fl_retire_en_mask[2] + fl_tail_reg;
    assign fl_buffer_full_nxt = (fl_retire_en_mask != 0 || fl_buffer_full) && (fl_tail_nxt == fl_head_nxt);

    // Add retired registers back to the freelist
    logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_tail_offsets[3];
    always_comb begin

    for (int i = 0; i < 3; i++) begin
          fl_tail_offsets[i] = fl_tail_reg + i;
    end

        fl_pr_stack_nxt = fl_pr_stack;
        case (fl_retire_en_mask)
            3'b000: ; // No lsq_retire_mask
            3'b001: fl_pr_stack_nxt[fl_tail_reg] = fl_retired_pr_list[0];
            3'b010: fl_pr_stack_nxt[fl_tail_reg] = fl_retired_pr_list[1];
            3'b011: begin
                fl_pr_stack_nxt[fl_tail_reg]     = fl_retired_pr_list[1];
                fl_pr_stack_nxt[fl_tail_offsets[1]] = fl_retired_pr_list[0];
            end
            3'b100: fl_pr_stack_nxt[fl_tail_reg] = fl_retired_pr_list[2];
            3'b101: begin
                fl_pr_stack_nxt[fl_tail_reg]     = fl_retired_pr_list[2];
                fl_pr_stack_nxt[fl_tail_offsets[1]] = fl_retired_pr_list[0];
            end
            3'b110: begin
                fl_pr_stack_nxt[fl_tail_reg]     = fl_retired_pr_list[2];
                fl_pr_stack_nxt[fl_tail_offsets[1]] = fl_retired_pr_list[1];
            end
            3'b111: begin
                fl_pr_stack_nxt[fl_tail_reg]     = fl_retired_pr_list[2];
                fl_pr_stack_nxt[fl_tail_offsets[1]] = fl_retired_pr_list[1];
                fl_pr_stack_nxt[fl_tail_offsets[2]] = fl_retired_pr_list[0];
            end
        endcase
    end

    // Sequential update of free list state
    always_ff @(posedge clk) begin
        if (rst) begin
            fl_head_reg <= `SYS_SMALL_DELAY 0;
            fl_tail_reg <= `SYS_SMALL_DELAY 0;
            fl_buffer_full <= `SYS_SMALL_DELAY 1;
            for (int i = 0; i < 2**`SYS_ROB_ADDR_WIDTH; i++) begin
                fl_pr_stack[i] <= `SYS_SMALL_DELAY i + 2**`SYS_ROB_ADDR_WIDTH; // Initialize physical registers
            end
        end
        else if (fch_rec_enable) begin
            fl_pr_stack <= `SYS_SMALL_DELAY fl_pr_stack_nxt;
            fl_head_reg <= `SYS_SMALL_DELAY fl_tail_nxt;
            fl_tail_reg <= `SYS_SMALL_DELAY fl_tail_nxt;
            fl_buffer_full <= `SYS_SMALL_DELAY 1; // Full again after recovery
        end
        else begin
            fl_pr_stack <= `SYS_SMALL_DELAY fl_pr_stack_nxt;
            fl_head_reg <= `SYS_SMALL_DELAY fl_head_nxt;
            fl_tail_reg <= `SYS_SMALL_DELAY fl_tail_nxt;
            fl_buffer_full <= `SYS_SMALL_DELAY fl_buffer_full_nxt;
        end
    end

    assign fl_free_count = fl_avai_cnt_wide[`SYS_ROB_ADDR_WIDTH-1:0];
    assign fl_head_ptr = fl_head_reg;
    // Debug outputs
    assign fl_disp_pr_stack = fl_pr_stack;
    assign head_display = fl_head_reg;
    assign fl_disp_tail_ptr = fl_tail_reg; 

endmodule 

`endif // __FREELIST_V__


