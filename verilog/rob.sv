
`ifndef __ROB_V__
`define __ROB_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module ROB(
    input  clk, 
    input  rst,
    // from lsq_disp_mask          
    input  ROB_ENTRY_PACKET[2:0] dispatch_rob_pkts,  
    // from complete
    input  [2:0] cs_retire_valid,          // Valid flags for completed entries
    input  [2:0][`SYS_ROB_ADDR_WIDTH-1:0] cs_retire_idx, // SYS_ROB_ADDR_WIDTH indices of completed entries
    input  [2:0] rb_recover_valid,     // Valid flags for branch misprediction recovery
    input  [2:0][`SYS_XLEN-1:0] cs_retire_pc,    // Correct target PCs for mispredictions
    // from lsq_retire_mask
    input  fch_rec_enable,                   // Branch prediction recovery enable
    // from lsq
    input  [2:0] rb_sq_stall,                // Store queue lsq_stall_mask signals
    // to lsq_disp_mask
    output logic [2:0][`SYS_ROB_ADDR_WIDTH-1:0] dispatch_index, // Allocated SYS_ROB_ADDR_WIDTH indices for new entries
    output logic [`SYS_ROB_ADDR_WIDTH:0] rb_free_slots,
    // to lsq_retire_mask
    output ROB_ENTRY_PACKET[2:0] retire_entry   // Retired entries (up to 3 per cycle)
);

// SYS_ROB_ADDR_WIDTH storage fl_pr_stack
ROB_ENTRY_PACKET [`SYS_ROB_ENTRIES-1:0] rb_entry;
ROB_ENTRY_PACKET [`SYS_ROB_ENTRIES-1:0] rb_entry_nxt;

// fl_head_ptr, Tail, input start pointers
logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_head_reg, fl_tail_reg;
logic [`SYS_ROB_ADDR_WIDTH-1:0] rb_in_start, rb_in_start1, rb_in_start2, rb_in_start3;

// fl_head_ptr, Tail, input start update pointers
logic [2:0] rb_in_count;
logic [2:0] rb_head_step_temp, rb_head_step;
logic [2:0] rb_tail_step;
logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_head_nxt, fl_tail_nxt;
logic [`SYS_ROB_ADDR_WIDTH-1:0] rb_head2tail_dist;
logic empty, rb_empty, rb_empty_nxt;
assign rb_in_start1 = rb_in_start + 1;
assign rb_in_start2 = rb_in_start + 2;
assign rb_in_start3 = rb_in_start + 3;

// Dispatch count based on valid inputs
// fl_tail_reg increment update logic
assign rb_tail_step = (dispatch_rob_pkts[0].valid & dispatch_rob_pkts[1].valid & dispatch_rob_pkts[2].valid) ? 3 :
                    (dispatch_rob_pkts[1].valid & dispatch_rob_pkts[2].valid) ? 2 :
                    (dispatch_rob_pkts[2].valid) ? 1 : 0;

// Compute free space left in SYS_ROB_ADDR_WIDTH
// output structural lsq_stall_mask logic
assign rb_head2tail_dist = fl_tail_reg - fl_head_reg;
assign rb_free_slots = (empty) ? 32 : 31 - rb_head2tail_dist + rb_head_step;


/* -------------------- fl_head_ptr Pointer Management -------------------- */
// Calculate how many completed instructions at the fl_head_reg can be retired

logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_head_offsets[3];
always_comb begin
    // Precompute fl_head_reg offsets
    for (int i = 0; i < 3; i++) begin
          fl_head_offsets[i] = fl_head_reg + i;
    end
end

always_comb begin
    rb_head_step_temp = 0;
    if (rb_entry[fl_head_reg].completed) begin
        rb_head_step_temp = 1;
        if (rb_entry[fl_head_offsets[1]].completed) begin
            rb_head_step_temp = 2;
            if (rb_entry[fl_head_offsets[2]].completed) begin
                rb_head_step_temp = 3;
            end
        end
    end
end

// fl_head_reg increment update logic (rob internal lsq_retire_mask logic), adjusted based on store queue stalls
logic [2:0] dec_is_store_flag; 

always_comb begin

    for (int i = 0; i < 3; i++) begin
        dec_is_store_flag[i] = (i == 0) ? rb_entry[fl_head_reg].dec_is_store_flag
                                     : rb_entry[fl_head_offsets[i]].dec_is_store_flag;
    end

	rb_head_step = rb_head_step_temp;  // set by default, if no rb_sq_stall && store insn, no change
	priority case (rb_sq_stall)
		3'b111: begin
			if (dec_is_store_flag[0])       rb_head_step = 0;
			else if (dec_is_store_flag[1])  rb_head_step = (rb_head_step_temp >= 1) ? 1 : rb_head_step_temp;
			else if (dec_is_store_flag[2])  rb_head_step = (rb_head_step_temp >= 2) ? 2 : rb_head_step_temp;
		end
		3'b011: begin
			if (dec_is_store_flag[0]) begin
				if (dec_is_store_flag[1])  rb_head_step = (rb_head_step_temp >= 1) ? 1 : rb_head_step_temp;
				else if (dec_is_store_flag[2])  rb_head_step = (rb_head_step_temp >= 2) ? 2 : rb_head_step_temp;
			end
			else if (dec_is_store_flag[1]) begin
				if (dec_is_store_flag[2])  rb_head_step = (rb_head_step_temp >= 2) ? 2 : rb_head_step_temp;
			end
		end
		3'b001: begin
			if (dec_is_store_flag[0] && dec_is_store_flag[1] && dec_is_store_flag[2])  rb_head_step = (rb_head_step_temp >= 2) ? 2 : rb_head_step_temp;
		end

		default: begin
			rb_head_step = rb_head_step_temp;
		end
	endcase
end
// Update fl_head_reg pointer and empty flag
// fl_head_reg pointer and empty flag update logic, rb_empty = 1 indicates that fl_head_reg temporarily reaching fl_tail_reg and so that rob is empty right now 
always_comb begin
    rb_empty = empty;
    priority case (rb_head_step)
        3: begin  // Retire 3 entries
            fl_head_nxt = (fl_head_offsets[2] == fl_tail_reg) ? fl_head_reg + 2 : fl_head_reg + 3;
            rb_empty = (fl_head_offsets[2] == fl_tail_reg) ? 1 : 0;
        end
        2: begin  // Retire 2 entries
            fl_head_nxt = (fl_head_offsets[1] == fl_tail_reg) ? fl_head_reg + 1 : fl_head_reg + 2;
            rb_empty = (fl_head_offsets[1] == fl_tail_reg) ? 1 : 0;
        end
        1: begin  // Retire 1 entry
            fl_head_nxt = (fl_head_reg == fl_tail_reg) ? fl_head_reg : fl_head_reg + 1;
            rb_empty = (fl_head_reg == fl_tail_reg) ? 1 : 0;
        end
        default: begin
            fl_head_nxt = fl_head_reg;
            rb_empty = empty;
        end
    endcase
end

/* -------------------- Tail Pointer Management -------------------- */
// Calculate next fl_tail_reg position and how many new entries inserted
	logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_tail_offsets[3];
	assign fl_tail_offsets[0] = fl_tail_reg + 1;
	assign fl_tail_offsets[1] = fl_tail_reg + 2;
	assign fl_tail_offsets[2] = fl_tail_reg + 3;
	
always_comb begin
	fl_tail_nxt = fl_tail_reg + rb_tail_step;
	rb_in_start = fl_tail_reg + 1;
	rb_empty_nxt = 0;
	rb_in_count = 0;
	// edge case: SYS_ROB_ADDR_WIDTH is empty
	if (fl_tail_reg == fl_head_nxt && rb_empty) begin
		rb_in_start = fl_tail_reg;
		fl_tail_nxt = (rb_tail_step > 0) ? fl_tail_reg + rb_tail_step - 1 : fl_tail_reg;
		rb_in_count = (rb_tail_step == 3) ? 3'b111 :
					(rb_tail_step == 2) ? 3'b110 :
					(rb_tail_step == 1) ? 3'b100 : 3'b000;
		rb_empty_nxt = (rb_tail_step > 0) ? 0 : 1;
	end
	else begin
		priority case (rb_tail_step)
			3: begin
				if (fl_tail_offsets[0] == fl_head_nxt) begin
					fl_tail_nxt = fl_tail_reg;
					rb_in_count = 3'b000;
				end
				else if (fl_tail_offsets[1] == fl_head_nxt) begin
					fl_tail_nxt = fl_tail_reg + 1;
					rb_in_count = 3'b100;
				end
				else if (fl_tail_offsets[2] == fl_head_nxt) begin
					fl_tail_nxt = fl_tail_reg + 2;
					rb_in_count = 3'b110;
				end
				else begin
					fl_tail_nxt = fl_tail_reg + 3;
					rb_in_count = 3'b111;
				end
			end
			2: begin
				if (fl_tail_offsets[0] == fl_head_nxt) begin
					fl_tail_nxt = fl_tail_reg;
					rb_in_count = 3'b000;
				end
				else if (fl_tail_offsets[1] == fl_head_nxt) begin
					fl_tail_nxt = fl_tail_reg + 1;
					rb_in_count = 3'b100;
				end
				else begin
					fl_tail_nxt = fl_tail_reg + 2;
					rb_in_count = 3'b110;
				end
			end
			1: begin
				if (fl_tail_offsets[0] == fl_head_nxt) begin
					fl_tail_nxt = fl_tail_reg;
					rb_in_count = 3'b000;
				end
				else begin
					fl_tail_nxt = fl_tail_reg + 1;
					rb_in_count = 3'b100;
				end
			end
			default begin
				rb_in_count = 3'b000;
				fl_tail_nxt = fl_tail_reg;
			end
		endcase
	end
end

/* -------------------- SYS_ROB_ADDR_WIDTH Update and Dispatch/Retire -------------------- */
always_comb begin
	rb_entry_nxt = rb_entry;
	retire_entry = 0;
	dispatch_index = 0;
	priority case (rb_head_step)
		3: begin
			retire_entry[2] = rb_entry[fl_head_reg];
			retire_entry[1] = rb_entry[fl_head_offsets[1]];
			retire_entry[0] = rb_entry[fl_head_offsets[2]];
			rb_entry_nxt[fl_head_reg] = 0;
			rb_entry_nxt[fl_head_offsets[1]] = 0;
			rb_entry_nxt[fl_head_offsets[2]] = 0;		
		end
		2: begin
			retire_entry[2] = rb_entry[fl_head_reg];
			retire_entry[1] = rb_entry[fl_head_offsets[1]];
			rb_entry_nxt[fl_head_reg] = 0;
			rb_entry_nxt[fl_head_offsets[1]] = 0;
		end
		1: begin
			retire_entry[2] = rb_entry[fl_head_reg];
			rb_entry_nxt[fl_head_reg] = 0;
		end
		default: begin
			retire_entry = 0;
		end
	endcase
	priority case (rb_in_count)
		3'b111: begin
			rb_entry_nxt[rb_in_start] = dispatch_rob_pkts[2];
			dispatch_index[2] = rb_in_start;
			rb_entry_nxt[rb_in_start1] = dispatch_rob_pkts[1];
			dispatch_index[1] = rb_in_start1;	
			rb_entry_nxt[rb_in_start2] = dispatch_rob_pkts[0];
			dispatch_index[0] = rb_in_start2;	
		end
		3'b110: begin
			rb_entry_nxt[rb_in_start] = dispatch_rob_pkts[2];
			dispatch_index[2] = rb_in_start;
			rb_entry_nxt[rb_in_start1] = dispatch_rob_pkts[1];	
			dispatch_index[1] = rb_in_start1;	
		end
		3'b100: begin
			rb_entry_nxt[rb_in_start] = dispatch_rob_pkts[2];
			dispatch_index[2] = rb_in_start;	
		end
		default: begin
			dispatch_index = 0;
		end
	endcase
        // Handle instruction completion and branch misprediction
	for (int i = 0; i < 3; i++) begin
		if (cs_retire_valid[i]) begin // insn completed signal from complete stage
			rb_entry_nxt[cs_retire_idx[i]].completed = 1;
			rb_entry_nxt[cs_retire_idx[i]].precise_state_need = 0;
		        // branch prediction recovery logic

			// rb_recover_valid from complete stage, defined as if fu take branch
			// bp_pred_taken, source logic in pp and bp, defined as if branch is predicted to be taken
                        if (rb_recover_valid[i] == 0 && rb_entry[cs_retire_idx[i]].bp_pred_taken == 1) begin //fu doesn't take, predict take
				rb_entry_nxt[cs_retire_idx[i]].precise_state_need = 1;  // bp predict wrong, precise state needs to recover
				rb_entry_nxt[cs_retire_idx[i]].cs_retire_pc = rb_entry[cs_retire_idx[i]].NPC; // recover to pc before branch taken
			end
			else if (rb_recover_valid[i] == 1 && rb_entry[cs_retire_idx[i]].bp_pred_taken == 0) begin // fu take, predict not
				rb_entry_nxt[cs_retire_idx[i]].precise_state_need = 1; // bp predict wrong, precise state needs to recover
				// cs_retire_pc from complete stage, defined as the taget pc of fu_branch
				rb_entry_nxt[cs_retire_idx[i]].cs_retire_pc = cs_retire_pc[i]; // recover to actual branch address
			end
			else if (rb_recover_valid[i] == 1 && rb_entry[cs_retire_idx[i]].bp_pred_taken == 1 && cs_retire_pc[i] != rb_entry[cs_retire_idx[i]].bp_pred_target) begin  // both fu and predict take branch, but bp preditc pc wrong, need to recover to actual branch address
				rb_entry_nxt[cs_retire_idx[i]].precise_state_need = 1;
				rb_entry_nxt[cs_retire_idx[i]].cs_retire_pc = cs_retire_pc[i];
			end
			else begin // predict result the same as actual bp fu, no need to recover
				rb_entry_nxt[cs_retire_idx[i]].precise_state_need = 0;
				rb_entry_nxt[cs_retire_idx[i]].cs_retire_pc = 0;
			end
		end
	end
end

/* -------------------- SYS_ROB_ADDR_WIDTH Register Update -------------------- */
// Sequential logic: Update pointers and entries on clk edge
always_ff @(posedge clk) begin
    if (rst) begin
        fl_head_reg <= `SYS_SMALL_DELAY 0;
        fl_tail_reg <= `SYS_SMALL_DELAY 0;
        empty <= `SYS_SMALL_DELAY 1;
        rb_entry <= `SYS_SMALL_DELAY 0;
    end
    else if (fch_rec_enable) begin
        fl_head_reg <= `SYS_SMALL_DELAY 0;
        fl_tail_reg <= `SYS_SMALL_DELAY 0;
        empty <= `SYS_SMALL_DELAY 1;

        // Clear only entries between fl_tail_reg and fl_head_reg
        for (int i = 0; i < `SYS_ROB_ENTRIES; i++) begin
            if (fl_tail_reg >= fl_head_reg) begin // Normal case
                if (i >= fl_head_reg && i < fl_tail_reg) rb_entry[i] <= `SYS_SMALL_DELAY 0;
            end
            else if (i >= fl_head_reg || i < fl_tail_reg) begin // Wrap-around case
                    rb_entry[i] <= `SYS_SMALL_DELAY 0;
            end
        end
    end
    else begin
        rb_entry <= `SYS_SMALL_DELAY rb_entry_nxt;
        fl_head_reg <= `SYS_SMALL_DELAY fl_head_nxt;
        fl_tail_reg <= `SYS_SMALL_DELAY fl_tail_nxt;
        empty <= `SYS_SMALL_DELAY rb_empty_nxt;
    end
end 

endmodule
`endif

