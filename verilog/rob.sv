
`define TEST_MODE
// `define RS_ALLOCATE_DEBUG
`ifndef __ROB_V__
`define __ROB_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module ROB(
    input  clock, 
    input  reset,
    // from dispatch          
    input  ROB_ENTRY_PACKET[2:0] rob_in,  
    // from complete
    input  [2:0] complete_valid,          // Valid flags for completed entries
    input  [2:0][`ROB-1:0] complete_entry, // ROB indices of completed entries
    input  [2:0] precise_state_valid,     // Valid flags for branch misprediction recovery
    input  [2:0][`XLEN-1:0] target_pc,    // Correct target PCs for mispredictions
    // from retire
    input  BPRecoverEN,                   // Branch prediction recovery enable
    // from lsq
    input  [2:0] sq_stall,                // Store queue stall signals
    // to dispatch
    output logic [2:0][`ROB-1:0] dispatch_index, // Allocated ROB indices for new entries
    output logic [2:0] struct_stall,             // Structural hazard stalls
    // to retire
    output ROB_ENTRY_PACKET[2:0] retire_entry   // Retired entries (up to 3 per cycle)

    `ifdef TEST_MODE
    , output ROB_ENTRY_PACKET [`ROBW-1:0] rob_entries_display
    , output [`ROB-1:0] head_display
    , output [`ROB-1:0] tail_display
    `endif 
);

// ROB storage array
ROB_ENTRY_PACKET [`ROBW-1:0] rob_entries;
ROB_ENTRY_PACKET [`ROBW-1:0] rob_entries_next;

// Head, Tail, input start pointers
logic [`ROB-1:0] head, tail;
logic [`ROB-1:0] input_start, input_start_incre1, input_start_incre2, input_start_incre3;

// Head, Tail, input start update pointers
logic [2:0] input_num;
logic [2:0] head_incre_temp, head_incre;
logic [2:0] tail_incre;
logic [`ROB-1:0] head_next, tail_next;
logic [`ROB-1:0] head_tail_diff;
logic empty, empty_temp, empty_next;
assign input_start_incre1 = input_start + 1;
assign input_start_incre2 = input_start + 2;
assign input_start_incre3 = input_start + 3;
logic [`ROB:0] space_left;


// Debug/Test outputs
`ifdef TEST_MODE
assign rob_entries_display = rob_entries;
assign head_display = head;
assign tail_display = tail;
`endif


// Dispatch count based on valid inputs
// tail increment update logic
assign tail_incre = (rob_in[0].valid & rob_in[1].valid & rob_in[2].valid) ? 3 :
                    (rob_in[1].valid & rob_in[2].valid) ? 2 :
                    (rob_in[2].valid) ? 1 : 0;

// Compute free space left in ROB
// output structural stall logic
assign head_tail_diff = tail - head;
assign space_left = (empty) ? 32 : 31 - head_tail_diff + head_incre;
assign struct_stall = (space_left == 0) ? 3'b111 :
                      (space_left == 1) ? 3'b011 :
                      (space_left == 2) ? 3'b001 :
                      3'b000;

/* -------------------- Head Pointer Management -------------------- */
// Calculate how many completed instructions at the head can be retired

logic [`ROB-1:0] head_offset[3];
always_comb begin
    // Precompute head offsets
    for (int i = 0; i < 3; i++) begin
          head_offset[i] = head + i;
    end
end

always_comb begin
    head_incre_temp = 0;
    if (rob_entries[head].completed) begin
        head_incre_temp = 1;
        if (rob_entries[head_offset[1]].completed) begin
            head_incre_temp = 2;
            if (rob_entries[head_offset[2]].completed) begin
                head_incre_temp = 3;
            end
        end
    end
end

// head increment update logic (rob internal retire logic), adjusted based on store queue stalls
logic [2:0] is_store; 

always_comb begin

    for (int i = 0; i < 3; i++) begin
        is_store[i] = (i == 0) ? rob_entries[head].is_store
                                     : rob_entries[head_offset[i]].is_store;
    end


	head_incre = head_incre_temp;  // set by default, if no sq_stall && store insn, no change
	priority case (sq_stall)
		3'b111: begin
			if (is_store[0])       head_incre = 0;
			else if (is_store[1])  head_incre = (head_incre_temp >= 1) ? 1 : head_incre_temp;
			else if (is_store[2])  head_incre = (head_incre_temp >= 2) ? 2 : head_incre_temp;
		end
		3'b011: begin
			if (is_store[0]) begin
				if (is_store[1])  head_incre = (head_incre_temp >= 1) ? 1 : head_incre_temp;
				else if (is_store[2])  head_incre = (head_incre_temp >= 2) ? 2 : head_incre_temp;
			end
			else if (is_store[1]) begin
				if (is_store[2])  head_incre = (head_incre_temp >= 2) ? 2 : head_incre_temp;
			end
		end
		3'b001: begin
			if (is_store[0] && is_store[1] && is_store[2])  head_incre = (head_incre_temp >= 2) ? 2 : head_incre_temp;
		end

		default: begin
			head_incre = head_incre_temp;
		end
	endcase
end
// Update head pointer and empty flag
// head pointer and empty flag update logic, empty_temp = 1 indicates that head temporarily reaching tail and so that rob is empty right now 
always_comb begin
    empty_temp = empty;
    priority case (head_incre)
        3: begin  // Retire 3 entries
            head_next = (head_offset[2] == tail) ? head + 2 : head + 3;
            empty_temp = (head_offset[2] == tail) ? 1 : 0;
        end
        2: begin  // Retire 2 entries
            head_next = (head_offset[1] == tail) ? head + 1 : head + 2;
            empty_temp = (head_offset[1] == tail) ? 1 : 0;
        end
        1: begin  // Retire 1 entry
            head_next = (head == tail) ? head : head + 1;
            empty_temp = (head == tail) ? 1 : 0;
        end
        default: begin
            head_next = head;
            empty_temp = empty;
        end
    endcase
end

/* -------------------- Tail Pointer Management -------------------- */
// Calculate next tail position and how many new entries inserted
	logic [`ROB-1:0] tail_offset[3];
	assign tail_offset[0] = tail + 1;
	assign tail_offset[1] = tail + 2;
	assign tail_offset[2] = tail + 3;
	
always_comb begin
	tail_next = tail + tail_incre;
	input_start = tail + 1;
	empty_next = 0;
	input_num = 0;
	// edge case: ROB is empty
	if (tail == head_next && empty_temp) begin
		input_start = tail;
		tail_next = (tail_incre > 0) ? tail + tail_incre - 1 : tail;
		input_num = (tail_incre == 3) ? 3'b111 :
					(tail_incre == 2) ? 3'b110 :
					(tail_incre == 1) ? 3'b100 : 3'b000;
		empty_next = (tail_incre > 0) ? 0 : 1;
	end
	else begin
		priority case (tail_incre)
			3: begin
				if (tail_offset[0] == head_next) begin
					tail_next = tail;
					input_num = 3'b000;
				end
				else if (tail_offset[1] == head_next) begin
					tail_next = tail + 1;
					input_num = 3'b100;
				end
				else if (tail_offset[2] == head_next) begin
					tail_next = tail + 2;
					input_num = 3'b110;
				end
				else begin
					tail_next = tail + 3;
					input_num = 3'b111;
				end
			end
			2: begin
				if (tail_offset[0] == head_next) begin
					tail_next = tail;
					input_num = 3'b000;
				end
				else if (tail_offset[1] == head_next) begin
					tail_next = tail + 1;
					input_num = 3'b100;
				end
				else begin
					tail_next = tail + 2;
					input_num = 3'b110;
				end
			end
			1: begin
				if (tail_offset[0] == head_next) begin
					tail_next = tail;
					input_num = 3'b000;
				end
				else begin
					tail_next = tail + 1;
					input_num = 3'b100;
				end
			end
			default begin
				input_num = 3'b000;
				tail_next = tail;
			end
		endcase
	end
end

/* -------------------- ROB Update and Dispatch/Retire -------------------- */
always_comb begin
	rob_entries_next = rob_entries;
	retire_entry = 0;
	dispatch_index = 0;
	priority case (head_incre)
		3: begin
			retire_entry[2] = rob_entries[head];
			retire_entry[1] = rob_entries[head_offset[1]];
			retire_entry[0] = rob_entries[head_offset[2]];
			rob_entries_next[head] = 0;
			rob_entries_next[head_offset[1]] = 0;
			rob_entries_next[head_offset[2]] = 0;		
		end
		2: begin
			retire_entry[2] = rob_entries[head];
			retire_entry[1] = rob_entries[head_offset[1]];
			rob_entries_next[head] = 0;
			rob_entries_next[head_offset[1]] = 0;
		end
		1: begin
			retire_entry[2] = rob_entries[head];
			rob_entries_next[head] = 0;
		end
		default: begin
			retire_entry = 0;
		end
	endcase
	priority case (input_num)
		3'b111: begin
			rob_entries_next[input_start] = rob_in[2];
			dispatch_index[2] = input_start;
			rob_entries_next[input_start_incre1] = rob_in[1];
			dispatch_index[1] = input_start_incre1;	
			rob_entries_next[input_start_incre2] = rob_in[0];
			dispatch_index[0] = input_start_incre2;	
		end
		3'b110: begin
			rob_entries_next[input_start] = rob_in[2];
			dispatch_index[2] = input_start;
			rob_entries_next[input_start_incre1] = rob_in[1];	
			dispatch_index[1] = input_start_incre1;	
		end
		3'b100: begin
			rob_entries_next[input_start] = rob_in[2];
			dispatch_index[2] = input_start;	
		end
		default: begin
			dispatch_index = 0;
		end
	endcase
        // Handle instruction completion and branch misprediction
	for (int i = 0; i < 3; i++) begin
		if (complete_valid[i]) begin // insn completed signal from complete stage
			rob_entries_next[complete_entry[i]].completed = 1;
			rob_entries_next[complete_entry[i]].precise_state_need = 0;
		        // branch prediction recovery logic

			// precise_state_valid from complete stage, defined as if fu take branch
			// predict_direction, source logic in pp and bp, defined as if branch is predicted to be taken
                        if (precise_state_valid[i] == 0 && rob_entries[complete_entry[i]].predict_direction == 1) begin //fu doesn't take, predict take
				rob_entries_next[complete_entry[i]].precise_state_need = 1;  // bp predict wrong, precise state needs to recover
				rob_entries_next[complete_entry[i]].target_pc = rob_entries[complete_entry[i]].NPC; // recover to pc before branch taken
			end
			else if (precise_state_valid[i] == 1 && rob_entries[complete_entry[i]].predict_direction == 0) begin // fu take, predict not
				rob_entries_next[complete_entry[i]].precise_state_need = 1; // bp predict wrong, precise state needs to recover
				// target_pc from complete stage, defined as the taget pc of fu_branch
				rob_entries_next[complete_entry[i]].target_pc = target_pc[i]; // recover to actual branch address
			end
			else if (precise_state_valid[i] == 1 && rob_entries[complete_entry[i]].predict_direction == 1 && target_pc[i] != rob_entries[complete_entry[i]].predict_pc) begin  // both fu and predict take branch, but bp preditc pc wrong, need to recover to actual branch address
				rob_entries_next[complete_entry[i]].precise_state_need = 1;
				rob_entries_next[complete_entry[i]].target_pc = target_pc[i];
			end
			else begin // predict result the same as actual bp fu, no need to recover
				rob_entries_next[complete_entry[i]].precise_state_need = 0;
				rob_entries_next[complete_entry[i]].target_pc = 0;
			end
		end
	end
end

/* -------------------- ROB Register Update -------------------- */
// Sequential logic: Update pointers and entries on clock edge
always_ff @(posedge clock) begin
    if (reset | BPRecoverEN) begin
		head <= `SD 0;
		tail <= `SD 0;
		empty <= `SD 1;
        rob_entries <= `SD 0; 
	end	 
    else begin 
        rob_entries <= `SD rob_entries_next;
		head <= `SD head_next;
		tail <= `SD tail_next;
		empty <= `SD empty_next;
	end
end

endmodule
`endif

