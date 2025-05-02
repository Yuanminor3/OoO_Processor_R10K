`ifndef __LSQUE_V__
`define __LSQUE_V__
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module SQ(
	input                   	clk,
	input                   	rst,  
	// lsq_disp_mask
	input [2:0]             	lsq_disp_mask, // from lsq_disp_mask, with lsq_stall_mask considered
	output logic [2:0]      	lsq_stall_mask, // to lsq_disp_mask and lsq_retire_mask
	output logic [2:0][`SYS_LSQ_ADDR_WIDTH-1:0] lsq_tail_alloc, // the newly allocated entry idx or the fl_tail_reg position if no allocation

	// rs
	//output logic [2**`SYS_LSQ_ADDR_WIDTH-1:0]  rsb_sq_ready_flags,

	// exe store (from alu)
	input [2:0]             	exs_store_flags,
	input SQ_ENTRY_PACKET [2:0] exe_store_entries,
	input [2:0][`SYS_LSQ_ADDR_WIDTH-1:0]   	exs_store_index,

	// LOAD (from Load fu)
	input LOAD_SQ_PACKET [1:0]  exs_load_req_pkts,
	output SQ_LOAD_PACKET [1:0] exs_load2pkts,
    
	// from lsq_retire_mask
	input [2:0]                 	lsq_retire_mask, // SQRetireEN

	// to dcache
	output SQ_ENTRY_PACKET [2:0]	lsq_wb_entry,
	output SQ_ENTRY_PACKET [2:0]	lsq_head_entry,

	output SQ_ENTRY_PACKET [0:2**`SYS_LSQ_ADDR_WIDTH-1] lsq_reg_snapshot,
	output logic [`SYS_LSQ_ADDR_WIDTH-1:0] lsq_head_ptr

);
// regs
logic [`SYS_LSQ_ADDR_WIDTH-1:0] fl_head_reg, fl_tail_reg; // fl_tail_reg points the first empty entry, fl_head_reg to the first valid entry
logic [`SYS_LSQ_ADDR_WIDTH-1:0] lsq_count; // record the store insn number so far, the reg is not allowed to be fl_buffer_full

SQ_ENTRY_PACKET [0:2**`SYS_LSQ_ADDR_WIDTH-1] lsq_entry;


assign lsq_reg_snapshot = lsq_entry;
assign lsq_head_ptr = fl_head_reg;

typedef struct packed {
	logic               	ready;
	logic [3:0]         	usebytes;
	logic [`SYS_XLEN-1:0]   	addr; // must be aligned with words
	logic [`SYS_XLEN-1:0]   	data;
} SQ_ENTRY_PACKET;

// next fl_head_reg and fl_tail_reg
logic [1:0] lsq_retire_count, lsq_dispatch_count;
logic [`SYS_LSQ_ADDR_WIDTH:0] lsq_head_nxt, lsq_tail_nxt, lsq_count_next; // can be larger than 2**`SYS_LSQ_ADDR_WIDTH
logic [`SYS_LSQ_ADDR_WIDTH-1:0] lsq_head_wrap, lsq_tail_wrap;

assign lsq_retire_count = lsq_retire_mask[0] + lsq_retire_mask[1] + lsq_retire_mask[2];
assign lsq_dispatch_count = lsq_disp_mask[0] + lsq_disp_mask[1] + lsq_disp_mask[2];

assign lsq_head_nxt = fl_head_reg + lsq_retire_count;
assign lsq_tail_nxt = fl_tail_reg + lsq_dispatch_count;
assign lsq_count_next = lsq_count + lsq_dispatch_count - lsq_retire_count;

assign lsq_head_wrap = lsq_head_nxt[`SYS_LSQ_ADDR_WIDTH-1:0];
assign lsq_tail_wrap = lsq_tail_nxt[`SYS_LSQ_ADDR_WIDTH-1:0];

always_ff @(posedge clk) begin
	if (rst) begin
    	fl_head_reg <=`SYS_SMALL_DELAY 0;
    	fl_tail_reg <=`SYS_SMALL_DELAY 0;
    	lsq_count <= `SYS_SMALL_DELAY 0;
	end else begin
    	fl_head_reg <= `SYS_SMALL_DELAY lsq_head_wrap;
    	fl_tail_reg <= `SYS_SMALL_DELAY lsq_tail_wrap;
    	lsq_count <= `SYS_SMALL_DELAY lsq_count_next;
	end
end

// lsq_stall_mask (dependent only on lsq_retire_mask)
logic [`SYS_LSQ_ADDR_WIDTH:0] num_empty_entries;
assign num_empty_entries = 2**`SYS_LSQ_ADDR_WIDTH - lsq_count;
always_comb begin
	if (num_empty_entries < 2) lsq_stall_mask = 3'b111;
	else if (num_empty_entries < 3) lsq_stall_mask = 3'b011;
	else if (num_empty_entries < 4) lsq_stall_mask = 3'b001;
	else lsq_stall_mask = 3'b000;
end

// set lsq_disp_mask index                             	 
always_comb begin
	lsq_tail_alloc = '{default: 0};
	case (lsq_disp_mask)
    	3'b000, 3'b001: begin
        	lsq_tail_alloc[2] = fl_tail_reg;
        	lsq_tail_alloc[1] = fl_tail_reg;
        	lsq_tail_alloc[0] = fl_tail_reg;
    	end
    	3'b010, 3'b011: begin
        	lsq_tail_alloc[2] = fl_tail_reg;
        	lsq_tail_alloc[1] = fl_tail_reg;
        	lsq_tail_alloc[0] = fl_tail_reg + 1;
    	end
    	3'b100, 3'b101: begin
        	lsq_tail_alloc[2] = fl_tail_reg;
        	lsq_tail_alloc[1] = fl_tail_reg + 1;
        	lsq_tail_alloc[0] = fl_tail_reg + 1;
    	end
    	3'b110, 3'b111: begin
        	lsq_tail_alloc[2] = fl_tail_reg;
        	lsq_tail_alloc[1] = fl_tail_reg + 1;
        	lsq_tail_alloc[0] = fl_tail_reg + 2;
    	end
	endcase
end

//logic [`SYS_LSQ_ADDR_WIDTH-1:0]  head_inc_1, head_inc_2;
//assign head_inc_1 = fl_head_reg+1;
//assign head_inc_2 = fl_head_reg+2;

// writeback lsq_retire_mask stores
always_comb begin
	lsq_wb_entry = lsq_head_entry;
	if (lsq_retire_count == 0) lsq_wb_entry[2].ready = 0;
	if (lsq_retire_count <= 1) lsq_wb_entry[1].ready = 0;
	if (lsq_retire_count <= 2) lsq_wb_entry[0].ready = 0;
end

always_comb begin
	for (int i = 0; i < 3; i++) begin
    	lsq_head_entry[2 - i] = lsq_entry[fl_head_reg + i];
	end
end


SQ_ENTRY_PACKET [0:2**`SYS_LSQ_ADDR_WIDTH-1] sq_reg_after_retire;
SQ_ENTRY_PACKET [0:2**`SYS_LSQ_ADDR_WIDTH-1] sq_reg_next;

// clear lsq_retire_mask entries
always_comb begin
	sq_reg_after_retire = lsq_entry;
	if (lsq_retire_count >= 1) sq_reg_after_retire[fl_head_reg] = 0;
	if (lsq_retire_count >= 2) sq_reg_after_retire[fl_head_reg+1] = 0;
	if (lsq_retire_count >= 3) sq_reg_after_retire[fl_head_reg+2] = 0;
end

always_comb begin
	sq_reg_next = sq_reg_after_retire;
	for (int i = 0; i < 3; i++) begin
    	if (exs_store_flags[i])
        	sq_reg_next[exs_store_index[i]] = exe_store_entries[i];
	end
end


always_ff @(posedge clk) begin
	if (rst)
    	lsq_entry <= `SYS_SMALL_DELAY 0;
	else lsq_entry <= `SYS_SMALL_DELAY sq_reg_next;
end

///////////////////////////////
////// Age Logic
///////////////////////////////

// reorder older stores
SQ_ENTRY_PACKET [1:0][2**`SYS_LSQ_ADDR_WIDTH-1:0] lsq_older_store_entry; // the younger, the higher idex
logic [1:0][2**`SYS_LSQ_ADDR_WIDTH-1:0] lsq_older_valid;

logic [1:0][`SYS_LSQ_ADDR_WIDTH-1:0] lsq_load_pos;
logic [1:0][2**`SYS_LSQ_ADDR_WIDTH-1:0] lsq_load_wait_mask;
logic [1:0][3:0][2**`SYS_LSQ_ADDR_WIDTH-1:0] lsq_byte_fwd_valid;
logic [1:0][3:0][2**`SYS_LSQ_ADDR_WIDTH-1:0] lsq_byte_fwd_select;
logic [1:0][2**`SYS_LSQ_ADDR_WIDTH-1:0][`SYS_LSQ_ADDR_WIDTH-1:0] lsq_original_index;

// operation on bit 0
// Calculate position of load's fl_tail_reg
assign lsq_load_pos[0] = exs_load_req_pkts[0].lsq_tail_alloc;
// operation on bit 1
assign lsq_load_pos[1] = exs_load_req_pkts[1].lsq_tail_alloc;
// Compute number of older stores relative to load
always_comb begin

// Mark stores that are logically older

	lsq_older_valid[0] = 0;
	lsq_older_valid[1] = 0;
	for (int i = 0; i < 2**`SYS_LSQ_ADDR_WIDTH; i++) begin
    	if (i + ((fl_head_reg <= lsq_load_pos[0]) ?
                    	lsq_load_pos[0] - fl_head_reg:
                    	2**`SYS_LSQ_ADDR_WIDTH - fl_head_reg + lsq_load_pos[0]) >= 2**`SYS_LSQ_ADDR_WIDTH)
        	lsq_older_valid[0][i] = 1;
    	if (i + ((fl_head_reg <= lsq_load_pos[1]) ?
                    	lsq_load_pos[1] - fl_head_reg :
                    	2**`SYS_LSQ_ADDR_WIDTH - fl_head_reg + lsq_load_pos[1]) >= 2**`SYS_LSQ_ADDR_WIDTH)
        	lsq_older_valid[1][i] = 1;
	end

// Calculate original indices of older stores

	for (int i = 0; i < 2**`SYS_LSQ_ADDR_WIDTH; i++) begin
    	lsq_original_index[0][i] = i + lsq_load_pos[0];
    	lsq_original_index[1][i] = i + lsq_load_pos[1];
	end


// Fetch older store entries from SQ
	for (int i = 0; i < 2**`SYS_LSQ_ADDR_WIDTH; i++) begin
    	lsq_older_store_entry[0][i] = lsq_entry[lsq_original_index[0][i]];
    	lsq_older_store_entry[1][i] = lsq_entry[lsq_original_index[1][i]];
	end


// Determine if load should lsq_stall_mask due to older store not ready

	for (int i = 0; i < 2**`SYS_LSQ_ADDR_WIDTH; i++) begin
    	lsq_load_wait_mask[0][i] = ~lsq_older_store_entry[0][i].ready & lsq_older_valid[0][i];
    	lsq_load_wait_mask[1][i] = ~lsq_older_store_entry[1][i].ready & lsq_older_valid[1][i];
	end
 	exs_load2pkts[0].lsq_stall_mask = |lsq_load_wait_mask[0];
 	exs_load2pkts[1].lsq_stall_mask = |lsq_load_wait_mask[1];
// Byte-wise forward match detection

    	lsq_byte_fwd_valid[0] = 0;
	lsq_byte_fwd_valid[1] = 0;
	for (int i = 0; i < 2**`SYS_LSQ_ADDR_WIDTH; i++) begin
    	if (lsq_older_valid[0][i] && lsq_older_store_entry[0][i].addr == exs_load_req_pkts[0].addr) begin
        	for (int j = 0; j < 4; j++) begin
            	lsq_byte_fwd_valid[0][j][i] = lsq_older_store_entry[0][i].usebytes[j];
        	end
    	end
if (lsq_older_valid[1][i] && lsq_older_store_entry[1][i].addr == exs_load_req_pkts[1].addr) begin
        	for (int j = 0; j < 4; j++) begin
            	lsq_byte_fwd_valid[1][j][i] = lsq_older_store_entry[1][i].usebytes[j];
        	end
    	end
	end
end
// Byte-level priority selectors
ps8 byte_sel_0(.rcmp_eq_flag(lsq_byte_fwd_valid[0][0]), .en(1'b1), .gnt(lsq_byte_fwd_select[0][0]));
ps8 byte_sel_1(.rcmp_eq_flag(lsq_byte_fwd_valid[0][1]), .en(1'b1), .gnt(lsq_byte_fwd_select[0][1]));
ps8 byte_sel_2(.rcmp_eq_flag(lsq_byte_fwd_valid[0][2]), .en(1'b1), .gnt(lsq_byte_fwd_select[0][2]));
ps8 byte_sel_3(.rcmp_eq_flag(lsq_byte_fwd_valid[0][3]), .en(1'b1), .gnt(lsq_byte_fwd_select[0][3]));
// Assemble load data from selected older store bytes
always_comb begin
	exs_load2pkts[0].data = 0;
	for (int i = 0; i < 2**`SYS_LSQ_ADDR_WIDTH; i++) begin
    	if (lsq_byte_fwd_select[0][0][i])
        	exs_load2pkts[0].data[7:0]   = lsq_older_store_entry[0][i].data[7:0];
    	if (lsq_byte_fwd_select[0][1][i])
        	exs_load2pkts[0].data[15:8]  = lsq_older_store_entry[0][i].data[15:8];
    	if (lsq_byte_fwd_select[0][2][i])
        	exs_load2pkts[0].data[23:16] = lsq_older_store_entry[0][i].data[23:16];
    	if (lsq_byte_fwd_select[0][3][i])
        	exs_load2pkts[0].data[31:24] = lsq_older_store_entry[0][i].data[31:24];
	end
end

// Byte-level priority selectors
ps8 byte_sel_4(.rcmp_eq_flag(lsq_byte_fwd_valid[1][0]), .en(1'b1), .gnt(lsq_byte_fwd_select[1][0]));
ps8 byte_sel_5(.rcmp_eq_flag(lsq_byte_fwd_valid[1][1]), .en(1'b1), .gnt(lsq_byte_fwd_select[1][1]));
ps8 byte_sel_6(.rcmp_eq_flag(lsq_byte_fwd_valid[1][2]), .en(1'b1), .gnt(lsq_byte_fwd_select[1][2]));
ps8 byte_sel_7(.rcmp_eq_flag(lsq_byte_fwd_valid[1][3]), .en(1'b1), .gnt(lsq_byte_fwd_select[1][3]));
// Assemble load data from selected older store bytes
always_comb begin
	exs_load2pkts[1].data = 0;
	for (int i = 0; i < 2**`SYS_LSQ_ADDR_WIDTH; i++) begin
    	if (lsq_byte_fwd_select[1][0][i])
        	exs_load2pkts[1].data[7:0]   = lsq_older_store_entry[1][i].data[7:0];
    	if (lsq_byte_fwd_select[1][1][i])
        	exs_load2pkts[1].data[15:8]  = lsq_older_store_entry[1][i].data[15:8];
    	if (lsq_byte_fwd_select[1][2][i])
        	exs_load2pkts[1].data[23:16] = lsq_older_store_entry[1][i].data[23:16];
    	if (lsq_byte_fwd_select[1][3][i])
        	exs_load2pkts[1].data[31:24] = lsq_older_store_entry[1][i].data[31:24];
	end

// Mark forwarded bytes used

	for (int i = 0; i < 4; i++) begin
    	exs_load2pkts[0].usebytes[i] = |lsq_byte_fwd_valid[0][i];

    	exs_load2pkts[1].usebytes[i] = |lsq_byte_fwd_valid[1][i];
	end
end

endmodule


`endif // __LSQUE_V__

