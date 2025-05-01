`ifndef __LSQUE_V__
`define __LSQUE_V__
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module SQ(
	input                   	clock,
	input                   	reset,  
	// dispatch
	input [2:0]             	dispatch, // from dispatch, with stall considered
	output logic [2:0]      	stall, // to dispatch and retire
	output logic [2:0][`LSQ-1:0] tail_pos, // the newly allocated entry idx or the tail position if no allocation

	// rs
	//output logic [2**`LSQ-1:0]  load_tail_ready,

	// exe store (from alu)
	input [2:0]             	exe_valid,
	input SQ_ENTRY_PACKET [2:0] exe_store,
	input [2:0][`LSQ-1:0]   	exe_idx,

	// LOAD (from Load fu)
	input LOAD_SQ_PACKET [1:0]  load_lookup,
	output SQ_LOAD_PACKET [1:0] load_forward,
    
	// from retire
	input [2:0]                 	retire, // SQRetireEN

	// to dcache
	output SQ_ENTRY_PACKET [2:0]	cache_wb,
	output SQ_ENTRY_PACKET [2:0]	sq_head,

	output SQ_ENTRY_PACKET [0:2**`LSQ-1] sq_reg_out,
	output logic [`LSQ-1:0] head_out

);
// regs
logic [`LSQ-1:0] head, tail; // tail points the first empty entry, head to the first valid entry
logic [`LSQ-1:0] filled_num; // record the store insn number so far, the reg is not allowed to be full

SQ_ENTRY_PACKET [0:2**`LSQ-1] sq_reg;


assign sq_reg_out = sq_reg;
assign head_out = head;

typedef struct packed {
	logic               	ready;
	logic [3:0]         	usebytes;
	logic [`XLEN-1:0]   	addr; // must be aligned with words
	logic [`XLEN-1:0]   	data;
} SQ_ENTRY_PACKET;

// next head and tail
logic [1:0] num_retire, num_dis;
logic [`LSQ:0] next_head, next_tail, next_filled_num; // can be larger than 2**`LSQ
logic [`LSQ-1:0] wrapped_head, wrapped_tail;

assign num_retire = retire[0] + retire[1] + retire[2];
assign num_dis = dispatch[0] + dispatch[1] + dispatch[2];

assign next_head = head + num_retire;
assign next_tail = tail + num_dis;
assign next_filled_num = filled_num + num_dis - num_retire;

assign wrapped_head = next_head[`LSQ-1:0];
assign wrapped_tail = next_tail[`LSQ-1:0];

always_ff @(posedge clock) begin
	if (reset) begin
    	head <=`SD 0;
    	tail <=`SD 0;
    	filled_num <= `SD 0;
	end else begin
    	head <= `SD wrapped_head;
    	tail <= `SD wrapped_tail;
    	filled_num <= `SD next_filled_num;
	end
end

// stall (dependent only on retire)
logic [`LSQ:0] num_empty_entries;
assign num_empty_entries = 2**`LSQ - filled_num;
always_comb begin
	if (num_empty_entries < 2) stall = 3'b111;
	else if (num_empty_entries < 3) stall = 3'b011;
	else if (num_empty_entries < 4) stall = 3'b001;
	else stall = 3'b000;
end

// set dispatch index                             	 
always_comb begin
	tail_pos = '{default: 0};
	case (dispatch)
    	3'b000, 3'b001: begin
        	tail_pos[2] = tail;
        	tail_pos[1] = tail;
        	tail_pos[0] = tail;
    	end
    	3'b010, 3'b011: begin
        	tail_pos[2] = tail;
        	tail_pos[1] = tail;
        	tail_pos[0] = tail + 1;
    	end
    	3'b100, 3'b101: begin
        	tail_pos[2] = tail;
        	tail_pos[1] = tail + 1;
        	tail_pos[0] = tail + 1;
    	end
    	3'b110, 3'b111: begin
        	tail_pos[2] = tail;
        	tail_pos[1] = tail + 1;
        	tail_pos[0] = tail + 2;
    	end
	endcase
end

//logic [`LSQ-1:0]  head_inc_1, head_inc_2;
//assign head_inc_1 = head+1;
//assign head_inc_2 = head+2;

// writeback retire stores
always_comb begin
	cache_wb = sq_head;
	if (num_retire == 0) cache_wb[2].ready = 0;
	if (num_retire <= 1) cache_wb[1].ready = 0;
	if (num_retire <= 2) cache_wb[0].ready = 0;
end

always_comb begin
	for (int i = 0; i < 3; i++) begin
    	sq_head[2 - i] = sq_reg[head + i];
	end
end


SQ_ENTRY_PACKET [0:2**`LSQ-1] sq_reg_after_retire;
SQ_ENTRY_PACKET [0:2**`LSQ-1] sq_reg_next;

// clear retire entries
always_comb begin
	sq_reg_after_retire = sq_reg;
	if (num_retire >= 1) sq_reg_after_retire[head] = 0;
	if (num_retire >= 2) sq_reg_after_retire[head+1] = 0;
	if (num_retire >= 3) sq_reg_after_retire[head+2] = 0;
end

always_comb begin
	sq_reg_next = sq_reg_after_retire;
	for (int i = 0; i < 3; i++) begin
    	if (exe_valid[i])
        	sq_reg_next[exe_idx[i]] = exe_store[i];
	end
end


always_ff @(posedge clock) begin
	if (reset)
    	sq_reg <= `SD 0;
	else sq_reg <= `SD sq_reg_next;
end

///////////////////////////////
////// Age Logic
///////////////////////////////

// reorder older stores
SQ_ENTRY_PACKET [1:0][2**`LSQ-1:0] older_stores; // the younger, the higher idex
logic [1:0][2**`LSQ-1:0] older_stores_valid;

logic [1:0][`LSQ-1:0] load_pos;
logic [1:0][2**`LSQ-1:0] waiting_store_addr;
logic [1:0][3:0][2**`LSQ-1:0] byte_forward_valid;
logic [1:0][3:0][2**`LSQ-1:0] byte_forward_sel;
logic [1:0][2**`LSQ-1:0][`LSQ-1:0] org_idx;

// operation on bit 0
// Calculate position of load's tail
assign load_pos[0] = load_lookup[0].tail_pos;
// operation on bit 1
assign load_pos[1] = load_lookup[1].tail_pos;
// Compute number of older stores relative to load
always_comb begin

// Mark stores that are logically older

	older_stores_valid[0] = 0;
	older_stores_valid[1] = 0;
	for (int i = 0; i < 2**`LSQ; i++) begin
    	if (i + ((head <= load_pos[0]) ?
                    	load_pos[0] - head:
                    	2**`LSQ - head + load_pos[0]) >= 2**`LSQ)
        	older_stores_valid[0][i] = 1;
    	if (i + ((head <= load_pos[1]) ?
                    	load_pos[1] - head :
                    	2**`LSQ - head + load_pos[1]) >= 2**`LSQ)
        	older_stores_valid[1][i] = 1;
	end

// Calculate original indices of older stores

	for (int i = 0; i < 2**`LSQ; i++) begin
    	org_idx[0][i] = i + load_pos[0];
    	org_idx[1][i] = i + load_pos[1];
	end


// Fetch older store entries from SQ
	for (int i = 0; i < 2**`LSQ; i++) begin
    	older_stores[0][i] = sq_reg[org_idx[0][i]];
    	older_stores[1][i] = sq_reg[org_idx[1][i]];
	end


// Determine if load should stall due to older store not ready

	for (int i = 0; i < 2**`LSQ; i++) begin
    	waiting_store_addr[0][i] = ~older_stores[0][i].ready & older_stores_valid[0][i];
    	waiting_store_addr[1][i] = ~older_stores[1][i].ready & older_stores_valid[1][i];
	end
 	load_forward[0].stall = |waiting_store_addr[0];
 	load_forward[1].stall = |waiting_store_addr[1];
// Byte-wise forward match detection

    	byte_forward_valid[0] = 0;
	byte_forward_valid[1] = 0;
	for (int i = 0; i < 2**`LSQ; i++) begin
    	if (older_stores_valid[0][i] && older_stores[0][i].addr == load_lookup[0].addr) begin
        	for (int j = 0; j < 4; j++) begin
            	byte_forward_valid[0][j][i] = older_stores[0][i].usebytes[j];
        	end
    	end
if (older_stores_valid[1][i] && older_stores[1][i].addr == load_lookup[1].addr) begin
        	for (int j = 0; j < 4; j++) begin
            	byte_forward_valid[1][j][i] = older_stores[1][i].usebytes[j];
        	end
    	end
	end
end
// Byte-level priority selectors
ps8 byte_sel_0(.req(byte_forward_valid[0][0]), .en(1'b1), .gnt(byte_forward_sel[0][0]));
ps8 byte_sel_1(.req(byte_forward_valid[0][1]), .en(1'b1), .gnt(byte_forward_sel[0][1]));
ps8 byte_sel_2(.req(byte_forward_valid[0][2]), .en(1'b1), .gnt(byte_forward_sel[0][2]));
ps8 byte_sel_3(.req(byte_forward_valid[0][3]), .en(1'b1), .gnt(byte_forward_sel[0][3]));
// Assemble load data from selected older store bytes
always_comb begin
	load_forward[0].data = 0;
	for (int i = 0; i < 2**`LSQ; i++) begin
    	if (byte_forward_sel[0][0][i])
        	load_forward[0].data[7:0]   = older_stores[0][i].data[7:0];
    	if (byte_forward_sel[0][1][i])
        	load_forward[0].data[15:8]  = older_stores[0][i].data[15:8];
    	if (byte_forward_sel[0][2][i])
        	load_forward[0].data[23:16] = older_stores[0][i].data[23:16];
    	if (byte_forward_sel[0][3][i])
        	load_forward[0].data[31:24] = older_stores[0][i].data[31:24];
	end
end

// Byte-level priority selectors
ps8 byte_sel_4(.req(byte_forward_valid[1][0]), .en(1'b1), .gnt(byte_forward_sel[1][0]));
ps8 byte_sel_5(.req(byte_forward_valid[1][1]), .en(1'b1), .gnt(byte_forward_sel[1][1]));
ps8 byte_sel_6(.req(byte_forward_valid[1][2]), .en(1'b1), .gnt(byte_forward_sel[1][2]));
ps8 byte_sel_7(.req(byte_forward_valid[1][3]), .en(1'b1), .gnt(byte_forward_sel[1][3]));
// Assemble load data from selected older store bytes
always_comb begin
	load_forward[1].data = 0;
	for (int i = 0; i < 2**`LSQ; i++) begin
    	if (byte_forward_sel[1][0][i])
        	load_forward[1].data[7:0]   = older_stores[1][i].data[7:0];
    	if (byte_forward_sel[1][1][i])
        	load_forward[1].data[15:8]  = older_stores[1][i].data[15:8];
    	if (byte_forward_sel[1][2][i])
        	load_forward[1].data[23:16] = older_stores[1][i].data[23:16];
    	if (byte_forward_sel[1][3][i])
        	load_forward[1].data[31:24] = older_stores[1][i].data[31:24];
	end

// Mark forwarded bytes used

	for (int i = 0; i < 4; i++) begin
    	load_forward[0].usebytes[i] = |byte_forward_valid[0][i];

    	load_forward[1].usebytes[i] = |byte_forward_valid[1][i];
	end
end

endmodule


`endif // __LSQUE_V__

