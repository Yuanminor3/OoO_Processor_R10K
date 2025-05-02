`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module fetch_stage (
	input                   	clk,
	input                   	rst,

	input logic fch_rec_enable,
	input logic   	[`SYS_XLEN-1:0]     fch_precise_pc,

	input   [2:0][31:0]     	ld_cache_fetched_data,     	// <- icache.fetch_inst_words
	input   [2:0]           	fch_icache_valid_flags,    	// <- fetch_inst_valids
	input                   	icache_branch,    	// taken-branch signal
	input   [`SYS_XLEN-1:0]     	cs_retire_pc,      	// target pc: use if icache_branch is TRUE
	input   [2:0]           	fch_dispatch_stall,

	output  logic           	icache_pipeline_hold,  	// -> icache.icache_pipeline_hold
	output  logic [1:0]     	icache_shift,          	// -> icache.icache_shift
	output  [2:0][`SYS_XLEN-1:0]	icache_req_addr,       // -> icache.icache_req_addr
	output  IF_ID_PACKET[2:0]       fch_ifid_pkts,          // output data from fetch stage to lsq_disp_mask stage

	//branch predictor
	output   [2:0]           	bp_fetch_enable,
	output   [2:0] [`SYS_XLEN-1:0]   bp_fetch_addr

);

	logic   [2:0][`SYS_XLEN-1:0]	fch_pc_regs;         	// the three PC we are currently fetching
	logic   [2:0][`SYS_XLEN-1:0]	fch_pc_next;        	// the next three PC we are gonna fetch

	// the fch_pc_next[2] (smallest PC) is:
	//  1. target_PC, if take branch
	//  2. fch_pc_regs[2], if no branch and the current fch_pc_regs[2] is not in the cache
	//  3. fch_pc_regs[1], if no branch and the current fch_pc_regs[1] is not in the cache
	//  4. fch_pc_regs[0], if no branch and the current fch_pc_regs[0] is not in the cache
	//  5. fch_pc_regs[0] + 4 = fch_pc_regs[2] + 12, if no branch and all three PCs are in the cache

always_comb begin
	if (fch_rec_enable) begin //precise state
	fch_pc_next[2] = icache_branch 	? fch_precise_pc : 	// if icache_branch, go to the bp_fetch_addr
                    	fch_dispatch_stall[2]	? fch_pc_regs[2] :
                    	~fch_icache_valid_flags[2] ? fch_pc_regs[2] : 	// if the first inst isn't ready, wait until finish reading from memory
                    	fch_dispatch_stall[1]	? fch_pc_regs[1] :
                    	~fch_icache_valid_flags[1] ? fch_pc_regs[1] : 	// same for the second inst
                    	fch_dispatch_stall[0]	? fch_pc_regs[0] :                	 
                    	~fch_icache_valid_flags[0] ? fch_pc_regs[0] : 	// and the third inst
                    	fch_pc_regs[0] + 4;
	end else begin
	fch_pc_next[2] = icache_branch 	? cs_retire_pc : 	// if icache_branch, go to the target PC
                    	fch_dispatch_stall[2]	? fch_pc_regs[2] :
                    	~fch_icache_valid_flags[2] ? fch_pc_regs[2] : 	// if the first inst isn't ready, wait until finish reading from memory
                    	fch_dispatch_stall[1]	? fch_pc_regs[1] :
                    	~fch_icache_valid_flags[1] ? fch_pc_regs[1] : 	// same for the second inst
                    	fch_dispatch_stall[0]	? fch_pc_regs[0] :                	 
                    	~fch_icache_valid_flags[0] ? fch_pc_regs[0] : 	// and the third inst
                    	fch_pc_regs[0] + 4;
	end
	fch_pc_next[1] = fch_pc_next[2] + 4;
	fch_pc_next[0] = fch_pc_next[1] + 4;

end
logic [1:0] fch_first_valid_way, fch_first_stall_way;

always_comb begin
	// --- icache_shift logic with exact priority order ---
	if (icache_branch) begin
    	icache_shift = 2'd0;
	end else if (fch_dispatch_stall[2]) begin
    	icache_shift = 2'd0;
	end else if (~fch_icache_valid_flags[2]) begin
    	icache_shift = 2'd0;
	end else if (fch_dispatch_stall[1]) begin
    	icache_shift = 2'd1;
	end else if (~fch_icache_valid_flags[1]) begin
    	icache_shift = 2'd1;
	end else if (fch_dispatch_stall[0]) begin
    	icache_shift = 2'd2;
	end else if (~fch_icache_valid_flags[0]) begin
    	icache_shift = 2'd2;
	end else begin
    	icache_shift = 2'd0;
	end
end

always_comb begin
	// --- fch_first_valid_way logic (priority: 2 > 1 > 0) ---
	if (fch_icache_valid_flags[2]) begin
    	fch_first_valid_way = 2'd2;
	end else if (fch_icache_valid_flags[1]) begin
    	fch_first_valid_way = 2'd1;
	end else if (fch_icache_valid_flags[0]) begin
    	fch_first_valid_way = 2'd0;
	end else begin
    	fch_first_valid_way = 2'd3;
	end
end

always_comb begin
	// --- fch_first_stall_way logic (priority: 2 > 1 > 0) ---
	if (fch_dispatch_stall[2]) begin
    	fch_first_stall_way = 2'd2;
	end else if (fch_dispatch_stall[1]) begin
    	fch_first_stall_way = 2'd1;
	end else if (fch_dispatch_stall[0]) begin
    	fch_first_stall_way = 2'd0;
	end else begin
    	fch_first_stall_way = 2'd3;
	end
end

always_comb begin
	// --- icache_pipeline_hold logic ---
	if (fch_first_valid_way == 2'd2 && fch_first_stall_way != 2'd3 &&
    	fch_pc_regs[fch_first_valid_way][`SYS_XLEN-1:3] == fch_pc_regs[fch_first_stall_way][`SYS_XLEN-1:3]) begin
    	icache_pipeline_hold = 1'b1;
	end else begin
    	icache_pipeline_hold = 1'b0;
	end
end

	// Pass PC and NPC down pipeline w/instruction
	assign fch_ifid_pkts[2].NPC = fch_pc_regs[2] + 4;
	assign fch_ifid_pkts[2].PC  = fch_pc_regs[2];
	assign fch_ifid_pkts[1].NPC = fch_pc_regs[1] + 4;
	assign fch_ifid_pkts[1].PC  = fch_pc_regs[1];
	assign fch_ifid_pkts[0].NPC = fch_pc_regs[0] + 4;
	assign fch_ifid_pkts[0].PC  = fch_pc_regs[0];

	// Assign the valid bits of output
	assign fch_ifid_pkts[2].valid = fch_icache_valid_flags[2] ? 1'b1 : 1'b0;
	assign fch_ifid_pkts[1].valid = ~fch_ifid_pkts[2].valid ? 1'b0 :
                                	fch_icache_valid_flags[1] ? 1'b1 : 1'b0;
	assign fch_ifid_pkts[0].valid = ~fch_ifid_pkts[1].valid ? 1'b0 :
                                	fch_icache_valid_flags[0] ? 1'b1 : 1'b0;

	// Assign the inst part of output
	assign fch_ifid_pkts[2].inst  = ~fch_icache_valid_flags[2] ? 32'b0 : ld_cache_fetched_data[2];
	assign fch_ifid_pkts[1].inst  = ~fch_icache_valid_flags[1] ? 32'b0 : ld_cache_fetched_data[1];
	assign fch_ifid_pkts[0].inst  = ~fch_icache_valid_flags[0] ? 32'b0 : ld_cache_fetched_data[0];

	assign icache_req_addr[2] = fch_pc_regs[2];
	assign icache_req_addr[1] = fch_pc_regs[1];
	assign icache_req_addr[0] = fch_pc_regs[0];

	// branch predictor
	assign bp_fetch_enable[2] = icache_branch ? 0 : fch_ifid_pkts[2].valid;
	assign bp_fetch_enable[1] = icache_branch ? 0 : fch_ifid_pkts[1].valid;
	assign bp_fetch_enable[0] = icache_branch ? 0 : fch_ifid_pkts[0].valid;
	assign bp_fetch_addr[2] = fch_ifid_pkts[2].PC;
	assign bp_fetch_addr[1] = fch_ifid_pkts[1].PC;
	assign bp_fetch_addr[0] = fch_ifid_pkts[0].PC;

	// synopsys sync_set_reset "rst"
	always_ff @(posedge clk) begin
    	if(rst) begin
        	fch_pc_regs <= `SYS_SMALL_DELAY {`SYS_XLEN'd0, `SYS_XLEN'd4, `SYS_XLEN'd8};   	// initial PC value
    	end
    	else begin
        	fch_pc_regs <= `SYS_SMALL_DELAY fch_pc_next; // transition to next PC
    	end
	end

endmodule



