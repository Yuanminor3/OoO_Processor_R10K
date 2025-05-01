/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   			Modulename :  pipeline.v                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
`ifndef __PIPELINE_V__
`define __PIPELINE_V__

`define TEST_MODE
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

///////////////////////////////////////////////
//                                           //
//   	   Pipeline Instantiation            //
//                                           //
///////////////////////////////////////////////

module pipeline(
    input         clock,                     
    input         reset,                     
    input [3:0]   mem2proc_response,         // Current request tag from memory about 
    input [63:0]  mem2proc_data,             // Data coming back from memory
    input [3:0]   mem2proc_tag,              // Current reply tag from memory 

    output logic [1:0]  proc2mem_command,    // Command sent to memory
    output logic [`XLEN-1:0] proc2mem_addr,  // Address sent to memory
    output logic [63:0] proc2mem_data,       // Data sent to memory

    output logic        halt,		     // if pp is halted
    output logic [2:0]  inst_count	     

`ifdef TEST_MODE 
    //Dcache
    , output logic [31:0] [63:0] cache_data_disp
    , output logic [31:0] [8:0]  cache_tags_disp
    , output logic [31:0]        valids_disp
    , output logic [1:0] [15:0]  dirties_disp
    , output MHSRS_ENTRY_PACKET [`MHSRS_W-1:0] MHSRS_disp
    , output logic [`MHSRS-1:0] head_pointer
    , output logic [`MHSRS-1:0] issue_pointer
    , output logic [`MHSRS-1:0] tail_pointer
    , output logic [2:0]        sq_stall_cache_display
`endif

);

///////////////////////////////////////////////
//                                           //
//   	     Wire Logic Definition           //
//                                           //
///////////////////////////////////////////////

/* Fetch Stage */
IF_ID_PACKET [2:0]           if_d_packet;
logic    [2:0][31:0]         cache_data;
logic    [2:0]               cache_valid;
logic    [1:0]               fetch_shift;
logic    [2:0][`XLEN-1:0]    proc2Icache_addr;
logic    [2:0][63:0]         cachemem_data;
logic    [2:0]               cachemem_valid;
logic    [2:0][4:0]          current_index;
logic    [2:0][7:0]          current_tag;
logic    [4:0]               wr_index;
logic    [7:0]               wr_tag;
logic                        data_write_enable;
logic                        branchEN;
logic [`XLEN-1:0]            branch_target_pc;

/* Dispatch Stage */
// Inputs
IF_ID_PACKET [2:0]      dis_packet_in; 
// outputs
RS_IN_PACKET [2:0]      dis_rs_packet;
ROB_ENTRY_PACKET [2:0]  dis_rob_packet;
logic [2:0]             dis_new_pr_en;
logic [2:0]             dis_stall; 
// go to maptable
logic [2:0][`PR-1:0]	maptable_allocate_pr;
logic [2:0][4:0]	maptable_allocate_ar;
logic [2:0][4:0]	maptable_lookup_reg1_ar;
logic [2:0][4:0]	maptable_lookup_reg2_ar;
// go to SQ
logic [2:0]		sq_stall;
logic [2:0]		sq_alloc;
logic [2:0][`LSQ-1:0]	sq_tail_pos;

/* Reservation Station */
logic                   rs_reset;
logic [2:0]             rs_stall;
RS_S_PACKET [2:0]       rs_is_packet;

/* free list */
logic [2:0]             free_pr_valid;
logic [2:0][`PR-1:0]    free_pr;
logic [2:0]		DispatchEN;
logic [2:0] 		RetireEN;
logic [2:0][`PR-1:0] 	RetireReg;
logic [`ROB-1:0] 	BPRecoverHead;
logic [`ROB-1:0] 	FreelistHead;
logic [`ROB-1:0]        fl_distance;

/* map table */
logic BPRecoverEN;
logic [31:0][`PR-1:0] 	archi_maptable;
logic [2:0][`PR-1:0]    maptable_old_pr;
logic [2:0][`PR-1:0]    maptable_reg1_pr;
logic [2:0][`PR-1:0]    maptable_reg2_pr;
logic [2:0]             maptable_reg1_ready;
logic [2:0]             maptable_reg2_ready;
logic [31:0][`PR-1:0] 	archi_maptable_out;

/* Issue stage */
RS_S_PACKET [2:0]       is_packet_in;
ISSUE_FU_PACKET [2**`FU-1:0] is_fu_packet;
FU_FIFO_PACKET          fu_fifo_stall;
logic [2:0][`PR-1:0]    rda_idx, rdb_idx; // access pr

/* physical register */
logic [2:0][`XLEN-1:0]  rdb_out, rda_out;

/* arch map table */
logic 		    [2:0][`PR-1:0] 	Tnew_in;
logic 		    [2:0][4:0] 		Retire_AR;

/* Reorder Buffer */
logic [2:0][`ROB-1:0]           new_rob_index;    // ROB.dispatch_index <-> dispatch.rob_index
logic       [2:0]               complete_valid;
logic       [2:0][`ROB-1:0]     complete_entry;    // which ROB entry is done
ROB_ENTRY_PACKET [2:0]          rob_retire_entry;  // which ENTRY to be retired
logic 	    [`ROB:0] 		space_left;
logic       [2:0]               rob_stall;
ROB_ENTRY_PACKET [`ROBW-1:0]    rob_entries;
ROB_ENTRY_PACKET [`ROBW-1:0]    rob_debug;
logic       [`ROB-1:0]          head;
logic       [`ROB-1:0]          tail;
logic       [2:0]               SQRetireEN;

/* functional unit */
FU_STATE_PACKET                     fu_ready;
ISSUE_FU_PACKET     [2**`FU-1:0]    fu_packet_in;
FU_STATE_PACKET                     complete_stall;
FU_COMPLETE_PACKET  [2**`FU-1:0]    fu_c_packet;
FU_STATE_PACKET                     fu_finish;

/* sq */
logic [2:0]                 exe_valid;
SQ_ENTRY_PACKET [2:0]       exe_store;
logic [2:0][`LSQ-1:0]       exe_idx;
LOAD_SQ_PACKET [1:0]        load_lookup;
SQ_LOAD_PACKET [1:0]        load_forward;
SQ_ENTRY_PACKET [2:0]       cache_wb;
SQ_ENTRY_PACKET [2:0]       sq_head;
logic [2**`LSQ-1:0]         load_tail_ready;

// icache
logic                       hit_but_stall;
logic [1:0]                 icache2mem_command;
logic [`XLEN-1:0]           icache2mem_addr;
logic   [1:0] prefetch_command;
logic   [`XLEN-1:0] prefetch_addr;
// If you use TEST_MODE displays, also add:
logic [7:0]                     pref_count_display;
logic [`PREF-1:0][3:0]          mem_tag_display;
logic [`PREF-1:0][4:0]          store_prefetch_index_display;
logic [`PREF-1:0][7:0]          store_prefetch_tag_display;

// Dcache
logic [1:0][`XLEN-1:0]      cache_read_addr;
logic [1:0]                 cache_read_start;

logic [1:0]                 dcache2ctlr_command;
logic [`XLEN-1:0]           dcache2ctlr_addr;
logic [63:0]                dcache2ctlr_data;

logic [2:0]                 sq_stall_cache;
logic [1:0]                 is_hit;
logic [1:0][`XLEN-1:0]      ld_data;
logic [1:0]                 broadcast_fu;
logic [`XLEN-1:0]           broadcast_data;

/* mem controller */
logic [3:0]                 ctlr2icache_response;
logic [63:0]                ctlr2icache_data;
logic [3:0]                 ctlr2icache_tag;
logic                       d_request;

logic [3:0]                 ctlr2dcache_response;
logic [63:0]                ctlr2dcache_data;
logic [3:0]                 ctlr2dcache_tag;

/* Complete Stage */
CDB_T_PACKET                    cdb_t;
FU_COMPLETE_PACKET [2**`FU-1:0] fu_c_in;
FU_STATE_PACKET                 fu_to_complete;
logic       [2:0][`XLEN-1:0]    wb_value;
logic [2:0]                     finish_valid;
logic [2:0][`FU:0]              finish;
logic       [2:0]               precise_state_valid;
logic       [2:0][`XLEN-1:0]    target_pc;

/* Retire Stage */
logic       [2:0][`PR-1:0]	    map_ar_pr;
logic       [2:0][4:0]		    map_ar;
logic       [31:0][`PR-1:0]         recover_maptable;
logic       [`XLEN-1:0]             fetch_pc;
logic                               re_halt;

/* Branch Predictor */
logic                               update_EN;
logic       [`XLEN-1:0]             update_pc;
logic                               update_direction;
logic       [`XLEN-1:0]             update_target;

logic       [2:0]                   fetch_EN;
logic       [2:0] [`XLEN-1:0]       bp_fetch_pc;
logic       [2:0]                   predict_direction_next;
logic       [2:0] [`XLEN-1:0]       predict_pc_next;
logic       [2:0]                   predict_found;

//////////////////////////////////////////////////
//                                              //
//                  Fetch Stage                 //
//                                              //
//////////////////////////////////////////////////

cache ic_mem(
    .clock(clock),
    .reset(reset),
    .wr1_en(data_write_enable),     // <- icache.data_write_enable
    .wr1_idx(wr_index),             // <- icache.wr_index
    .rd1_idx(current_index),        // <- icache.current_index
    .wr1_tag(wr_tag),               // <- icache.wr_tag
    .rd1_tag(current_tag),          // <- icache.current_tag
    .wr1_data(ctlr2icache_data),    // <- controller.ctlr2icache_data
    .rd1_data(cachemem_data),       // -> icache.cachemem_data
    .rd1_valid(cachemem_valid)      // -> icache.mcachemem_valid
);

icache ic(
    .clock(clock),
    .reset(reset),
    .take_branch(branchEN),
    .Imem2proc_response(ctlr2icache_response), // <- controller.ctlr2icache_response
    .Imem2proc_data(ctlr2icache_data),         // <- controller.ctlr2icache_data
    .Imem2proc_tag(ctlr2icache_tag),           // <- controller.ctlr2icache_tag
    .d_request(d_request),                     // <- controller.d_request
    .shift(fetch_shift),                    // <- fetch.shift
    .proc2Icache_addr(proc2Icache_addr),    // <- fetch.proc2Icache_addr
    .cachemem_data(cachemem_data),          // <- cache.rd1_data
    .cachemem_valid(cachemem_valid),        // <- cache.rd1_valid

    .hit_but_stall(hit_but_stall),              

    .proc2Imem_command(icache2mem_command), // -> controller.icache2mem_command
    .proc2Imem_addr(icache2mem_addr),       // -> controller.icache2mem_addr
    .Icache_data_out(cache_data),           // -> fetch.cache_data
    .Icache_valid_out(cache_valid),         // -> fetch.cache_valid
    .current_index(current_index),          // -> cache.rd1_idx
    .current_tag(current_tag),              // -> cache.rd1_tag
    .wr_index(wr_index),                    // -> cache.wr_idx
    .wr_tag(wr_tag),                        // -> cache.wr_tag
    .data_write_enable(data_write_enable)   // -> cache.wr1_en
);

assign branchEN =   BPRecoverEN | dis_packet_in[2].predict_direction | dis_packet_in[1].predict_direction | dis_packet_in[0].predict_direction;

assign branch_target_pc =  	(dis_packet_in[2].predict_direction) ? dis_packet_in[2].predict_pc :
                        	(dis_packet_in[1].predict_direction) ? dis_packet_in[1].predict_pc :
                        	(dis_packet_in[0].predict_direction) ? dis_packet_in[0].predict_pc : 0;

// —— arbiter: fetch > prefetch
wire [1:0] ic_cmd   = icache2mem_command;
wire [`XLEN-1:0] ic_addr = icache2mem_addr;
wire [1:0] pf_cmd   = prefetch_command;
wire [`XLEN-1:0] pf_addr = prefetch_addr;

// ic_cmd > pf_cmd
wire [1:0] arb_command = (ic_cmd != BUS_NONE) ? ic_cmd : pf_cmd;
wire [`XLEN-1:0] arb_addr	= (ic_cmd != BUS_NONE) ? ic_addr : pf_addr;

fetch_stage fetch(
	.clock(clock),
	.reset(reset),
	//for precise state redo instruction
	.fetch_pc_ps (fetch_pc),
	.BPRecoverEN(BPRecoverEN),

	.take_branch(branchEN),             	// <- retire.BPRecoverEN
	.target_pc(branch_target_pc),       	// <- retire.target_pc
	.cache_data(cache_data),            	// <- icache.Icache_data_out
	.cache_valid(cache_valid),          	// <- icache.Icache_valid_out
	.dis_stall(dis_stall),              	// <- dispatch.stall
    
	.hit_but_stall(hit_but_stall),      	// -> icache.hit_but_stall
	.shift(fetch_shift),                	// -> icache.shift
	.proc2Icache_addr(proc2Icache_addr),	// -> icache.proc2Icache_addr
	.if_packet_out(if_d_packet),        	// -> dispatch

	.fetch_EN(fetch_EN),
	.fetch_pc(bp_fetch_pc)
);


//////////////////////////////////////////////////
//               PREFETCH MODULE                //
//////////////////////////////////////////////////
prefetch prefetch_0 (
    .clock                (clock),
    .reset                (reset),
    .Imem2pref_response   (ctlr2icache_response), // from mem_controller
    .Imem2pref_tag        (ctlr2icache_tag),      // from mem_controller
    .give_way             (~hit_but_stall),       // avoid collision with fetch
    .branch               (branchEN),
    .proc2Icache_addr     (proc2Icache_addr),
    .cachemem_valid       (cachemem_valid),
    .want_to_fetch        (icache2mem_command == BUS_LOAD),

    .already_fetched      (),                    
    .prefetch_command     (prefetch_command),
    .prefetch_addr        (prefetch_addr),
    .prefetch_index       (),                   
    .prefetch_tag         (),
    .prefetch_wr_enable   ()
);

//////////////////////////////////////////////////
//                                              //
//                 Mem controller               //
//                                              //
//////////////////////////////////////////////////

mem_controller mc (
    /* to mem */
    .mem2ctlr_response(mem2proc_response),  // <- mem.mem2proc_response
    .mem2ctlr_data(mem2proc_data),          // <- mem.mem2proc_data
    .mem2ctlr_tag(mem2proc_tag),            // <- mem.mem2proc_tag

    .ctlr2mem_command(proc2mem_command),    // -> mem.proc2mem_command
    .ctlr2mem_addr(proc2mem_addr),          // -> mem.proc2mem_addr
    .ctlr2mem_data(proc2mem_data),          // -> mem.proc2mem_data

    .icache2ctlr_command (arb_command),
    .icache2ctlr_addr (arb_addr),

    .ctlr2icache_response(ctlr2icache_response),
    .ctlr2icache_data(ctlr2icache_data),              
    .ctlr2icache_tag(ctlr2icache_tag),          // directly assign
    .d_request(d_request),                      // if high, mem is assigned to Dcache

    /* to Dcache */
    .dcache2ctlr_command(dcache2ctlr_command),  // <- dcache 
    .dcache2ctlr_addr(dcache2ctlr_addr),        // <- dcache 
    .dcache2ctlr_data(dcache2ctlr_data),        // <- dcache 

    .ctlr2dcache_response(ctlr2dcache_response),// -> dcache 
    .ctlr2dcache_data(ctlr2dcache_data),        // -> dcache
    .ctlr2dcache_tag(ctlr2dcache_tag)           // -> dcache 
);

//////////////////////////////////////////////////
//                                              //
//                 IF-D Register                //
//                                              //
//////////////////////////////////////////////////

// intermediate storage for branch-pruned and next packets
IF_ID_PACKET [2:0] dis_packet_in_branch;
IF_ID_PACKET [2:0] dis_packet_in_next;

// 1) prune any younger instructions when a branch is taken
always_comb begin
    // default: copy all three slots
    for (int i = 0; i < 3; i++)
        dis_packet_in_branch[i] = dis_packet_in[i];

    // if the oldest slot predicts branch, squash slot 1&0
    if (dis_packet_in[2].predict_direction) begin
        dis_packet_in_branch[1] = '0;
        dis_packet_in_branch[0] = '0;
    end
    // if the middle slot predicts branch, squash slot 0
    if (dis_packet_in[1].predict_direction) begin
        dis_packet_in_branch[0] = '0;
    end
end

// 2) form next‐cycle packets based on stall mask
always_comb begin
    // build a zero‐or‐fresh‐fetch template that matches dis_packet_in_next’s type
    IF_ID_PACKET [2:0] tmp_pkts;
    for (int j = 0; j < 3; j++) begin
        tmp_pkts[j] = branchEN ? '0      : if_d_packet[j];
        tmp_pkts[j].predict_direction = branchEN ? 1'b0 : predict_direction_next[j];
        tmp_pkts[j].predict_pc        = branchEN ? '0    : predict_pc_next[j];
    end

    // steer slots based on which are stalled
    priority case (dis_stall)
        3'b000: begin
            // no stall → take the template as is
            dis_packet_in_next = tmp_pkts;
        end
        3'b001: begin
            // stall slot0 only
            if (dis_packet_in_branch[0].valid)
                dis_packet_in_next = '{ dis_packet_in_branch[0],
                                       tmp_pkts[2],
                                       tmp_pkts[1] };
            else
                dis_packet_in_next = '{ tmp_pkts[2],
                                       tmp_pkts[1],
                                       dis_packet_in_branch[0] };
        end
        3'b011: begin
            // stall slots0&1
            if (dis_packet_in_branch[1].valid && dis_packet_in_branch[0].valid)
                dis_packet_in_next = '{ dis_packet_in_branch[1],
                                       dis_packet_in_branch[0],
                                       tmp_pkts[2] };
            else if (!dis_packet_in_branch[1].valid && dis_packet_in_branch[0].valid)
                dis_packet_in_next = '{ dis_packet_in_branch[0],
                                       tmp_pkts[2],
                                       dis_packet_in_branch[1] };
            else if (dis_packet_in_branch[1].valid && !dis_packet_in_branch[0].valid)
                dis_packet_in_next = '{ dis_packet_in_branch[1],
                                       tmp_pkts[2],
                                       dis_packet_in_branch[0] };
            else
                dis_packet_in_next = '{ tmp_pkts[2],
                                       dis_packet_in_branch[1],
                                       dis_packet_in_branch[0] };
        end
        3'b111: begin
            // all slots stalled → preserve pruned packets
            dis_packet_in_next = dis_packet_in_branch;
        end
        default: begin
            // catch-all → same as full stall
            dis_packet_in_next = dis_packet_in_branch;
        end
    endcase
end

// 3) clocked update: reset or latch the new packets
always_ff @(posedge clock) begin
    if (reset | BPRecoverEN) begin
        for (int k = 0; k < 3; k++) begin
            dis_packet_in[k] <= '0;
            dis_packet_in[k].inst <= `NOP;
        end
    end else begin
        for (int k = 0; k < 3; k++)
            dis_packet_in[k] <= dis_packet_in_next[k];
    end
end

//////////////////////////////////////////////////
//                                              //
//               DISPATCH-Stage                 //
//                                              //
//////////////////////////////////////////////////

// Calculate structural hazard stalls
assign dis_stall = rs_stall | rob_stall | ~free_pr_valid | sq_stall;

FU_SELECT [2:0] fu_sel_out;
IF_ID_PACKET [2:0] dis_packet_out;
logic       [2:0]                   dispatch_EN;
logic       [2:0] [`XLEN-1:0]       dispatch_pc;

dispatch_stage dipatch_0(
    // Input: Packets coming from the IF-D Register
    .dis_packet_in(dis_packet_in),

    // Inputs/Outputs for Reservation Station (RS)
    .rs_in(dis_rs_packet),       // Output: RS allocated packet

    // Inputs/Outputs for Reorder Buffer (ROB)
    .rob_index(new_rob_index),   // Output: New allocated ROB index
    .rob_in(dis_rob_packet),     // Output: ROB allocated packet

    // Inputs/Outputs for Physical Register File (Freelist)
    .free_pr_in(free_pr),            // Output: Allocated physical registers

    // Inputs/Outputs for Store Queue (SQ)
    .sq_alloc(sq_alloc),             // Output: SQ allocation signal
    .sq_tail_pos(sq_tail_pos),       // Input: SQ tail pointer

    // Inputs/Outputs for Maptable lookup/allocation
    .maptable_new_pr(maptable_allocate_pr),    // Output: New PRs to map
    .maptable_ar(maptable_allocate_ar),        // Output: Architectural reg to map
    .maptable_old_pr(maptable_old_pr),          // Output: Old PRs to free later

    .reg1_ar(maptable_lookup_reg1_ar),          // Output: src1 AR for lookup
    .reg2_ar(maptable_lookup_reg2_ar),          // Output: src2 AR for lookup

    .reg1_pr(maptable_reg1_pr),                 // Input: src1 PR
    .reg2_pr(maptable_reg2_pr),                 // Input: src2 PR
    .reg1_ready(maptable_reg1_ready),            // Input: src1 readiness
    .reg2_ready(maptable_reg2_ready),            // Input: src2 readiness

    // Control signals
    .new_pr_en(dis_new_pr_en),      // Output: New PR allocation enable
    .d_stall(dis_stall),            // Output: Dispatch stall back to fetch

    // Branch predictor interaction
    .fu_sel_out(fu_sel_out),
    .dis_packet_out (dis_packet_out)
);

// Send branch prediction information to BP unit
always_comb begin
    dispatch_EN = 0;
    dispatch_pc = 0;
    for (int i = 0; i < 3; i++) begin
        if (dis_packet_out[i].valid && fu_sel_out[i] == BRANCH) begin
            dispatch_EN[i] = 1;
            dispatch_pc[i] = dis_packet_out[i].PC;
        end
    end
end

//////////////////////////////////////////////////
//                                              //
//                   Maptable                   //
//                                              //
//////////////////////////////////////////////////

map_table map_table_0(
    .clock(clock),
    .reset(reset),
    .archi_maptable(archi_maptable),
    .BPRecoverEN(BPRecoverEN),
    .cdb_t_in(cdb_t),
    .maptable_new_ar(maptable_allocate_ar),
    .maptable_new_pr(maptable_allocate_pr),

    .reg1_ar(maptable_lookup_reg1_ar),
    .reg2_ar(maptable_lookup_reg2_ar),

    .reg1_tag(maptable_reg1_pr),
    .reg2_tag(maptable_reg2_pr),

    .reg1_ready(maptable_reg1_ready),
    .reg2_ready(maptable_reg2_ready),
    .Told_out(maptable_old_pr)

);

//////////////////////////////////////////////////
//                                              //
//             Reservation Station              //
//                                              //
//////////////////////////////////////////////////

RS RS_0(
    // Inputs
    .clock(clock),
    .reset(reset | BPRecoverEN),
    .rs_in(dis_rs_packet),
    .cdb_t(cdb_t),
    .fu_fifo_stall(fu_fifo_stall),
    .load_tail_ready(load_tail_ready), //-> SQ 
    
    // Outputs
    .issue_insts(rs_is_packet),
    .struct_stall(rs_stall)
);

//////////////////////////////////////////////////
//                                              //
//                RS-IS-Register                //
//                                              //
//////////////////////////////////////////////////

always_ff @(posedge clock) begin
    if (reset | BPRecoverEN) is_packet_in <= `SD 0;
    else is_packet_in <= `SD rs_is_packet;
end

//////////////////////////////////////////////////
//                                              //
//                  ISSUE-Stage                 //
//                                              //
//////////////////////////////////////////////////

ISSUE_FU_PACKET [2:0]   issue;

always_comb begin
    for(int i=0; i<3; i++) begin
        issue[i].valid = is_packet_in[i].valid;
        issue[i].op_sel = is_packet_in[i].op_sel;
        issue[i].NPC = is_packet_in[i].NPC;
        issue[i].PC = is_packet_in[i].PC;
        issue[i].opa_select = is_packet_in[i].opa_select;
        issue[i].opb_select = is_packet_in[i].opb_select;
        issue[i].inst = is_packet_in[i].inst;
        issue[i].halt = is_packet_in[i].halt;
        issue[i].rob_entry = is_packet_in[i].rob_entry;
        issue[i].sq_tail = is_packet_in[i].sq_tail;
        issue[i].dest_pr = is_packet_in[i].dest_pr;
        issue[i].r1_value = rda_out[i];
        issue[i].r2_value = rdb_out[i];
    end
end

// is fifo input
FU_STATE_PACKET fu_ready_is;
assign fu_ready_is = fu_ready & ~complete_stall;
issue_stage is_0(
    // Input
    .clock(clock),                            
    .reset(reset | BPRecoverEN),  
    .is_packet_in(is_packet_in),                            
    .issue_fu_packet(issue), 
    .fu_ready_is(fu_ready_is),

    // Output                       
    .fu_fifo_stall(fu_fifo_stall), 
    .is_fu_packet(is_fu_packet)
);

//////////////////////////////////////////////////
//                                              //
//                Physical Reg                  //
//                                              //
//////////////////////////////////////////////////

/* read pr value */

always_comb begin
    for(int i=0; i<3; i++) begin
        rda_idx[i] = is_packet_in[i].reg1_pr;
        rdb_idx[i] = is_packet_in[i].reg2_pr;
    end
end

physical_regfile pr_0(
    // Inputs PR tag
    .read_idx_1 (rda_idx),
    .read_idx_2 (rdb_idx),
    .write_data (wb_value),
    .write_idx (cdb_t),
    .clock (clock),
    .reset (reset),

    // Output
    .read_out_1(rda_out),
    .read_out_2(rdb_out)

);

//////////////////////////////////////////////////
//                                              //
//                IS-FU-Register                //
//                                              //
//////////////////////////////////////////////////
ISSUE_FU_PACKET [2**`FU-1:0] fu_packet_in_next;
always_comb begin
    fu_packet_in_next = fu_packet_in;
    for(int i=0; i<2**`FU; i++) begin
        if(~complete_stall[i])fu_packet_in_next[i] = is_fu_packet[i];
    end
end

always_ff @(posedge clock) begin
    if (reset | BPRecoverEN) fu_packet_in <= `SD 0;
    else fu_packet_in <= `SD fu_packet_in_next;
end

//////////////////////////////////////////////////
//                                          	//
//       	EXECUTE-Stage and SQ           	//
//                                  	 	//
//////////////////////////////////////////////////

SQ_ENTRY_PACKET [0:2**`LSQ-1] sq_reg_out;
logic [`LSQ-1:0] head_out;

execution_stage ex(
	.clock(clock),                       	 
	.reset(reset | BPRecoverEN), 	 
	// all fu I/O
	.complete_stall(complete_stall),
	.fu_packet_in(fu_packet_in),  	 
	.fu_ready(fu_ready),           	 
	.fu_finish(fu_finish),   	 
	.fu_c_packet(fu_c_packet),    	 
	// alu to sq
	.exe_valid(exe_valid),
	.exe_store(exe_store),
	.exe_idx(exe_idx),
	// fu_load <-> sq
	.load_forward(load_forward),
	.load_lookup(load_lookup),
	// fu_load <-> dcache
	.cache_read_addr(cache_read_addr),
	.cache_read_start(cache_read_start),
	.ld_data(ld_data),
	.is_hit(is_hit),
	.broadcast_fu(broadcast_fu),
	.broadcast_data(broadcast_data),
	// fu_branch to bp
	.update_EN(update_EN),
	.update_pc(update_pc),
	.update_direction(update_direction),
	.update_target(update_target)
);

SQ SQ_0(
	.clock(clock),
	.reset(reset | BPRecoverEN),
	.stall(sq_stall),         	// -> dispatch.
	.dispatch(sq_alloc),      	// <- dispatch.sq_alloc
	.tail_pos(sq_tail_pos),   	// -> dispatch.sq_tail_pos
	.exe_valid(exe_valid),    	// <- alu.exe_valid
	.exe_store(exe_store),    	// <- alu.exe_store
	.exe_idx(exe_idx),        	// <- alu.exe_idx
	.load_lookup(load_lookup),	// <- load.load_lookup
	.load_forward(load_forward),  // -> load.load_forward
	.retire(SQRetireEN),      	// <- retire. SQRetireEN
	.cache_wb(cache_wb),      	 
	.sq_head(sq_head),
	.head_out(head_out),
	.sq_reg_out(sq_reg_out)
);

////////////////////////////////////////
//	    Load Tail Ready	      //
////////////////////////////////////////


always_comb begin
	for(int i=0; i<2**`LSQ; i++) begin // for each tail position
    	load_tail_ready[i] = 1;
    	for(int j=0; j<2**`LSQ; j++) begin // for each entry
        	if( i >= head_out && j >= head_out && j < i) // is older than load tail
            	if (sq_reg_out[j].ready == 0)
    	load_tail_ready[i] = 0;
        	if ( i < head_out && (j < i || j >= head_out))
            	if (sq_reg_out[j].ready == 0)
    	load_tail_ready[i] = 0;
    	end
	end
end


//////////////////////////////////////////////////
//                                              //
//                 Data-Cache                   //
//                                              //
//////////////////////////////////////////////////

dcache dche_0(
    .clock(clock),
    .reset(reset),
    .Ctlr2proc_response(ctlr2dcache_response),
    .Ctlr2proc_data(ctlr2dcache_data),
    .Ctlr2proc_tag(ctlr2dcache_tag),
    .dcache2ctlr_command(dcache2ctlr_command),
    .dcache2ctlr_addr(dcache2ctlr_addr),
    .dcache2ctlr_data(dcache2ctlr_data),
    .sq_in(cache_wb),
    .sq_head(sq_head),
    .sq_stall(sq_stall_cache),
    .ld_addr_in(cache_read_addr),
    .ld_start(cache_read_start),
    .is_hit(is_hit),
    .ld_data(ld_data),
    .broadcast_fu(broadcast_fu),
    .broadcast_data(broadcast_data)

    `ifdef TEST_MODE
    , .cache_data_disp(cache_data_disp)
    , .cache_tags_disp(cache_tags_disp)
    , .valids_disp(valids_disp)
    , .dirties_disp(dirties_disp)
    , .MHSRS_disp(MHSRS_disp)
    , .head_pointer(head_pointer)
    , .issue_pointer(issue_pointer)
    , .tail_pointer(tail_pointer)
    `endif 
);

//////////////////////////////////////////////////
//                                              //
//                FU-C-Register                 //
//                                              //
//////////////////////////////////////////////////
FU_STATE_PACKET fu_result_waiting;
FU_COMPLETE_PACKET [2**`FU-1:0] fu_c_in_next;
always_comb begin
    for(int i=0; i<2**`FU; i++) begin
        if(fu_finish[i]) fu_c_in_next[i] = fu_c_packet[i]; 
        else if (complete_stall[i]) fu_c_in_next[i] = fu_c_in[i];
        else fu_c_in_next[i] = 0;
    end
end
// if something is coming from fu, prioirty is to take it
// else, if stall, keep the value in reg. 

always_ff @(posedge clock) begin
    if (reset | BPRecoverEN) begin
        fu_c_in <= `SD 0;
        fu_result_waiting <= `SD 0;
    end else begin
        fu_c_in <= `SD fu_c_in_next;
        fu_result_waiting <= `SD complete_stall;
    end
end

assign fu_to_complete = fu_result_waiting | fu_finish;

//////////////////////////////////////////////////
//                                              //
//                      ROB                     //
//                                              //
//////////////////////////////////////////////////
ROB rob_0(
    .clock(clock), 
    .reset(reset), 
    .rob_in(dis_rob_packet),                    // <- dispatch.rob_in
    .complete_valid(complete_valid),            // <- complete.complete_valid
    .complete_entry(complete_entry),            // <- complete.complete_entry
    .precise_state_valid(precise_state_valid),  // <- complete.precise_state_valid
    .target_pc(target_pc),                      // <- complete.target_pc
    .BPRecoverEN(BPRecoverEN),                  // <- retire.BPRecoverEN
    .sq_stall(sq_stall_cache),
    .dispatch_index(new_rob_index),             // -> dispatch.rob_index
    .retire_entry(rob_retire_entry),            // -> retire.rob_head_entry
    .space_left(space_left)
    //.struct_stall(rob_stall)                  // -> dispatch.rob_stall
);

// to dispatch
assign rob_stall = (space_left == 0) ? 3'b111 :
                   (space_left == 1) ? 3'b011 :
                   (space_left == 2) ? 3'b001 :
                   3'b000; 

//////////////////////////////////////////////////
//                                              //
//                Complete Stage                //
//                                              //
//////////////////////////////////////////////////

complete_stage cs(
    .clock(clock),
    .reset(reset | BPRecoverEN),
    .fu_finish(fu_to_complete),                 // <- fu.fu_finish
    .fu_c_in(fu_c_in),                          // <- fu.fu_c_in
    .fu_c_stall(complete_stall),                // -> fu.complete_stall
    .cdb_t(cdb_t),                              // -> cdb_t broadcast
    .wb_value(wb_value),                        // -> wb_value, to register file
    .complete_valid(complete_valid),            // -> ROB.complete_valid
    .complete_entry(complete_entry),            // -> ROB.complete_entry
    .finish_valid(finish_valid),
    .finish(finish),
    .target_pc(target_pc)                       // -> ROB.target_pc
);

always_comb begin
    for (int i = 0; i < 3; i++) begin
	precise_state_valid[i] = 1'b0;
        if (finish_valid[i] && fu_c_in[finish[i]].if_take_branch) begin
                precise_state_valid[i] = 1'b1;
        end
    end
end 

//////////////////////////////////////////////////
//                                              //
//                 Retire Stage                 //
//                                              //
//////////////////////////////////////////////////
    
assign halt = re_halt;

retire_stage retire_0(
    .rob_head_entry(rob_retire_entry),          // <- ROB.retire_entry

    .BPRecoverEN(BPRecoverEN),                  // -> ROB.BPRecoverEN, Freelist.BPRecoverEN, fetch.take_branch
    .target_pc(fetch_pc),                       // -> fetch.target_pc
    .archi_maptable(archi_maptable_out),        // <- arch map.archi_maptable
    .recover_maptable(archi_maptable),          // -> map table.archi_maptable

    .fl_distance(fl_distance),                  // <- Freelist.fl_distance
    .Retire_EN(RetireEN),                       // -> Freelist.RetireEN
    .SQRetireEN(SQRetireEN),                    // -> SQ.retire
    .halt(re_halt),
    .inst_count(inst_count)
);

    always_comb begin
        for (int i = 0; i < 3; i++) begin
            map_ar[i]     = rob_retire_entry[i].arch_reg;
            map_ar_pr[i]  = rob_retire_entry[i].Tnew;
            RetireReg[i]  = rob_retire_entry[i].Told;
        end
    end

    // Internal next-state and reset values
    logic [31:0][`PR-1:0] archi_maptable_reset;  // Initial mapping: AR[i] -> PR[i]
    logic [31:0][`PR-1:0] archi_maptable_next;   // Next-state value of archi_maptable

    // Compute default/reset state: each AR maps to its index
    always_comb begin : GenResetMap
        for (int i = 0; i < 32; i++) begin
            archi_maptable_reset[i] = i[`PR-1:0];  // Truncate or extend to match PR width
        end
    end

    // Sequential logic: update archi_maptable on clock edge
    always_ff @(posedge clock) begin : ArchMapRegister
        if (reset) begin
            archi_maptable_out <= `SD archi_maptable_reset;  // Reset to identity mapping
        end else begin
            archi_maptable_out <= `SD archi_maptable_next;   // Commit next-state updates
        end
    end

    // Combinational logic: compute next-state mapping
    always_comb begin : ComputeNextMap
        archi_maptable_next = archi_maptable_out;  // Start from current state

        // Process retire instructions from oldest to youngest
        for (int i = 2; i >= 0; i--) begin
            if (RetireEN[i]) begin
                archi_maptable_next[map_ar[i]] = map_ar_pr[i];
            end
        end
    end

//////////////////////////////////////////////////
//                                              //
//                   Free List                  //
//                                              //
//////////////////////////////////////////////////

Freelist fl_0(
    .clock(clock), 
    .reset(reset), 
    .DispatchEN(dis_new_pr_en),                 
    .RetireEN(RetireEN),                        // <- retire.RetireEN
    .RetireReg(RetireReg),                      // <- retire.RetireReg
    .BPRecoverEN(BPRecoverEN),                  // <- retire.BPRecoverEN
    .FreeReg(free_pr),                          // -> dispatch.free_pr_in 
    .Head(FreelistHead),                        // -> retire.FreelistHead
    .FreeRegValid(free_pr_valid),               // -> dispatch.free_reg_valid 
    .fl_distance(fl_distance)                   // -> retire.fl_distance
);

//////////////////////////////////////////////////
//                                              //
//            Branch Predictor                  //
//                                              //
//////////////////////////////////////////////////

branch_predictor bp_0(
    .clock(clock), 
    .reset(reset), 
    .update_EN(update_EN), 
    .update_pc(update_pc), 
    .update_direction(update_direction), 
    .update_target(update_target), 
    .dispatch_EN(dispatch_EN), 
    .dispatch_pc(dispatch_pc),
    .fetch_EN(fetch_EN), 
    .fetch_pc(bp_fetch_pc),
    .predict_found(predict_found), 
    .predict_direction(predict_direction_next), 
    .predict_pc(predict_pc_next)
);


endmodule

`endif // __PIPELINE_V__

