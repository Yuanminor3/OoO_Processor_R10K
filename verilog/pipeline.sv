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
    input         clk,                     
    input         rst,                     
    input [3:0]   mem_resp_code,         // Current request tag from memory about 
    input [63:0]  mem_resp_data,             // Data coming back from memory
    input [3:0]   mem_resp_id,              // Current reply tag from memory 

    output logic [1:0]  mem_req_cmd,    // Command sent to memory
    output logic [`SYS_XLEN-1:0] mem_req_addr,  // Address sent to memory
    output logic [63:0] mem_req_data,       // Data sent to memory

    output logic        halt,		     // if pp is halted
    output logic [2:0]  retired_inst_cnt	     

`ifdef TEST_MODE 
    //Dcache
    , output logic [31:0] [63:0] dc_dbg_data
    , output logic [31:0] [8:0]  dc_dbg_tag
    , output logic [31:0]        dc_dbg_valid
    , output logic [1:0] [15:0]  dc_dbg_dir
    , output MHSRS_ENTRY_PACKET [`SYS_MHSRS_NUM-1:0] mhsrs_entry_vector
    , output logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_head_index
    , output logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_issue_index
    , output logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_tail_index
    , output logic [2:0]        sq_cache_stall_status
`endif

);

///////////////////////////////////////////////
//                                           //
//   	     Wire Logic Definition           //
//                                           //
///////////////////////////////////////////////

/* Fetch Stage */
IF_ID_PACKET [2:0]           if_d_packet;
logic    [2:0][31:0]         ld_cache_fetched_data;
logic    [2:0]               fch_icache_valid_flags;
logic    [1:0]               fetch_shift;
logic    [2:0][`SYS_XLEN-1:0]    icache_req_addr;
logic    [2:0][63:0]         cachemem_data;
logic    [2:0]               cachemem_valid;
logic    [2:0][4:0]          icache_index;
logic    [2:0][7:0]          icache_id;
logic    [4:0]               icache_wr_index;
logic    [7:0]               icache_wr_id;
logic                        icache_wrEN;
logic                        branchEN;
logic [`SYS_XLEN-1:0]            branch_target_pc;

/* Dispatch Stage */
// Inputs
IF_ID_PACKET [2:0]      dispatch_if_pkts; 
// outputs
RS_S_PACKET [2:0]      dis_rs_packet;
ROB_ENTRY_PACKET [2:0]  dis_rob_packet;
logic [2:0]             dis_new_pr_en;
logic [2:0]             fch_dispatch_stall; 
// go to maptable
logic [2:0][`SYS_PHYS_REG-1:0]	maptable_allocate_pr;
logic [2:0][4:0]	maptable_allocate_ar;
logic [2:0][4:0]	maptable_lookup_reg1_ar;
logic [2:0][4:0]	maptable_lookup_reg2_ar;
// go to SQ
logic [2:0]		rb_sq_stall;
logic [2:0]		dispatch_sq_flags;
logic [2:0][`SYS_LSQ_ADDR_WIDTH-1:0]	dispatch_pointer_tail;

/* Reservation Station */
logic                   rs_reset;
logic [2:0]             rs_stall;
RS_S_PACKET [2:0]       rs_is_packet;

/* free list */
logic [2:0]             free_pr_valid;
logic [2:0][`SYS_PHYS_REG-1:0]    free_pr;
logic [2:0]		fl_dispatch_en_mask;
logic [2:0] 		fl_retire_en_mask;
logic [2:0][`SYS_PHYS_REG-1:0] 	fl_retired_pr_list;
logic [`SYS_ROB_ADDR_WIDTH-1:0] 	BPRecoverHead;
logic [`SYS_ROB_ADDR_WIDTH-1:0] 	FreelistHead;
logic [`SYS_ROB_ADDR_WIDTH-1:0]        fl_free_count;

/* map table */
logic fch_rec_enable;
logic [31:0][`SYS_PHYS_REG-1:0] 	mt_checkpoint_tbl;
logic [2:0][`SYS_PHYS_REG-1:0]    dispatch_oldprs;
logic [2:0][`SYS_PHYS_REG-1:0]    maptable_reg1_pr;
logic [2:0][`SYS_PHYS_REG-1:0]    maptable_reg2_pr;
logic [2:0]             maptable_reg1_ready;
logic [2:0]             maptable_reg2_ready;
logic [31:0][`SYS_PHYS_REG-1:0] 	archi_maptable_out;

/* Issue stage */
RS_S_PACKET [2:0]       iss_rs_in_pkts;
ISSUE_FU_PACKET [2**`SYS_FU_ADDR_WIDTH-1:0] iss_issued_fu_pkts;
FU_FIFO_PACKET          fu_fifo_stall;
logic [2:0][`SYS_PHYS_REG-1:0]    rda_idx, rdb_idx; // access pr

/* physical register */
logic [2:0][`SYS_XLEN-1:0]  rdb_out, rda_out;

/* arch map table */
logic 		    [2:0][`SYS_PHYS_REG-1:0] 	Tnew_in;
logic 		    [2:0][4:0] 		Retire_AR;

/* Reorder Buffer */
logic [2:0][`SYS_ROB_ADDR_WIDTH-1:0]           new_rob_index;    // SYS_ROB_ADDR_WIDTH.dispatch_index <-> lsq_disp_mask.dispatch_idx
logic       [2:0]               cs_retire_valid;
logic       [2:0][`SYS_ROB_ADDR_WIDTH-1:0]     cs_retire_idx;    // which SYS_ROB_ADDR_WIDTH entry is done
ROB_ENTRY_PACKET [2:0]          rob_retire_entry;  // which ENTRY to be retired
logic 	    [`SYS_ROB_ADDR_WIDTH:0] 		rb_free_slots;
logic       [2:0]               rob_stall;
ROB_ENTRY_PACKET [`SYS_ROB_ENTRIES-1:0]    rb_entry;
ROB_ENTRY_PACKET [`SYS_ROB_ENTRIES-1:0]    rob_debug;
logic       [`SYS_ROB_ADDR_WIDTH-1:0]          fl_head_reg;
logic       [`SYS_ROB_ADDR_WIDTH-1:0]          fl_tail_reg;
logic       [2:0]               SQRetireEN;

/* functional unit */
FU_STATE_PACKET                     rsb_fu_ready;
ISSUE_FU_PACKET     [2**`SYS_FU_ADDR_WIDTH-1:0]    bs_in_pkt;
FU_STATE_PACKET                     bs_hazard;
FU_COMPLETE_PACKET  [2**`SYS_FU_ADDR_WIDTH-1:0]    exs_fu_completion_pkts;
FU_STATE_PACKET                     cs_fu_done_flags;

/* sq */
logic [2:0]                 exs_store_flags;
SQ_ENTRY_PACKET [2:0]       exe_store_entries;
logic [2:0][`SYS_LSQ_ADDR_WIDTH-1:0]       exs_store_index;
LOAD_SQ_PACKET [1:0]        exs_load_req_pkts;
SQ_LOAD_PACKET [1:0]        exs_load2pkts;
SQ_ENTRY_PACKET [2:0]       lsq_wb_entry;
SQ_ENTRY_PACKET [2:0]       lsq_head_entry;
logic [2**`SYS_LSQ_ADDR_WIDTH-1:0]         rsb_sq_ready_flags;

// icache
logic                       icache_pipeline_hold;
logic [1:0]                 icache2mem_command;
logic [`SYS_XLEN-1:0]           icache2mem_addr;
logic   [1:0] icache_pref_cmd;
logic   [`SYS_XLEN-1:0] icache_pref_addr;
// If you use TEST_MODE displays, also add:
logic [7:0]                     pref_count_display;
logic [`SYS_PREFETCH_DISTANCE-1:0][3:0]          mem_tag_display;
logic [`SYS_PREFETCH_DISTANCE-1:0][4:0]          store_prefetch_index_display;
logic [`SYS_PREFETCH_DISTANCE-1:0][7:0]          store_prefetch_tag_display;

// Dcache
logic [1:0][`SYS_XLEN-1:0]      exs_dcache_req_addr;
logic [1:0]                 exs_dcache_req_valid;

logic [1:0]                 mc_dc_cmd;
logic [`SYS_XLEN-1:0]           mc_dc_addr;
logic [63:0]                mc_dc_wr_data;

logic [2:0]                 sq_stall_cache;
logic [1:0]                 exs_dcache_hit_flags;
logic [1:0][`SYS_XLEN-1:0]      exs_dcache_resp_data;
logic [1:0]                 exs_dcache_brdcast_mask;
logic [`SYS_XLEN-1:0]           exs_dcache_brdcast_data;

/* mem controller */
logic [3:0]                 mc_to_ic_resp;
logic [63:0]                mc_to_ic_rd_data;
logic [3:0]                 mc_to_ic_rd_id;
logic                       mc_ic_hold_flag;

logic [3:0]                 mc_to_dc_resp;
logic [63:0]                mc_to_dc_rd_data;
logic [3:0]                 mc_to_dc_rd_id;

/* Complete Stage */
CDB_T_PACKET                    cs_cdb_broadcast;
FU_COMPLETE_PACKET [2**`SYS_FU_ADDR_WIDTH-1:0] cs_fu_complete_pkts;
FU_STATE_PACKET                 fu_to_complete;
logic       [2:0][`SYS_XLEN-1:0]    cs_wb_data;
logic [2:0]                     cs_finish_valid;
logic [2:0][`SYS_FU_ADDR_WIDTH:0]              finish;
logic       [2:0]               rb_recover_valid;
logic       [2:0][`SYS_XLEN-1:0]    cs_retire_pc;

/* Retire Stage */
logic       [2:0][`SYS_PHYS_REG-1:0]	    map_ar_pr;
logic       [2:0][4:0]		    map_ar;
logic       [31:0][`SYS_PHYS_REG-1:0]         recover_maptable;
logic       [`SYS_XLEN-1:0]             bp_fetch_addr;
logic                               re_halt;

/* Branch Predictor */
logic                               bs_upd_en;
logic       [`SYS_XLEN-1:0]             bs_upd_pc;
logic                               bs_upd_taken;
logic       [`SYS_XLEN-1:0]             bs_upd_target;

logic       [2:0]                   bp_fetch_enable;
logic       [2:0] [`SYS_XLEN-1:0]       bp_fetch_pc;
logic       [2:0]                   predict_direction_next;
logic       [2:0] [`SYS_XLEN-1:0]       predict_pc_next;
logic       [2:0]                   bp_pred_valid;

//////////////////////////////////////////////////
//                                              //
//                  Fetch Stage                 //
//                                              //
//////////////////////////////////////////////////

cache ic_mem(
    .clk(clk),
    .rst(rst),
    .dc_sq_wrEN(icache_wrEN),     // <- icache.icache_wrEN
    .dc_sq_wr_index(icache_wr_index),             // <- icache.icache_wr_index
    .dc_Id_index(icache_index),        // <- icache.icache_index
    .dc_sq_wr_id(icache_wr_id),               // <- icache.icache_wr_id
    .dc_Id_id(icache_id),          // <- icache.icache_id
    .dc_sq_wr_data(mc_to_ic_rd_data),    // <- controller.mc_to_ic_rd_data
    .dc_Id_data(cachemem_data),       // -> icache.cachemem_data
    .dc_Id_valid(cachemem_valid)      // -> icache.mcachemem_valid
);

icache ic(
    .clk(clk),
    .rst(rst),
    .icache_branch(branchEN),
    .icache_mem_resp_code(mc_to_ic_resp), // <- controller.mc_to_ic_resp
    .icache_mem_resp_data(mc_to_ic_rd_data),         // <- controller.mc_to_ic_rd_data
    .icache_mem_resp_id(mc_to_ic_rd_id),           // <- controller.mc_to_ic_rd_id
    .mc_ic_hold_flag(mc_ic_hold_flag),                     // <- controller.mc_ic_hold_flag
    .icache_shift(fetch_shift),                    // <- fetch.icache_shift
    .icache_req_addr(icache_req_addr),    // <- fetch.icache_req_addr
    .cachemem_data(cachemem_data),          // <- cache.dc_Id_data
    .cachemem_valid(cachemem_valid),        // <- cache.dc_Id_valid

    .icache_pipeline_hold(icache_pipeline_hold),              

    .icache_mem_req_cmd(icache2mem_command), // -> controller.icache2mem_command
    .icache_mem_req_addr(icache2mem_addr),       // -> controller.icache2mem_addr
    .fetch_inst_words(ld_cache_fetched_data),           // -> fetch.ld_cache_fetched_data
    .fetch_inst_valids(fch_icache_valid_flags),         // -> fetch.fch_icache_valid_flags
    .icache_index(icache_index),          // -> cache.dc_Id_index
    .icache_id(icache_id),              // -> cache.dc_Id_id
    .icache_wr_index(icache_wr_index),                    // -> cache.wr_idx
    .icache_wr_id(icache_wr_id),                        // -> cache.icache_wr_id
    .icache_wrEN(icache_wrEN)   // -> cache.dc_sq_wrEN
);

assign branchEN =   fch_rec_enable | dispatch_if_pkts[2].bp_pred_taken | dispatch_if_pkts[1].bp_pred_taken | dispatch_if_pkts[0].bp_pred_taken;

assign branch_target_pc =  	(dispatch_if_pkts[2].bp_pred_taken) ? dispatch_if_pkts[2].bp_pred_target :
                        	(dispatch_if_pkts[1].bp_pred_taken) ? dispatch_if_pkts[1].bp_pred_target :
                        	(dispatch_if_pkts[0].bp_pred_taken) ? dispatch_if_pkts[0].bp_pred_target : 0;

// —— arbiter: fetch > prefetch
wire [1:0] ic_cmd   = icache2mem_command;
wire [`SYS_XLEN-1:0] ic_addr = icache2mem_addr;
wire [1:0] pf_cmd   = icache_pref_cmd;
wire [`SYS_XLEN-1:0] pf_addr = icache_pref_addr;

// ic_cmd > pf_cmd
wire [1:0] arb_command = (ic_cmd != BUS_NONE) ? ic_cmd : pf_cmd;
wire [`SYS_XLEN-1:0] arb_addr	= (ic_cmd != BUS_NONE) ? ic_addr : pf_addr;

fetch_stage fetch(
	.clk(clk),
	.rst(rst),
	//for precise state redo instruction
	.fch_precise_pc (bp_fetch_addr),
	.fch_rec_enable(fch_rec_enable),

	.icache_branch(branchEN),             	// <- lsq_retire_mask.fch_rec_enable
	.cs_retire_pc(branch_target_pc),       	// <- lsq_retire_mask.cs_retire_pc
	.ld_cache_fetched_data(ld_cache_fetched_data),            	// <- icache.fetch_inst_words
	.fch_icache_valid_flags(fch_icache_valid_flags),          	// <- icache.fetch_inst_valids
	.fch_dispatch_stall(fch_dispatch_stall),              	// <- lsq_disp_mask.lsq_stall_mask
    
	.icache_pipeline_hold(icache_pipeline_hold),      	// -> icache.icache_pipeline_hold
	.icache_shift(fetch_shift),                	// -> icache.icache_shift
	.icache_req_addr(icache_req_addr),	// -> icache.icache_req_addr
	.fch_ifid_pkts(if_d_packet),        	// -> lsq_disp_mask

	.bp_fetch_enable(bp_fetch_enable),
	.bp_fetch_addr(bp_fetch_pc)
);


//////////////////////////////////////////////////
//               PREFETCH MODULE                //
//////////////////////////////////////////////////
prefetch prefetch_0 (
    .clk                (clk),
    .rst                (rst),
    .Imem2pref_response   (mc_to_ic_resp), // from mem_controller
    .Imem2pref_tag        (mc_to_ic_rd_id),      // from mem_controller
    .pf_bus_priority             (~icache_pipeline_hold),       // avoid collision with fetch
    .branch               (branchEN),
    .icache_req_addr     (icache_req_addr),
    .cachemem_valid       (cachemem_valid),
    .want_to_fetch        (icache2mem_command == BUS_LOAD),

    .icache_pref_done      (),                    
    .icache_pref_cmd     (icache_pref_cmd),
    .icache_pref_addr        (icache_pref_addr),
    .icache_pref_idx       (),                   
    .icache_pref_id         (),
    .icache_pref_wEN   ()
);

//////////////////////////////////////////////////
//                                              //
//                 Mem controller               //
//                                              //
//////////////////////////////////////////////////

mem_controller mc (
    /* to mem */
    .mc_mem_resp(mem_resp_code),  // <- mem.mem_resp_code
    .mc_mem_rd_data(mem_resp_data),          // <- mem.mem_resp_data
    .mc_mem_rd_id(mem_resp_id),            // <- mem.mem_resp_id

    .mc_to_mem_cmd(mem_req_cmd),    // -> mem.mem_req_cmd
    .mc_to_mem_addr(mem_req_addr),          // -> mem.mem_req_addr
    .mc_to_mem_wr_data(mem_req_data),          // -> mem.mem_req_data

    .mc_ic_cmd (arb_command),
    .mc_ic_addr (arb_addr),

    .mc_to_ic_resp(mc_to_ic_resp),
    .mc_to_ic_rd_data(mc_to_ic_rd_data),              
    .mc_to_ic_rd_id(mc_to_ic_rd_id),          // directly assign
    .mc_ic_hold_flag(mc_ic_hold_flag),                      // if high, mem is assigned to Dcache

    /* to Dcache */
    .mc_dc_cmd(mc_dc_cmd),  // <- dcache 
    .mc_dc_addr(mc_dc_addr),        // <- dcache 
    .mc_dc_wr_data(mc_dc_wr_data),        // <- dcache 

    .mc_to_dc_resp(mc_to_dc_resp),// -> dcache 
    .mc_to_dc_rd_data(mc_to_dc_rd_data),        // -> dcache
    .mc_to_dc_rd_id(mc_to_dc_rd_id)           // -> dcache 
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
        dis_packet_in_branch[i] = dispatch_if_pkts[i];

    // if the oldest slot predicts branch, squash slot 1&0
    if (dispatch_if_pkts[2].bp_pred_taken) begin
        dis_packet_in_branch[1] = '0;
        dis_packet_in_branch[0] = '0;
    end
    // if the middle slot predicts branch, squash slot 0
    if (dispatch_if_pkts[1].bp_pred_taken) begin
        dis_packet_in_branch[0] = '0;
    end
end

// 2) form next‐cycle packets based on lsq_stall_mask mask
always_comb begin
    // build a zero‐or‐fresh‐fetch template that matches dis_packet_in_next’s type
    IF_ID_PACKET [2:0] tmp_pkts;
    for (int j = 0; j < 3; j++) begin
        tmp_pkts[j] = branchEN ? '0      : if_d_packet[j];
        tmp_pkts[j].bp_pred_taken = branchEN ? 1'b0 : predict_direction_next[j];
        tmp_pkts[j].bp_pred_target        = branchEN ? '0    : predict_pc_next[j];
    end

    // steer slots based on which are stalled
    priority case (fch_dispatch_stall)
        3'b000: begin
            // no lsq_stall_mask → take the template as is
            dis_packet_in_next = tmp_pkts;
        end
        3'b001: begin
            // lsq_stall_mask slot0 only
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
            // lsq_stall_mask slots0&1
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
            // catch-all → same as fl_buffer_full lsq_stall_mask
            dis_packet_in_next = dis_packet_in_branch;
        end
    endcase
end

// 3) clocked update: rst or latch the new packets
always_ff @(posedge clk) begin
    if (rst | fch_rec_enable) begin
        for (int k = 0; k < 3; k++) begin
            dispatch_if_pkts[k] <= '0;
            dispatch_if_pkts[k].inst <= `SYS_INST_NOP;
        end
    end else begin
        for (int k = 0; k < 3; k++)
            dispatch_if_pkts[k] <= dis_packet_in_next[k];
    end
end

//////////////////////////////////////////////////
//                                              //
//               DISPATCH-Stage                 //
//                                              //
//////////////////////////////////////////////////

// Calculate structural hazard stalls
assign fch_dispatch_stall = rs_stall | rob_stall | ~free_pr_valid | rb_sq_stall;

FU_SELECT [2:0] dispatc_unit_sel;
IF_ID_PACKET [2:0] dispatch_if_pkts_out;
logic       [2:0]                   bp_disp_enable;
logic       [2:0] [`SYS_XLEN-1:0]       bp_disp_addr;

dispatch_stage dipatch_0(
    // Input: Packets coming from the IF-D Register
    .dispatch_if_pkts(dispatch_if_pkts),

    // Inputs/Outputs for Reservation Station (RS)
    .dispatch_rs_pkts(dis_rs_packet),       // Output: RS allocated packet

    // Inputs/Outputs for Reorder Buffer (SYS_ROB_ADDR_WIDTH)
    .dispatch_idx(new_rob_index),   // Output: New allocated SYS_ROB_ADDR_WIDTH index
    .dispatch_rob_pkts(dis_rob_packet),     // Output: SYS_ROB_ADDR_WIDTH allocated packet

    // Inputs/Outputs for Physical Register File (Freelist)
    .dispatch_free_prs(free_pr),            // Output: Allocated physical registers

    // Inputs/Outputs for Store Queue (SQ)
    .dispatch_sq_flags(dispatch_sq_flags),             // Output: SQ allocation signal
    .dispatch_pointer_tail(dispatch_pointer_tail),       // Input: SQ fl_tail_reg pointer

    // Inputs/Outputs for Maptable lookup/allocation
    .dispatch_pr_alloc_tags(maptable_allocate_pr),    // Output: New PRs to map
    .dispatch_arch_regs(maptable_allocate_ar),        // Output: Architectural reg to map
    .dispatch_oldprs(dispatch_oldprs),          // Output: Old PRs to free later

    .dispatch_src1_arch_regs(maptable_lookup_reg1_ar),          // Output: src1 AR for lookup
    .dispatch_src2_arch_regs(maptable_lookup_reg2_ar),          // Output: src2 AR for lookup

    .dispatch_src1_pr(maptable_reg1_pr),                 // Input: src1 SYS_PHYS_REG
    .dispatch_src2_pr(maptable_reg2_pr),                 // Input: src2 SYS_PHYS_REG
    .dispatch_src1_rdy(maptable_reg1_ready),            // Input: src1 readiness
    .dispatch_src2_rdy(maptable_reg2_ready),            // Input: src2 readiness

    // Control signals
    .dispatch_pr_allocEN(dis_new_pr_en),      // Output: New SYS_PHYS_REG allocation enable
    .dispatch_stall_mask(fch_dispatch_stall),            // Output: Dispatch lsq_stall_mask back to fetch

    // Branch predictor interaction
    .dispatc_unit_sel(dispatc_unit_sel),
    .dispatch_if_pkts_out (dispatch_if_pkts_out)
);

// Send branch prediction information to SYS_BRANCH_PREDICTION unit
always_comb begin
    bp_disp_enable = 0;
    bp_disp_addr = 0;
    for (int i = 0; i < 3; i++) begin
        if (dispatch_if_pkts_out[i].valid && dispatc_unit_sel[i] == BRANCH) begin
            bp_disp_enable[i] = 1;
            bp_disp_addr[i] = dispatch_if_pkts_out[i].PC;
        end
    end
end

//////////////////////////////////////////////////
//                                              //
//                   Maptable                   //
//                                              //
//////////////////////////////////////////////////

map_table map_table_0(
    .clk(clk),
    .rst(rst),
    .mt_checkpoint_tbl(mt_checkpoint_tbl),
    .fch_rec_enable(fch_rec_enable),
    .cdb_t_in(cs_cdb_broadcast),
    .mt_new_arch_regs(maptable_allocate_ar),
    .dispatch_pr_alloc_tags(maptable_allocate_pr),

    .dispatch_src1_arch_regs(maptable_lookup_reg1_ar),
    .dispatch_src2_arch_regs(maptable_lookup_reg2_ar),

    .reg1_tag(maptable_reg1_pr),
    .reg2_tag(maptable_reg2_pr),

    .dispatch_src1_rdy(maptable_reg1_ready),
    .dispatch_src2_rdy(maptable_reg2_ready),
    .mt_old_pr_tags(dispatch_oldprs)

);

//////////////////////////////////////////////////
//                                              //
//             Reservation Station              //
//                                              //
//////////////////////////////////////////////////

RS RS_0(
    // Inputs
    .clk(clk),
    .rst(rst | fch_rec_enable),
    .dispatch_rs_pkts(dis_rs_packet),
    .cs_cdb_broadcast(cs_cdb_broadcast),
    .fu_fifo_stall(fu_fifo_stall),
    .rsb_sq_ready_flags(rsb_sq_ready_flags), //-> SQ 
    
    // Outputs
    .rsb_issue_packets(rs_is_packet),
    .rsb_struct_halt(rs_stall)
);

//////////////////////////////////////////////////
//                                              //
//                RS-IS-Register                //
//                                              //
//////////////////////////////////////////////////

always_ff @(posedge clk) begin
    if (rst | fch_rec_enable) iss_rs_in_pkts <= `SYS_SMALL_DELAY 0;
    else iss_rs_in_pkts <= `SYS_SMALL_DELAY rs_is_packet;
end

//////////////////////////////////////////////////
//                                              //
//                  ISSUE-Stage                 //
//                                              //
//////////////////////////////////////////////////

ISSUE_FU_PACKET [2:0]   issue;

always_comb begin
    for(int i=0; i<3; i++) begin
        issue[i].valid = iss_rs_in_pkts[i].valid;
        issue[i].dec_fu_opcode = iss_rs_in_pkts[i].dec_fu_opcode;
        issue[i].NPC = iss_rs_in_pkts[i].NPC;
        issue[i].PC = iss_rs_in_pkts[i].PC;
        issue[i].dec_operandA_mux = iss_rs_in_pkts[i].dec_operandA_mux;
        issue[i].dec_operandB_mux = iss_rs_in_pkts[i].dec_operandB_mux;
        issue[i].inst = iss_rs_in_pkts[i].inst;
        issue[i].halt = iss_rs_in_pkts[i].halt;
        issue[i].rob_entry = iss_rs_in_pkts[i].rob_entry;
        issue[i].sq_tail = iss_rs_in_pkts[i].sq_tail;
        issue[i].dispatch_allocated_prs = iss_rs_in_pkts[i].dispatch_allocated_prs;
        issue[i].r1_value = rda_out[i];
        issue[i].r2_value = rdb_out[i];
    end
end

// is fifo input
FU_STATE_PACKET fu_ready_is;
assign fu_ready_is = rsb_fu_ready & ~bs_hazard;
issue_stage is_0(
    // Input
    .clk(clk),                            
    .rst(rst | fch_rec_enable),  
    .iss_rs_in_pkts(iss_rs_in_pkts),                            
    .iss_issue_in_pkts(issue), 
    .fu_ready_is(fu_ready_is),

    // Output                       
    .fu_fifo_stall(fu_fifo_stall), 
    .iss_issued_fu_pkts(iss_issued_fu_pkts)
);

//////////////////////////////////////////////////
//                                              //
//                Physical Reg                  //
//                                              //
//////////////////////////////////////////////////

/* read pr value */

always_comb begin
    for(int i=0; i<3; i++) begin
        rda_idx[i] = iss_rs_in_pkts[i].dispatch_src1_pr;
        rdb_idx[i] = iss_rs_in_pkts[i].dispatch_src2_pr;
    end
end

physical_regfile pr_0(
    // Inputs SYS_PHYS_REG tag
    .rd_index_1 (rda_idx),
    .rd_index_2 (rdb_idx),
    .write_data (cs_wb_data),
    .write_idx (cs_cdb_broadcast),
    .clk (clk),
    .rst (rst),

    // Output
    .rd_result_1(rda_out),
    .rd_result_2(rdb_out)

);

//////////////////////////////////////////////////
//                                              //
//                IS-SYS_FU_ADDR_WIDTH-Register                //
//                                              //
//////////////////////////////////////////////////
ISSUE_FU_PACKET [2**`SYS_FU_ADDR_WIDTH-1:0] fu_packet_in_next;
always_comb begin
    fu_packet_in_next = bs_in_pkt;
    for(int i=0; i<2**`SYS_FU_ADDR_WIDTH; i++) begin
        if(~bs_hazard[i])fu_packet_in_next[i] = iss_issued_fu_pkts[i];
    end
end

always_ff @(posedge clk) begin
    if (rst | fch_rec_enable) bs_in_pkt <= `SYS_SMALL_DELAY 0;
    else bs_in_pkt <= `SYS_SMALL_DELAY fu_packet_in_next;
end

//////////////////////////////////////////////////
//                                          	//
//       	EXECUTE-Stage and SQ           	//
//                                  	 	//
//////////////////////////////////////////////////

SQ_ENTRY_PACKET [0:2**`SYS_LSQ_ADDR_WIDTH-1] lsq_reg_snapshot;
logic [`SYS_LSQ_ADDR_WIDTH-1:0] lsq_head_ptr;

execution_stage ex(
	.clk(clk),                       	 
	.rst(rst | fch_rec_enable), 	 
	// all fu I/O
	.bs_hazard(bs_hazard),
	.bs_in_pkt(bs_in_pkt),  	 
	.rsb_fu_ready(rsb_fu_ready),           	 
	.cs_fu_done_flags(cs_fu_done_flags),   	 
	.exs_fu_completion_pkts(exs_fu_completion_pkts),    	 
	// alu to sq
	.exs_store_flags(exs_store_flags),
	.exe_store_entries(exe_store_entries),
	.exs_store_index(exs_store_index),
	// fu_load <-> sq
	.exs_load2pkts(exs_load2pkts),
	.exs_load_req_pkts(exs_load_req_pkts),
	// fu_load <-> dcache
	.exs_dcache_req_addr(exs_dcache_req_addr),
	.exs_dcache_req_valid(exs_dcache_req_valid),
	.exs_dcache_resp_data(exs_dcache_resp_data),
	.exs_dcache_hit_flags(exs_dcache_hit_flags),
	.exs_dcache_brdcast_mask(exs_dcache_brdcast_mask),
	.exs_dcache_brdcast_data(exs_dcache_brdcast_data),
	// fu_branch to bp
	.bs_upd_en(bs_upd_en),
	.bs_upd_pc(bs_upd_pc),
	.bs_upd_taken(bs_upd_taken),
	.bs_upd_target(bs_upd_target)
);

SQ SQ_0(
	.clk(clk),
	.rst(rst | fch_rec_enable),
	.lsq_stall_mask(rb_sq_stall),         	// -> lsq_disp_mask.
	.lsq_disp_mask(dispatch_sq_flags),      	// <- lsq_disp_mask.dispatch_sq_flags
	.lsq_tail_alloc(dispatch_pointer_tail),   	// -> lsq_disp_mask.dispatch_pointer_tail
	.exs_store_flags(exs_store_flags),    	// <- alu.exs_store_flags
	.exe_store_entries(exe_store_entries),    	// <- alu.exe_store_entries
	.exs_store_index(exs_store_index),        	// <- alu.exs_store_index
	.exs_load_req_pkts(exs_load_req_pkts),	// <- load.exs_load_req_pkts
	.exs_load2pkts(exs_load2pkts),  // -> load.exs_load2pkts
	.lsq_retire_mask(SQRetireEN),      	// <- lsq_retire_mask. SQRetireEN
	.lsq_wb_entry(lsq_wb_entry),      	 
	.lsq_head_entry(lsq_head_entry),
	.lsq_head_ptr(lsq_head_ptr),
	.lsq_reg_snapshot(lsq_reg_snapshot)
);

////////////////////////////////////////
//	    Load Tail Ready	      //
////////////////////////////////////////


always_comb begin
	for(int i=0; i<2**`SYS_LSQ_ADDR_WIDTH; i++) begin // for each fl_tail_reg position
    	rsb_sq_ready_flags[i] = 1;
    	for(int j=0; j<2**`SYS_LSQ_ADDR_WIDTH; j++) begin // for each entry
        	if( i >= lsq_head_ptr && j >= lsq_head_ptr && j < i) // is older than load fl_tail_reg
            	if (lsq_reg_snapshot[j].ready == 0)
    	rsb_sq_ready_flags[i] = 0;
        	if ( i < lsq_head_ptr && (j < i || j >= lsq_head_ptr))
            	if (lsq_reg_snapshot[j].ready == 0)
    	rsb_sq_ready_flags[i] = 0;
    	end
	end
end


//////////////////////////////////////////////////
//                                              //
//                 Data-Cache                   //
//                                              //
//////////////////////////////////////////////////

dcache dche_0(
    .clk(clk),
    .rst(rst),
    .Ctlr2proc_response(mc_to_dc_resp),
    .Ctlr2proc_data(mc_to_dc_rd_data),
    .Ctlr2proc_tag(mc_to_dc_rd_id),
    .mc_dc_cmd(mc_dc_cmd),
    .mc_dc_addr(mc_dc_addr),
    .mc_dc_wr_data(mc_dc_wr_data),
    .sq_in(lsq_wb_entry),
    .lsq_head_entry(lsq_head_entry),
    .rb_sq_stall(sq_stall_cache),
    .ld_addr_in(exs_dcache_req_addr),
    .ld_start(exs_dcache_req_valid),
    .exs_dcache_hit_flags(exs_dcache_hit_flags),
    .exs_dcache_resp_data(exs_dcache_resp_data),
    .exs_dcache_brdcast_mask(exs_dcache_brdcast_mask),
    .exs_dcache_brdcast_data(exs_dcache_brdcast_data)

    `ifdef TEST_MODE
    , .dc_dbg_data(dc_dbg_data)
    , .dc_dbg_tag(dc_dbg_tag)
    , .dc_dbg_valid(dc_dbg_valid)
    , .dc_dbg_dir(dc_dbg_dir)
    , .mhsrs_entry_vector(mhsrs_entry_vector)
    , .mhsrs_head_index(mhsrs_head_index)
    , .mhsrs_issue_index(mhsrs_issue_index)
    , .mhsrs_tail_index(mhsrs_tail_index)
    `endif 
);

//////////////////////////////////////////////////
//                                              //
//                SYS_FU_ADDR_WIDTH-C-Register                 //
//                                              //
//////////////////////////////////////////////////
FU_STATE_PACKET fu_result_waiting;
FU_COMPLETE_PACKET [2**`SYS_FU_ADDR_WIDTH-1:0] fu_c_in_next;
always_comb begin
    for(int i=0; i<2**`SYS_FU_ADDR_WIDTH; i++) begin
        if(cs_fu_done_flags[i]) fu_c_in_next[i] = exs_fu_completion_pkts[i]; 
        else if (bs_hazard[i]) fu_c_in_next[i] = cs_fu_complete_pkts[i];
        else fu_c_in_next[i] = 0;
    end
end
// if something is coming from fu, prioirty is to take it
// else, if lsq_stall_mask, keep the value in reg. 

always_ff @(posedge clk) begin
    if (rst | fch_rec_enable) begin
        cs_fu_complete_pkts <= `SYS_SMALL_DELAY 0;
        fu_result_waiting <= `SYS_SMALL_DELAY 0;
    end else begin
        cs_fu_complete_pkts <= `SYS_SMALL_DELAY fu_c_in_next;
        fu_result_waiting <= `SYS_SMALL_DELAY bs_hazard;
    end
end

assign fu_to_complete = fu_result_waiting | cs_fu_done_flags;

//////////////////////////////////////////////////
//                                              //
//                      SYS_ROB_ADDR_WIDTH                     //
//                                              //
//////////////////////////////////////////////////
ROB rob_0(
    .clk(clk), 
    .rst(rst), 
    .dispatch_rob_pkts(dis_rob_packet),                    // <- lsq_disp_mask.dispatch_rob_pkts
    .cs_retire_valid(cs_retire_valid),            // <- complete.cs_retire_valid
    .cs_retire_idx(cs_retire_idx),            // <- complete.cs_retire_idx
    .rb_recover_valid(rb_recover_valid),  // <- complete.rb_recover_valid
    .cs_retire_pc(cs_retire_pc),                      // <- complete.cs_retire_pc
    .fch_rec_enable(fch_rec_enable),                  // <- lsq_retire_mask.fch_rec_enable
    .rb_sq_stall(sq_stall_cache),
    .dispatch_index(new_rob_index),             // -> lsq_disp_mask.dispatch_idx
    .retire_entry(rob_retire_entry),            // -> lsq_retire_mask.rob_head_entry
    .rb_free_slots(rb_free_slots)
    //.rsb_struct_halt(rob_stall)                  // -> lsq_disp_mask.rob_stall
);

// to lsq_disp_mask
assign rob_stall = (rb_free_slots == 0) ? 3'b111 :
                   (rb_free_slots == 1) ? 3'b011 :
                   (rb_free_slots == 2) ? 3'b001 :
                   3'b000; 

//////////////////////////////////////////////////
//                                              //
//                Complete Stage                //
//                                              //
//////////////////////////////////////////////////

complete_stage cs(
    .clk(clk),
    .rst(rst | fch_rec_enable),
    .cs_fu_done_flags(fu_to_complete),                 // <- fu.cs_fu_done_flags
    .cs_fu_complete_pkts(cs_fu_complete_pkts),                          // <- fu.cs_fu_complete_pkts
    .cs_stall_mask(bs_hazard),                // -> fu.bs_hazard
    .cs_cdb_broadcast(cs_cdb_broadcast),                              // -> cs_cdb_broadcast broadcast
    .cs_wb_data(cs_wb_data),                        // -> cs_wb_data, to register file
    .cs_retire_valid(cs_retire_valid),            // -> SYS_ROB_ADDR_WIDTH.cs_retire_valid
    .cs_retire_idx(cs_retire_idx),            // -> SYS_ROB_ADDR_WIDTH.cs_retire_idx
    .cs_finish_valid(cs_finish_valid),
    .finish(finish),
    .cs_retire_pc(cs_retire_pc)                       // -> SYS_ROB_ADDR_WIDTH.cs_retire_pc
);

always_comb begin
    for (int i = 0; i < 3; i++) begin
	rb_recover_valid[i] = 1'b0;
        if (cs_finish_valid[i] && cs_fu_complete_pkts[finish[i]].if_take_branch) begin
                rb_recover_valid[i] = 1'b1;
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
    .rob_head_entry(rob_retire_entry),          // <- SYS_ROB_ADDR_WIDTH.retire_entry

    .fch_rec_enable(fch_rec_enable),                  // -> SYS_ROB_ADDR_WIDTH.fch_rec_enable, Freelist.fch_rec_enable, fetch.icache_branch
    .cs_retire_pc(bp_fetch_addr),                       // -> fetch.cs_retire_pc
    .mt_checkpoint_tbl(archi_maptable_out),        // <- arch map.mt_checkpoint_tbl
    .recover_maptable(mt_checkpoint_tbl),          // -> map table.mt_checkpoint_tbl

    .fl_free_count(fl_free_count),                  // <- Freelist.fl_free_count
    .fl_retire_en_mask(fl_retire_en_mask),                       // -> Freelist.fl_retire_en_mask
    .SQRetireEN(SQRetireEN),                    // -> SQ.lsq_retire_mask
    .halt(re_halt),
    .retired_inst_cnt(retired_inst_cnt)
);

    always_comb begin
        for (int i = 0; i < 3; i++) begin
            map_ar[i]     = rob_retire_entry[i].arch_reg;
            map_ar_pr[i]  = rob_retire_entry[i].Tnew;
            fl_retired_pr_list[i]  = rob_retire_entry[i].Told;
        end
    end

    // Internal next-state and rst values
    logic [31:0][`SYS_PHYS_REG-1:0] archi_maptable_reset;  // Initial mapping: AR[i] -> SYS_PHYS_REG[i]
    logic [31:0][`SYS_PHYS_REG-1:0] archi_maptable_next;   // Next-state value of mt_checkpoint_tbl

    // Compute default/rst state: each AR maps to its index
    always_comb begin : GenResetMap
        for (int i = 0; i < 32; i++) begin
            archi_maptable_reset[i] = i[`SYS_PHYS_REG-1:0];  // Truncate or extend to match SYS_PHYS_REG width
        end
    end

    // Sequential logic: update mt_checkpoint_tbl on clk edge
    always_ff @(posedge clk) begin : ArchMapRegister
        if (rst) begin
            archi_maptable_out <= `SYS_SMALL_DELAY archi_maptable_reset;  // Reset to identity mapping
        end else begin
            archi_maptable_out <= `SYS_SMALL_DELAY archi_maptable_next;   // Commit next-state updates
        end
    end

    // Combinational logic: compute next-state mapping
    always_comb begin : ComputeNextMap
        archi_maptable_next = archi_maptable_out;  // Start from current state

        // Process lsq_retire_mask instructions from oldest to youngest
        for (int i = 2; i >= 0; i--) begin
            if (fl_retire_en_mask[i]) begin
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
    .clk(clk), 
    .rst(rst), 
    .fl_dispatch_en_mask(dis_new_pr_en),                 
    .fl_retire_en_mask(fl_retire_en_mask),                        // <- lsq_retire_mask.fl_retire_en_mask
    .fl_retired_pr_list(fl_retired_pr_list),                      // <- lsq_retire_mask.fl_retired_pr_list
    .fch_rec_enable(fch_rec_enable),                  // <- lsq_retire_mask.fch_rec_enable
    .fl_allocated_pr_list(free_pr),                          // -> lsq_disp_mask.dispatch_free_prs 
    .fl_head_ptr(FreelistHead),                        // -> lsq_retire_mask.FreelistHead
    .fl_alloc_valid_mask(free_pr_valid),               // -> lsq_disp_mask.free_reg_valid 
    .fl_free_count(fl_free_count)                   // -> lsq_retire_mask.fl_free_count
);

//////////////////////////////////////////////////
//                                              //
//            Branch Predictor                  //
//                                              //
//////////////////////////////////////////////////

branch_predictor bp_0(
    .clk(clk), 
    .rst(rst), 
    .bs_upd_en(bs_upd_en), 
    .bs_upd_pc(bs_upd_pc), 
    .bs_upd_taken(bs_upd_taken), 
    .bs_upd_target(bs_upd_target), 
    .bp_disp_enable(bp_disp_enable), 
    .bp_disp_addr(bp_disp_addr),
    .bp_fetch_enable(bp_fetch_enable), 
    .bp_fetch_addr(bp_fetch_pc),
    .bp_pred_valid(bp_pred_valid), 
    .bp_pred_taken(predict_direction_next), 
    .bp_pred_target(predict_pc_next)
);


endmodule

`endif // __PIPELINE_V__

