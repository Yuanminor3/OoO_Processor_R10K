
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/fu_alu.sv"
`include "verilog/fu_load.sv"
`include "verilog/fu_mult.sv"
`include "verilog/branch_fu.sv"

module execution_stage(
    input                               clk,
    input                               rst,
    // all fu I/O
    input FU_STATE_PACKET		bs_hazard,
    input ISSUE_FU_PACKET [2**`SYS_FU_ADDR_WIDTH-1:0]  bs_in_pkt,
    output FU_STATE_PACKET               rsb_fu_ready,
    output FU_STATE_PACKET               cs_fu_done_flags,
    output FU_COMPLETE_PACKET  [2**`SYS_FU_ADDR_WIDTH-1:0]  exs_fu_completion_pkts,
    // alu to sq
    output [2:0]                       exs_store_flags,
    output SQ_ENTRY_PACKET [2:0]       exe_store_entries,
    output [2:0][`SYS_LSQ_ADDR_WIDTH-1:0]             exs_store_index,
    // fu_load <-> sq
    input SQ_LOAD_PACKET [1:0]         exs_load2pkts,
    output LOAD_SQ_PACKET [1:0]        exs_load_req_pkts,
    // fu_load <-> dcache
    output [1:0][`SYS_XLEN-1:0]      exs_dcache_req_addr,
    output [1:0]		 exs_dcache_req_valid,
    input [1:0][`SYS_XLEN-1:0]       exs_dcache_resp_data,
    input  [1:0]                 exs_dcache_hit_flags,
    input  [1:0]                 exs_dcache_brdcast_mask,
    input  [`SYS_XLEN-1:0]           exs_dcache_brdcast_data,
    // fu_branch to bp
    output 			 bs_upd_en,
    output [`SYS_XLEN-1:0]           bs_upd_pc,
    output 			 bs_upd_taken,
    output [`SYS_XLEN-1:0]           bs_upd_target
);

fu_alu fu_alu_1(
    .clk(clk),                             // system clk
    .rst(rst),               	       // system rst
    .bs_hazard(bs_hazard[ALU_1]),    // <- complete.cs_stall_mask
    .bs_in_pkt(bs_in_pkt[ALU_1]),        // <- issue.issue_2_fu
    .rsb_fu_ready(rsb_fu_ready.alu_1),                 // -> issue.rsb_fu_ready
    .fum_complete_req(cs_fu_done_flags.alu_1),        // -> complete.cs_fu_done_flags
    .bs_out_pkt(exs_fu_completion_pkts[ALU_1]),         

    // STORE
    .alu_store_enable(exs_store_flags[0]),
    .alu_store_output_entry(exe_store_entries[0]),
    .alu_store_sq_index(exs_store_index[0])
);

fu_alu fu_alu_2(
    .clk(clk),                             // system clk
    .rst(rst),               	       // system rst
    .bs_hazard(bs_hazard[ALU_2]),    // <- complete.cs_stall_mask
    .bs_in_pkt(bs_in_pkt[ALU_2]),        // <- issue.issue_2_fu
    .rsb_fu_ready(rsb_fu_ready.alu_2),                 // -> issue.rsb_fu_ready
    .fum_complete_req(cs_fu_done_flags.alu_2),        // -> complete.cs_fu_done_flags
    .bs_out_pkt(exs_fu_completion_pkts[ALU_2]),          

    // STORE
    .alu_store_enable(exs_store_flags[1]),
    .alu_store_output_entry(exe_store_entries[1]),
    .alu_store_sq_index(exs_store_index[1])
);

fu_alu fu_alu_3(
    .clk(clk),                             // system clk
    .rst(rst),               	       // system rst
    .bs_hazard(bs_hazard[ALU_3]),    // <- complete.cs_stall_mask
    .bs_in_pkt(bs_in_pkt[ALU_3]),        // <- issue.issue_2_fu
    .rsb_fu_ready(rsb_fu_ready.alu_3),                 // -> issue.rsb_fu_ready
    .fum_complete_req(cs_fu_done_flags.alu_3),        // -> complete.cs_fu_done_flags
    .bs_out_pkt(exs_fu_completion_pkts[ALU_3]),        // -> complete.cs_fu_complete_pkts

    // STORE
    .alu_store_enable(exs_store_flags[2]),
    .alu_store_output_entry(exe_store_entries[2]),
    .alu_store_sq_index(exs_store_index[2])
);

fu_mult fu_mult_1(
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard.mult_1),
    .bs_in_pkt(bs_in_pkt[MULT_1]),
    .rsb_fu_ready(rsb_fu_ready.mult_1),
    .fum_complete_req(cs_fu_done_flags.mult_1),
    .bs_out_pkt(exs_fu_completion_pkts[MULT_1])
);

fu_mult fu_mult_2(
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard.mult_2),
    .bs_in_pkt(bs_in_pkt[MULT_2]),
    .rsb_fu_ready(rsb_fu_ready.mult_2),
    .fum_complete_req(cs_fu_done_flags.mult_2),
    .bs_out_pkt(exs_fu_completion_pkts[MULT_2])
);

fu_load fu_load_1(
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard.loadstore_1),
    .bs_in_pkt(bs_in_pkt[LS_1]),

    // output
    .rsb_fu_ready(rsb_fu_ready.loadstore_1),
    .fum_complete_req(cs_fu_done_flags.loadstore_1),
    .bs_out_pkt(exs_fu_completion_pkts[LS_1]),

    // SQ
    .ld_sq_request(exs_load_req_pkts[0]),    // -> SQ.exs_load_req_pkts
    .ld_sq_response(exs_load2pkts[0]),   // <- SQ.exs_load2pkts

    // Cache
    .addr(exs_dcache_req_addr[0]),      // -> dcache 
    .ld_cache_data_in(exs_dcache_resp_data[0]), // <- dcache 
    .ld_cache_read_enable(exs_dcache_req_valid[0]),
    .exs_dcache_hit_flags(exs_dcache_hit_flags[0]),
    .broadcast_en(exs_dcache_brdcast_mask[0]),
    .exs_dcache_brdcast_data(exs_dcache_brdcast_data)
);

fu_load fu_load_2(
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard.loadstore_2),
    .bs_in_pkt(bs_in_pkt[LS_2]),

    // output
    .rsb_fu_ready(rsb_fu_ready.loadstore_2),
    .fum_complete_req(cs_fu_done_flags.loadstore_2),
    .bs_out_pkt(exs_fu_completion_pkts[LS_2]),

    // SQ
    .ld_sq_request(exs_load_req_pkts[1]),    // -> SQ.exs_load_req_pkts
    .ld_sq_response(exs_load2pkts[1]),   // <- SQ.exs_load2pkts

    // Cache
    .addr(exs_dcache_req_addr[1]),      // -> dcache 
    .ld_cache_data_in(exs_dcache_resp_data[1]),     // <- dcache 
    .ld_cache_read_enable(exs_dcache_req_valid[1]),
    .exs_dcache_hit_flags(exs_dcache_hit_flags[1]),
    .broadcast_en(exs_dcache_brdcast_mask[1]),
    .exs_dcache_brdcast_data(exs_dcache_brdcast_data)
);

branch_stage branc(
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard.branch),
    .bs_in_pkt(bs_in_pkt[BRANCH]),
    .rsb_fu_ready(rsb_fu_ready.branch),
    .bs_fire(cs_fu_done_flags.branch),
    .bs_out_pkt(exs_fu_completion_pkts[BRANCH]),
    .bs_upd_en(bs_upd_en), 
    .bs_upd_pc(bs_upd_pc), 
    .bs_upd_taken(bs_upd_taken),
    .bs_upd_target(bs_upd_target)
);

endmodule
