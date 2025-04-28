`define TEST_MODE
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/fu_alu.sv"
`include "verilog/fu_load.sv"
`include "verilog/fu_mult.sv"
`include "verilog/branch_fu.sv"

module execution_stage(
    input                               clock,
    input                               reset,
    // all fu I/O
    input FU_STATE_PACKET		complete_stall,
    input ISSUE_FU_PACKET [2**`FU-1:0]  fu_packet_in,
    output FU_STATE_PACKET               fu_ready,
    output FU_STATE_PACKET               fu_finish,
    output FU_COMPLETE_PACKET  [2**`FU-1:0]  fu_c_packet,
    // alu to sq
    output [2:0]                       exe_valid,
    output SQ_ENTRY_PACKET [2:0]       exe_store,
    output [2:0][`LSQ-1:0]             exe_idx,
    // fu_load <-> sq
    input SQ_LOAD_PACKET [1:0]         load_forward,
    output LOAD_SQ_PACKET [1:0]        load_lookup,
    // fu_load <-> dcache
    output [1:0][`XLEN-1:0]      cache_read_addr,
    output [1:0]		 cache_read_start,
    input [1:0][`XLEN-1:0]       ld_data,
    input  [1:0]                 is_hit,
    input  [1:0]                 broadcast_fu,
    input  [`XLEN-1:0]           broadcast_data,
    // fu_branch to bp
    output 			 update_EN,
    output [`XLEN-1:0]           update_pc,
    output 			 update_direction,
    output [`XLEN-1:0]           update_target
);

fu_alu fu_alu_1(
    .clock(clock),                             // system clock
    .reset(reset),               	       // system reset
    .complete_stall(complete_stall[ALU_1]),    // <- complete.fu_c_stall
    .fu_packet_in(fu_packet_in[ALU_1]),        // <- issue.issue_2_fu
    .fu_ready(fu_ready.alu_1),                 // -> issue.fu_ready
    .want_to_complete(fu_finish.alu_1),        // -> complete.fu_finish
    .fu_packet_out(fu_c_packet[ALU_1]),         

    // STORE
    .if_store(exe_valid[0]),
    .store_pckt(exe_store[0]),
    .sq_idx(exe_idx[0])
);

fu_alu fu_alu_2(
    .clock(clock),                             // system clock
    .reset(reset),               	       // system reset
    .complete_stall(complete_stall[ALU_2]),    // <- complete.fu_c_stall
    .fu_packet_in(fu_packet_in[ALU_2]),        // <- issue.issue_2_fu
    .fu_ready(fu_ready.alu_2),                 // -> issue.fu_ready
    .want_to_complete(fu_finish.alu_2),        // -> complete.fu_finish
    .fu_packet_out(fu_c_packet[ALU_2]),          

    // STORE
    .if_store(exe_valid[1]),
    .store_pckt(exe_store[1]),
    .sq_idx(exe_idx[1])
);

fu_alu fu_alu_3(
    .clock(clock),                             // system clock
    .reset(reset),               	       // system reset
    .complete_stall(complete_stall[ALU_3]),    // <- complete.fu_c_stall
    .fu_packet_in(fu_packet_in[ALU_3]),        // <- issue.issue_2_fu
    .fu_ready(fu_ready.alu_3),                 // -> issue.fu_ready
    .want_to_complete(fu_finish.alu_3),        // -> complete.fu_finish
    .fu_packet_out(fu_c_packet[ALU_3]),        // -> complete.fu_c_in

    // STORE
    .if_store(exe_valid[2]),
    .store_pckt(exe_store[2]),
    .sq_idx(exe_idx[2])
);

fu_mult fu_mult_1(
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall.mult_1),
    .fu_packet_in(fu_packet_in[MULT_1]),
    .fu_ready(fu_ready.mult_1),
    .want_to_complete(fu_finish.mult_1),
    .fu_packet_out(fu_c_packet[MULT_1])
);

fu_mult fu_mult_2(
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall.mult_2),
    .fu_packet_in(fu_packet_in[MULT_2]),
    .fu_ready(fu_ready.mult_2),
    .want_to_complete(fu_finish.mult_2),
    .fu_packet_out(fu_c_packet[MULT_2])
);

fu_load fu_load_1(
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall.loadstore_1),
    .fu_packet_in(fu_packet_in[LS_1]),

    // output
    .fu_ready(fu_ready.loadstore_1),
    .want_to_complete(fu_finish.loadstore_1),
    .fu_packet_out(fu_c_packet[LS_1]),

    // SQ
    .sq_lookup(load_lookup[0]),    // -> SQ.load_lookup
    .sq_result(load_forward[0]),   // <- SQ.load_forward

    // Cache
    .addr(cache_read_addr[0]),      // -> dcache 
    .cache_data_in(ld_data[0]), // <- dcache 
    .cache_read_EN(cache_read_start[0]),
    .is_hit(is_hit[0]),
    .broadcast_en(broadcast_fu[0]),
    .broadcast_data(broadcast_data)
);

fu_load fu_load_2(
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall.loadstore_2),
    .fu_packet_in(fu_packet_in[LS_2]),

    // output
    .fu_ready(fu_ready.loadstore_2),
    .want_to_complete(fu_finish.loadstore_2),
    .fu_packet_out(fu_c_packet[LS_2]),

    // SQ
    .sq_lookup(load_lookup[1]),    // -> SQ.load_lookup
    .sq_result(load_forward[1]),   // <- SQ.load_forward

    // Cache
    .addr(cache_read_addr[1]),      // -> dcache 
    .cache_data_in(ld_data[1]),     // <- dcache 
    .cache_read_EN(cache_read_start[1]),
    .is_hit(is_hit[1]),
    .broadcast_en(broadcast_fu[1]),
    .broadcast_data(broadcast_data)
);

branch_stage branc(
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall.branch),
    .fu_packet_in(fu_packet_in[BRANCH]),
    .fu_ready(fu_ready.branch),
    .want_to_complete_branch(fu_finish.branch),
    .fu_packet_out(fu_c_packet[BRANCH]),
    .update_EN(update_EN), 
    .update_pc(update_pc), 
    .update_direction(update_direction),
    .update_target(update_target)
);

endmodule
