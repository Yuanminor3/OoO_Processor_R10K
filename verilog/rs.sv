
`ifndef __RS_V__
`define __RS_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module RS(
    input                       clk,
    input                       rst,
    // from lsq_disp_mask
    input RS_IN_PACKET [2:0]    dispatch_rs_pkts,
    // from SYS_FU_ADDR_WIDTH
    input FU_FIFO_PACKET        fu_fifo_stall,  // high if fu FIFO has < 3 available
    // from CDB
    input CDB_T_PACKET          cs_cdb_broadcast,
    // from SYS_LSQ_ADDR_WIDTH
    input [2**`SYS_LSQ_ADDR_WIDTH-1:0]         rsb_sq_ready_flags,
    // to issue
    output RS_S_PACKET [2:0]    rsb_issue_packets,
    // to lsq_disp_mask
    output logic [2:0]          rsb_struct_halt    // if high, lsq_stall_mask corresponding lsq_disp_mask, dependent on fu_rcmp_eq_flag

);

/* The struct fl_pr_stack that stores all RS entries */
RS_IN_PACKET [`SYS_RS_ADDR_WIDTH-1:0]        rs_entries;

/* select next entry to allocate */
logic [2:0][`SYS_RS_ADDR_WIDTH-1:0] rsb_allocate_mask;    // one hot coding
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_issue_dequeue;          // which entry to issue next

// 1. rs structural lsq_stall_mask logic
logic [2:0] rsb_not_halted; 
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_spare_slots, rsb_spare_after_head2, rsb_spare_after_head1;  // entry available; entry available after allocating dispatch_rs_pkts[2] (and dispatch_rs_pkts[1]) entry

always_comb 
    for(int i=0; i<`SYS_RS_ADDR_WIDTH; i++) begin
        rsb_spare_slots[i] = ~rs_entries[i].valid;  // invalid (not in use) entry is available
    end
assign rsb_spare_after_head2 = rsb_spare_slots & ~rsb_allocate_mask[2];  
assign rsb_spare_after_head1 = rsb_spare_after_head2 & ~rsb_allocate_mask[1]; 

ps16 rsb_sel_free2(.rcmp_eq_flag(rsb_spare_slots), .en(1'b1), .gnt(rsb_allocate_mask[2]), .req_up(rsb_not_halted[2]));  // if no bit selected (i.e. rsb_spare_slots all 0, rs_entries all valid),
ps16 rsb_sel_free1(.rcmp_eq_flag(rsb_spare_after_head2), .en(1'b1), .gnt(rsb_allocate_mask[1]), .req_up(rsb_not_halted[1]));  // req_up = 0, rsb_not_halted = 0, rsb_struct_halt = 1 (i.e. rs fl_buffer_full)
ps16 rsb_sel_free0(.rcmp_eq_flag(rsb_spare_after_head1), .en(1'b1), .gnt(rsb_allocate_mask[0]), .req_up(rsb_not_halted[0]));  // eg. 1111 1111 ... --> 1000 0000 ...; rsb_not_halted= 1; lsq_stall_mask = 0 
assign rsb_struct_halt = ~rsb_not_halted;

// 2. CDB broadcast logic, RS updates dispatch_src1_rdy or dispatch_src2_rdy
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_rdy1_next;
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_rdy2_next;
always_comb begin
    for(int i=0; i<`SYS_RS_ADDR_WIDTH; i++)begin
        rsb_rdy1_next[i] = rs_entries[i].dispatch_src1_pr==cs_cdb_broadcast.t0 ||
                             rs_entries[i].dispatch_src1_pr==cs_cdb_broadcast.t1 ||
                             rs_entries[i].dispatch_src1_pr==cs_cdb_broadcast.t2 ? 
                             1'b1 : rs_entries[i].dispatch_src1_rdy;
        rsb_rdy2_next[i] = rs_entries[i].dispatch_src2_pr==cs_cdb_broadcast.t0 ||
                             rs_entries[i].dispatch_src2_pr==cs_cdb_broadcast.t1 ||
                             rs_entries[i].dispatch_src2_pr==cs_cdb_broadcast.t2 ? 
                             1'b1 : rs_entries[i].dispatch_src2_rdy;
    end
end

// 3. allocate logic, rsb_entry_nxt state update
RS_IN_PACKET [`SYS_RS_ADDR_WIDTH-1:0] rsb_entry_nxt;
always_comb begin
    for(int i=0; i < `SYS_RS_ADDR_WIDTH; i++) begin
        if (rsb_allocate_mask[2][i])
            rsb_entry_nxt[i] = dispatch_rs_pkts[2];
        else if (rsb_allocate_mask[1][i])
            rsb_entry_nxt[i] = dispatch_rs_pkts[1];
        else if (rsb_allocate_mask[0][i])
            rsb_entry_nxt[i] = dispatch_rs_pkts[0];
        else begin
            rsb_entry_nxt[i] = rs_entries[i];  // unchange
            rsb_entry_nxt[i].dispatch_src1_rdy = rsb_rdy1_next[i];  
            rsb_entry_nxt[i].dispatch_src2_rdy = rsb_rdy2_next[i];
            if (rsb_issue_dequeue[i]) rsb_entry_nxt[i].valid = 0;
        end
    end
end

always_ff @(posedge clk) begin
    if (rst)
    `ifndef IS_DEBUG
        rs_entries <= `SYS_SMALL_DELAY 0; 
    `else
        rs_entries <= `SYS_SMALL_DELAY rs_entries_debug;
    `endif
    else 
        rs_entries <= `SYS_SMALL_DELAY rsb_entry_nxt;
end

// 4. issue ready logic
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_id_ready;
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_fu_ready;
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_exec_ready_mask, rsb_exec_ready_mask_rev;
logic [`SYS_RS_ADDR_WIDTH-1:0] rsb_ready__rev_masked [2:0];
logic [2:0][`SYS_RS_ADDR_WIDTH-1:0] rsb_gnt_rev;
logic [2:0][`SYS_RS_ADDR_WIDTH-1:0] rsb_gnt;

// 4.1 tag ready: both src registers ready, entry valid, and SYS_LSQ_ADDR_WIDTH ready
always_comb begin
    for (int i = 0; i < `SYS_RS_ADDR_WIDTH; i++) begin
        rsb_id_ready[i] = rsb_rdy1_next[i] & rsb_rdy2_next[i] &
                       rs_entries[i].valid & rsb_sq_ready_flags[rs_entries[i].sq_tail];
    end
end

// 4.2 SYS_FU_ADDR_WIDTH ready: corresponding SYS_FU_ADDR_WIDTH not stalled
always_comb begin
    rsb_fu_ready = '0;
    for (int i = 0; i < `SYS_RS_ADDR_WIDTH; i++) begin
        unique case (rs_entries[i].dec_fu_unit_sel)
            ALU_1   : rsb_fu_ready[i] = ~fu_fifo_stall.alu;
            LS_1    : rsb_fu_ready[i] = ~fu_fifo_stall.ls;
            MULT_1  : rsb_fu_ready[i] = ~fu_fifo_stall.mult;
            BRANCH  : rsb_fu_ready[i] = ~fu_fifo_stall.branch;
            default : rsb_fu_ready[i] = 1'b0;
        endcase
    end
end

// 4.3 rsb_exec_ready_mask = rsb_id_ready & rsb_fu_ready
assign rsb_exec_ready_mask = rsb_id_ready & rsb_fu_ready;
assign rsb_exec_ready_mask_rev = {<<{rsb_exec_ready_mask}};  // reverse for LSB-first priority

// 4.4 masking for 3-way issue
assign rsb_ready__rev_masked[2] = rsb_exec_ready_mask_rev;
assign rsb_ready__rev_masked[1] = rsb_exec_ready_mask_rev & ~rsb_gnt_rev[2];
assign rsb_ready__rev_masked[0] = rsb_ready__rev_masked[1] & ~rsb_gnt_rev[1];

// 4.5 priority selectors (LSB-first)
ps16 rsb_sel_issue2 (.rcmp_eq_flag(rsb_ready__rev_masked[2]), .en(1'b1), .gnt(rsb_gnt_rev[2]));
ps16 rsb_sel_issue1 (.rcmp_eq_flag(rsb_ready__rev_masked[1]), .en(1'b1), .gnt(rsb_gnt_rev[1]));
ps16 rsb_sel_issue0 (.rcmp_eq_flag(rsb_ready__rev_masked[0]), .en(1'b1), .gnt(rsb_gnt_rev[0]));


RS_IN_PACKET [2:0] rsb_issue_tmp;
always_comb begin
// 4.6 reverse selected tag back to normal order
    for (int i = 0; i < 3; i++) begin
        rsb_gnt[i] = {<<{rsb_gnt_rev[i]}};
    end

// 5. issue logicalways_comb begin
    rsb_issue_tmp = '{default: '0};
    for (int j = 0; j <= 2; j++) begin
        for (int i = 0; i < `SYS_RS_ADDR_WIDTH; i++) begin
            if (rsb_gnt[j][i])
                rsb_issue_tmp[j] = rs_entries[i];
        end
    end
end

`ifdef RS_ALLOCATE_DEBUG
    assign rsb_issue_dequeue = 0;
`else
    assign rsb_issue_dequeue = rsb_gnt[0] | rsb_gnt[1] | rsb_gnt[2];
`endif

// assign output issue packet
always_comb begin
    for(int i=0; i<3; i++)begin
        rsb_issue_packets[i].valid = rsb_issue_tmp[i].valid;
        rsb_issue_packets[i].dec_fu_unit_sel = rsb_issue_tmp[i].dec_fu_unit_sel;
        rsb_issue_packets[i].dec_fu_opcode = rsb_issue_tmp[i].dec_fu_opcode;
        rsb_issue_packets[i].NPC = rsb_issue_tmp[i].NPC;
        rsb_issue_packets[i].PC = rsb_issue_tmp[i].PC;
        rsb_issue_packets[i].dec_operandA_mux = rsb_issue_tmp[i].dec_operandA_mux;
        rsb_issue_packets[i].dec_operandB_mux = rsb_issue_tmp[i].dec_operandB_mux;
        rsb_issue_packets[i].inst = rsb_issue_tmp[i].inst;
        rsb_issue_packets[i].halt = rsb_issue_tmp[i].halt;
        rsb_issue_packets[i].rob_entry = rsb_issue_tmp[i].rob_entry;
        rsb_issue_packets[i].sq_tail = rsb_issue_tmp[i].sq_tail;
        rsb_issue_packets[i].dispatch_allocated_prs = rsb_issue_tmp[i].dispatch_allocated_prs;
        rsb_issue_packets[i].dispatch_src1_pr = rsb_issue_tmp[i].dispatch_src1_pr;
        rsb_issue_packets[i].dispatch_src2_pr = rsb_issue_tmp[i].dispatch_src2_pr;
    end
end

endmodule

`endif

