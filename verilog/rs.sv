
`ifndef __RS_V__
`define __RS_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module RS(
    input                       clock,
    input                       reset,
    // from dispatch
    input RS_IN_PACKET [2:0]    rs_in,
    // from FU
    input FU_FIFO_PACKET        fu_fifo_stall,  // high if fu FIFO has < 3 available
    // from CDB
    input CDB_T_PACKET          cdb_t,
    // from LSQ
    input [2**`LSQ-1:0]         load_tail_ready,
    // to issue
    output RS_S_PACKET [2:0]    issue_insts,
    // to dispatch
    output logic [2:0]          struct_stall    // if high, stall corresponding dispatch, dependent on fu_req

);

/* The struct array that stores all RS entries */
RS_IN_PACKET [`RSW-1:0]        rs_entries;

/* select next entry to allocate */
logic [2:0][`RSW-1:0] new_entry;    // one hot coding
logic [`RSW-1:0] issue_EN;          // which entry to issue next

// 1. rs structural stall logic
logic [2:0] not_stall; 
logic [`RSW-1:0] entry_av, entry_av_after2, entry_av_after1;  // entry available; entry available after allocating rs_in[2] (and rs_in[1]) entry

always_comb 
    for(int i=0; i<`RSW; i++) begin
        entry_av[i] = ~rs_entries[i].valid;  // invalid (not in use) entry is available
    end
assign entry_av_after2 = entry_av & ~new_entry[2];  
assign entry_av_after1 = entry_av_after2 & ~new_entry[1]; 

ps16 sel_av2(.req(entry_av), .en(1'b1), .gnt(new_entry[2]), .req_up(not_stall[2]));  // if no bit selected (i.e. entry_av all 0, rs_entries all valid),
ps16 sel_av1(.req(entry_av_after2), .en(1'b1), .gnt(new_entry[1]), .req_up(not_stall[1]));  // req_up = 0, not_stall = 0, struct_stall = 1 (i.e. rs full)
ps16 sel_av0(.req(entry_av_after1), .en(1'b1), .gnt(new_entry[0]), .req_up(not_stall[0]));  // eg. 1111 1111 ... --> 1000 0000 ...; not_stall= 1; stall = 0 
assign struct_stall = ~not_stall;

// 2. CDB broadcast logic, RS updates reg1_ready or reg2_ready
logic [`RSW-1:0] reg1_ready_next;
logic [`RSW-1:0] reg2_ready_next;
always_comb begin
    for(int i=0; i<`RSW; i++)begin
        reg1_ready_next[i] = rs_entries[i].reg1_pr==cdb_t.t0 ||
                             rs_entries[i].reg1_pr==cdb_t.t1 ||
                             rs_entries[i].reg1_pr==cdb_t.t2 ? 
                             1'b1 : rs_entries[i].reg1_ready;
        reg2_ready_next[i] = rs_entries[i].reg2_pr==cdb_t.t0 ||
                             rs_entries[i].reg2_pr==cdb_t.t1 ||
                             rs_entries[i].reg2_pr==cdb_t.t2 ? 
                             1'b1 : rs_entries[i].reg2_ready;
    end
end

// 3. allocate logic, rs_entries_next state update
RS_IN_PACKET [`RSW-1:0] rs_entries_next;
always_comb begin
    for(int i=0; i < `RSW; i++) begin
        if (new_entry[2][i])
            rs_entries_next[i] = rs_in[2];
        else if (new_entry[1][i])
            rs_entries_next[i] = rs_in[1];
        else if (new_entry[0][i])
            rs_entries_next[i] = rs_in[0];
        else begin
            rs_entries_next[i] = rs_entries[i];  // unchange
            rs_entries_next[i].reg1_ready = reg1_ready_next[i];  
            rs_entries_next[i].reg2_ready = reg2_ready_next[i];
            if (issue_EN[i]) rs_entries_next[i].valid = 0;
        end
    end
end

always_ff @(posedge clock) begin
    if (reset)
    `ifndef IS_DEBUG
        rs_entries <= `SD 0; 
    `else
        rs_entries <= `SD rs_entries_debug;
    `endif
    else 
        rs_entries <= `SD rs_entries_next;
end

// 4. issue ready logic
logic [`RSW-1:0] tag_ready;
logic [`RSW-1:0] fu_ready;
logic [`RSW-1:0] entry_ready, entry_ready_rev;
logic [`RSW-1:0] entry_ready_rev_masked [2:0];
logic [2:0][`RSW-1:0] tag_issue_separate_rev;
logic [2:0][`RSW-1:0] tag_issue_separate;

// 4.1 tag ready: both src registers ready, entry valid, and LSQ ready
always_comb begin
    for (int i = 0; i < `RSW; i++) begin
        tag_ready[i] = reg1_ready_next[i] & reg2_ready_next[i] &
                       rs_entries[i].valid & load_tail_ready[rs_entries[i].sq_tail];
    end
end

// 4.2 FU ready: corresponding FU not stalled
always_comb begin
    fu_ready = '0;
    for (int i = 0; i < `RSW; i++) begin
        unique case (rs_entries[i].fu_sel)
            ALU_1   : fu_ready[i] = ~fu_fifo_stall.alu;
            LS_1    : fu_ready[i] = ~fu_fifo_stall.ls;
            MULT_1  : fu_ready[i] = ~fu_fifo_stall.mult;
            BRANCH  : fu_ready[i] = ~fu_fifo_stall.branch;
            default : fu_ready[i] = 1'b0;
        endcase
    end
end

// 4.3 entry_ready = tag_ready & fu_ready
assign entry_ready = tag_ready & fu_ready;
assign entry_ready_rev = {<<{entry_ready}};  // reverse for LSB-first priority

// 4.4 masking for 3-way issue
assign entry_ready_rev_masked[2] = entry_ready_rev;
assign entry_ready_rev_masked[1] = entry_ready_rev & ~tag_issue_separate_rev[2];
assign entry_ready_rev_masked[0] = entry_ready_rev_masked[1] & ~tag_issue_separate_rev[1];

// 4.5 priority selectors (LSB-first)
ps16 is_ps2 (.req(entry_ready_rev_masked[2]), .en(1'b1), .gnt(tag_issue_separate_rev[2]));
ps16 is_ps1 (.req(entry_ready_rev_masked[1]), .en(1'b1), .gnt(tag_issue_separate_rev[1]));
ps16 is_ps0 (.req(entry_ready_rev_masked[0]), .en(1'b1), .gnt(tag_issue_separate_rev[0]));


RS_IN_PACKET [2:0] issue_pckts;
always_comb begin
// 4.6 reverse selected tag back to normal order
    for (int i = 0; i < 3; i++) begin
        tag_issue_separate[i] = {<<{tag_issue_separate_rev[i]}};
    end

// 5. issue logicalways_comb begin
    issue_pckts = '{default: '0};
    for (int j = 0; j <= 2; j++) begin
        for (int i = 0; i < `RSW; i++) begin
            if (tag_issue_separate[j][i])
                issue_pckts[j] = rs_entries[i];
        end
    end
end

`ifdef RS_ALLOCATE_DEBUG
    assign issue_EN = 0;
`else
    assign issue_EN = tag_issue_separate[0] | tag_issue_separate[1] | tag_issue_separate[2];
`endif

// assign output issue packet
always_comb begin
    for(int i=0; i<3; i++)begin
        issue_insts[i].valid = issue_pckts[i].valid;
        issue_insts[i].fu_sel = issue_pckts[i].fu_sel;
        issue_insts[i].op_sel = issue_pckts[i].op_sel;
        issue_insts[i].NPC = issue_pckts[i].NPC;
        issue_insts[i].PC = issue_pckts[i].PC;
        issue_insts[i].opa_select = issue_pckts[i].opa_select;
        issue_insts[i].opb_select = issue_pckts[i].opb_select;
        issue_insts[i].inst = issue_pckts[i].inst;
        issue_insts[i].halt = issue_pckts[i].halt;
        issue_insts[i].rob_entry = issue_pckts[i].rob_entry;
        issue_insts[i].sq_tail = issue_pckts[i].sq_tail;
        issue_insts[i].dest_pr = issue_pckts[i].dest_pr;
        issue_insts[i].reg1_pr = issue_pckts[i].reg1_pr;
        issue_insts[i].reg2_pr = issue_pckts[i].reg2_pr;
    end
end

endmodule

`endif

