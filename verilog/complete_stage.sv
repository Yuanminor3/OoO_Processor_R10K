`define TEST_MODE
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module complete_stage(
    input                               clock,
    input                               reset,
    input   FU_STATE_PACKET             fu_finish,
    input   FU_COMPLETE_PACKET [7:0]    fu_c_in,    // 7: branch ... 0: alu_1

    output  FU_STATE_PACKET             fu_c_stall, // stall on complete hazard
    output  CDB_T_PACKET                cdb_t,      // destination pr
    output  logic [2:0][`XLEN-1:0]      wb_value,
    output  logic [2:0]                 complete_valid,
    output  logic [2:0][`ROB-1:0]       complete_entry,
    output  logic [2:0]                 precise_state_valid,
    output  logic [2:0][`XLEN-1:0]      target_pc

    `ifdef TEST_MODE
    , output FU_COMPLETE_PACKET [2:0]   complete_pckt_in_display
    `endif 
);

    wire [7:0]      sel_1, sel_2, sel_3;
    wire [7:0]      fu_finish_12, fu_finish_23;
    logic [2:0]      finish_valid;

    logic [2:0][`FU:0]      finish_next;
    logic [2:0][`FU:0]      finish;

    ps8 sel_1st(fu_finish   , 1'b1, sel_1, rq1w);
    ps8 sel_2nd(fu_finish_12, 1'b1, sel_2, rq2w);
    ps8 sel_3rd(fu_finish_23, 1'b1, sel_3, rq3w);

    assign fu_finish_12 = fu_finish    & ~sel_1;
    assign fu_finish_23 = fu_finish_12 & ~sel_2;
    assign fu_c_stall   = fu_finish_23 & ~sel_3;

    // pass through from fu_c_in to cdb 
    assign cdb_t.t2 = finish_valid[2] ? fu_c_in[finish[2]].dest_pr : 0;
    assign cdb_t.t1 = finish_valid[1] ? fu_c_in[finish[1]].dest_pr : 0;
    assign cdb_t.t0 = finish_valid[0] ? fu_c_in[finish[0]].dest_pr : 0;

always_comb begin
    for (int i = 0; i < 3; i++) begin
 	finish_valid[i] = (finish[i] < 4'd8);	
        wb_value[i]    = finish_valid[i] ? fu_c_in[finish[i]].dest_value  : '0;
    end
end

always_comb begin
    // 1. send completed entry back to rob
    for (int i = 0; i < 3; i++) begin
        complete_valid[i]       = 1'b0;
        complete_entry[i]       = '0;
        precise_state_valid[i]  = 1'b0;
        target_pc[i]            = '0;

        if (finish_valid[i]) begin
            complete_valid[i] = 1'b1;
            complete_entry[i] = fu_c_in[finish[i]].rob_entry;

            if (fu_c_in[finish[i]].if_take_branch) begin
                precise_state_valid[i] = 1'b1;
                target_pc[i] = fu_c_in[finish[i]].target_pc;
            end
        end
    end

    // 2. update the FU to be completed next cycle
    finish_next = '{4'hF, 4'hF, 4'hF};
    for (int i = 0; i < 8; i++) begin
        if (sel_1[i])
            finish_next[2] = i[3:0];
        if (sel_2[i])
            finish_next[1] = i[3:0];
        if (sel_3[i])
            finish_next[0] = i[3:0];
    end

end

    always_ff @(posedge clock) begin
        if (reset)
            finish <= `SD {4'b1111, 4'b1111, 4'b1111};
        else
            finish <= `SD finish_next;
    end


    `ifdef TEST_MODE
    assign complete_pckt_in_display[2].if_take_branch = precise_state_valid[2];
    assign complete_pckt_in_display[2].valid = complete_valid[2];
    assign complete_pckt_in_display[2].halt = finish_valid[2] ? fu_c_in[finish[2]].halt : 0;
    assign complete_pckt_in_display[2].target_pc = target_pc[2];
    assign complete_pckt_in_display[2].dest_pr = cdb_t.t2;
    assign complete_pckt_in_display[2].dest_value = wb_value[2];
    assign complete_pckt_in_display[2].rob_entry = complete_entry[2];
    
    assign complete_pckt_in_display[1].if_take_branch = precise_state_valid[1];
    assign complete_pckt_in_display[1].valid = complete_valid[1];
    assign complete_pckt_in_display[1].halt = finish_valid[1] ? fu_c_in[finish[1]].halt : 0;
    assign complete_pckt_in_display[1].target_pc = target_pc[1];
    assign complete_pckt_in_display[1].dest_pr = cdb_t.t1;
    assign complete_pckt_in_display[1].dest_value = wb_value[1];
    assign complete_pckt_in_display[1].rob_entry = complete_entry[1];
    
    assign complete_pckt_in_display[0].if_take_branch = precise_state_valid[0];
    assign complete_pckt_in_display[0].valid = complete_valid[0];
    assign complete_pckt_in_display[0].halt = finish_valid[0] ? fu_c_in[finish[0]].halt : 0;
    assign complete_pckt_in_display[0].target_pc = target_pc[0];
    assign complete_pckt_in_display[0].dest_pr = cdb_t.t0;
    assign complete_pckt_in_display[0].dest_value = wb_value[0];
    assign complete_pckt_in_display[0].rob_entry = complete_entry[0];
    `endif 

endmodule

