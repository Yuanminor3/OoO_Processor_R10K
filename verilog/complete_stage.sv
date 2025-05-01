
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
    output  logic [2:0]                 finish_valid,
    output  logic [2:0][`FU:0]          finish,
    output  logic [2:0][`XLEN-1:0]      target_pc

);

    wire [7:0]      sel_1, sel_2, sel_3;
    wire [7:0]      fu_finish_12, fu_finish_23;

    logic [2:0][`FU:0]      finish_next;

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
        target_pc[i]            = '0;

        if (finish_valid[i]) begin
            complete_valid[i] = 1'b1;
            complete_entry[i] = fu_c_in[finish[i]].rob_entry;

            if (fu_c_in[finish[i]].if_take_branch) begin
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

endmodule

