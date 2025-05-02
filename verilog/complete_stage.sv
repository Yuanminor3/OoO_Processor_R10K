
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module complete_stage(
    input                               clk,
    input                               rst,
    input   FU_STATE_PACKET             cs_fu_done_flags,
    input   FU_COMPLETE_PACKET [7:0]    cs_fu_complete_pkts,    // 7: branch ... 0: alu_1

    output  FU_STATE_PACKET             cs_stall_mask, // lsq_stall_mask on complete hazard
    output  CDB_T_PACKET                cs_cdb_broadcast,      // destination pr
    output  logic [2:0][`SYS_XLEN-1:0]      cs_wb_data,
    output  logic [2:0]                 cs_retire_valid,
    output  logic [2:0][`SYS_ROB_ADDR_WIDTH-1:0]       cs_retire_idx,
    output  logic [2:0]                 cs_finish_valid,
    output  logic [2:0][`SYS_FU_ADDR_WIDTH:0]          finish,
    output  logic [2:0][`SYS_XLEN-1:0]      cs_retire_pc

);

    wire [7:0]      cs_pick_one, cs_pick_two, cs_pick_three;
    wire [7:0]      cs_done_mask12, cs_done_mask23;

    logic [2:0][`SYS_FU_ADDR_WIDTH:0]      finish_next;

    ps8 sel_1st(cs_fu_done_flags   , 1'b1, cs_pick_one, cs_req_mask_one);
    ps8 sel_2nd(cs_done_mask12, 1'b1, cs_pick_two, cs_req_mask_two);
    ps8 sel_3rd(cs_done_mask23, 1'b1, cs_pick_three, cs_req_mask_three);

    assign cs_done_mask12 = cs_fu_done_flags    & ~cs_pick_one;
    assign cs_done_mask23 = cs_done_mask12 & ~cs_pick_two;
    assign cs_stall_mask   = cs_done_mask23 & ~cs_pick_three;

    // pass through from cs_fu_complete_pkts to cdb 
    assign cs_cdb_broadcast.t2 = cs_finish_valid[2] ? cs_fu_complete_pkts[finish[2]].dispatch_allocated_prs : 0;
    assign cs_cdb_broadcast.t1 = cs_finish_valid[1] ? cs_fu_complete_pkts[finish[1]].dispatch_allocated_prs : 0;
    assign cs_cdb_broadcast.t0 = cs_finish_valid[0] ? cs_fu_complete_pkts[finish[0]].dispatch_allocated_prs : 0;

always_comb begin
    for (int i = 0; i < 3; i++) begin
 	cs_finish_valid[i] = (finish[i] < 4'd8);	
        cs_wb_data[i]    = cs_finish_valid[i] ? cs_fu_complete_pkts[finish[i]].dest_value  : '0;
    end
end

always_comb begin
    // 1. send completed entry back to rob
    for (int i = 0; i < 3; i++) begin
        cs_retire_valid[i]       = 1'b0;
        cs_retire_idx[i]       = '0;
        cs_retire_pc[i]            = '0;

        if (cs_finish_valid[i]) begin
            cs_retire_valid[i] = 1'b1;
            cs_retire_idx[i] = cs_fu_complete_pkts[finish[i]].rob_entry;

            if (cs_fu_complete_pkts[finish[i]].if_take_branch) begin
                cs_retire_pc[i] = cs_fu_complete_pkts[finish[i]].cs_retire_pc;
            end
        end
    end

    // 2. update the SYS_FU_ADDR_WIDTH to be completed next cycle
    finish_next = '{4'hF, 4'hF, 4'hF};
    for (int i = 0; i < 8; i++) begin
        if (cs_pick_one[i])
            finish_next[2] = i[3:0];
        if (cs_pick_two[i])
            finish_next[1] = i[3:0];
        if (cs_pick_three[i])
            finish_next[0] = i[3:0];
    end

end

    always_ff @(posedge clk) begin
        if (rst)
            finish <= `SYS_SMALL_DELAY {4'b1111, 4'b1111, 4'b1111};
        else
            finish <= `SYS_SMALL_DELAY finish_next;
    end 

endmodule

