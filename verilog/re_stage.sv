`include "verilog/sys_defs.svh"

module retire_stage (
    input           [`SYS_ROB_ADDR_WIDTH-1:0]              fl_free_count,         // Distance in SYS_ROB_ADDR_WIDTH for reg-write instructions
    output  logic                           fch_rec_enable,         // Enable signal for branch predictor recovery
    output  logic   [`SYS_XLEN-1:0]             cs_retire_pc,           // Target PC for recovery
    input           ROB_ENTRY_PACKET[2:0]   rob_head_entry,      // SYS_ROB_ADDR_WIDTH fl_head_reg entries (3 in-order lsq_retire_mask entries)
    
    input           [31:0][`SYS_PHYS_REG-1:0]         mt_checkpoint_tbl,      // Architectural map table from AMT

    output  logic   [31:0][`SYS_PHYS_REG-1:0]         recover_maptable,    // Recovered architectural map table

    output  logic   [2:0]                   fl_retire_en_mask,           // Retire enable signals for 3 entries
    output  logic   [2:0]                   SQRetireEN,          // Store queue lsq_retire_mask enable signals

    output  logic                           halt,                // Halt signal
    output  logic   [2:0]	            retired_inst_cnt
);

// When the branch prediction is wrong, the recovery distance (fl_recover_dis) adjusts the free list header pointer to roll back to the correct physical register allocation state. 

// Depending on whether the retirement instruction is written to the architecture register (arch_reg != 0), progressively reducing the recovery distance (fl_recover_dis), which represents the number of physical registers to be freed
// Intermediate rollback distance through stages
logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_recover_dis;
logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_recover_dis_stage1, fl_recover_dis_stage2, fl_recover_dis_stage3;
logic [2:0] retire_valid;

// Calculate rollback distance if entry writes to AR
assign fl_recover_dis_stage1 = rob_head_entry[2].arch_reg==5'b0 ? fl_free_count : fl_free_count - 1;
assign fl_recover_dis_stage2 = rob_head_entry[1].arch_reg==5'b0 ? fl_recover_dis_stage1 : fl_recover_dis_stage1 - 1;
assign fl_recover_dis_stage3 = rob_head_entry[0].arch_reg==5'b0 ? fl_recover_dis_stage2 : fl_recover_dis_stage2 - 1;

// Write flag bits for each entry
logic [2:0] is_write_bit;

// Determine write-back status (write to AR ≠ x0)
always_comb begin
    for (int i = 0; i < 3; i++) begin
        is_write_bit[i] = (rob_head_entry[i].arch_reg != 5'b0);
    end
end

assign retired_inst_cnt = retire_valid;

logic stop_retire;
always_comb begin
    fl_retire_en_mask         = 3'b000;
    SQRetireEN        = 3'b000;
    fch_rec_enable       = 1'b0;
    retire_valid      = 3'b000;
    halt              = 1'b0;
    fl_recover_dis    = fl_free_count;
    recover_maptable  = mt_checkpoint_tbl;
    cs_retire_pc         = 32'd0;
    stop_retire = 1'b0;
    // Flags to check if we should continue retiring

    for (int i = 2; i >= 0; i--) begin
        if (!stop_retire && rob_head_entry[i].completed) begin
            retire_valid[i] = 1'b1;
            fl_retire_en_mask[i]    = (i == 2) ? is_write_bit[2] :
                              (i == 1) ? is_write_bit[1] :
                                         is_write_bit[0];

            SQRetireEN[i]   = rob_head_entry[i].dec_is_store_flag;
            halt            = rob_head_entry[i].halt;

            recover_maptable[rob_head_entry[i].arch_reg] = rob_head_entry[i].Tnew;

            if (rob_head_entry[i].precise_state_need) begin
                fch_rec_enable    = 1'b1;
                cs_retire_pc      = rob_head_entry[i].cs_retire_pc;
                fl_recover_dis = (i == 2) ? fl_recover_dis_stage1 :
                                 (i == 1) ? fl_recover_dis_stage2 :
                                           fl_recover_dis_stage3;
                stop_retire = 1'b1; // Stop retiring earlier entries
            end

            if (rob_head_entry[i].halt) begin
                stop_retire = 1'b1;
            end
        end
        else begin
            stop_retire = 1'b1;
        end
    end
end

endmodule

