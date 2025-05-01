`include "verilog/sys_defs.svh"

module retire_stage (
    input           [`ROB-1:0]              fl_distance,         // Distance in ROB for reg-write instructions
    output  logic                           BPRecoverEN,         // Enable signal for branch predictor recovery
    output  logic   [`XLEN-1:0]             target_pc,           // Target PC for recovery
    input           ROB_ENTRY_PACKET[2:0]   rob_head_entry,      // ROB head entries (3 in-order retire entries)
    
    input           [31:0][`PR-1:0]         archi_maptable,      // Architectural map table from AMT

    output  logic   [31:0][`PR-1:0]         recover_maptable,    // Recovered architectural map table

    output  logic   [2:0]                   Retire_EN,           // Retire enable signals for 3 entries
    output  logic   [2:0]                   SQRetireEN,          // Store queue retire enable signals

    output  logic                           halt,                // Halt signal
    output  logic   [2:0]	            inst_count
);

// When the branch prediction is wrong, the recovery distance (fl_recover_dis) adjusts the free list header pointer to roll back to the correct physical register allocation state. 

// Depending on whether the retirement instruction is written to the architecture register (arch_reg != 0), progressively reducing the recovery distance (fl_recover_dis), which represents the number of physical registers to be freed
// Intermediate rollback distance through stages
logic [`ROB-1:0] fl_recover_dis;
logic [`ROB-1:0] fl_recover_dis_stage1, fl_recover_dis_stage2, fl_recover_dis_stage3;
logic [2:0] retire_valid;

// Calculate rollback distance if entry writes to AR
assign fl_recover_dis_stage1 = rob_head_entry[2].arch_reg==5'b0 ? fl_distance : fl_distance - 1;
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

assign inst_count = retire_valid;

logic stop_retire;
always_comb begin
    Retire_EN         = 3'b000;
    SQRetireEN        = 3'b000;
    BPRecoverEN       = 1'b0;
    retire_valid      = 3'b000;
    halt              = 1'b0;
    fl_recover_dis    = fl_distance;
    recover_maptable  = archi_maptable;
    target_pc         = 32'd0;
    stop_retire = 1'b0;
    // Flags to check if we should continue retiring

    for (int i = 2; i >= 0; i--) begin
        if (!stop_retire && rob_head_entry[i].completed) begin
            retire_valid[i] = 1'b1;
            Retire_EN[i]    = (i == 2) ? is_write_bit[2] :
                              (i == 1) ? is_write_bit[1] :
                                         is_write_bit[0];

            SQRetireEN[i]   = rob_head_entry[i].is_store;
            halt            = rob_head_entry[i].halt;

            recover_maptable[rob_head_entry[i].arch_reg] = rob_head_entry[i].Tnew;

            if (rob_head_entry[i].precise_state_need) begin
                BPRecoverEN    = 1'b1;
                target_pc      = rob_head_entry[i].target_pc;
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

