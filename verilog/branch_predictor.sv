
`timescale 1ns/100ps

//-----------------------------------------------------------------------------
// Tournament Branch Predictor (Local + GShare Hybrid)
//-----------------------------------------------------------------------------
// This predictor uses two parallel 2-bit predictors:
//   1) Local predictor, indexed by PC low bits (per‐branch history).
//   2) Global (GShare) predictor, indexed by PC low bits XOR’d with global_history_register.
// A 2-bit branch_sel_cnters_next table records which predictor has been more accurate
// for each PC index, and selects between local or global at fetch time.
// global_history_register (Global History Register) keeps the last `SYS_BRANCH_PREDICTION` branch outcomes.
//-----------------------------------------------------------------------------

module branch_predictor(
    input                       clk,
    input                       rst,

    // New branch entry allocation in Dispatch stage
    input       [2:0]           bp_disp_enable,
    input  [2:0][`SYS_XLEN-1:0]     bp_disp_addr,

    // Update signals from branch_stage
    input                       bs_upd_en,
    input  [`SYS_XLEN-1:0]          bs_upd_pc,
    input                       bs_upd_taken,
    input  [`SYS_XLEN-1:0]          bs_upd_target,

    // Lookup in Fetch stage
    input       [2:0]           bp_fetch_enable,
    input  [2:0][`SYS_XLEN-1:0]     bp_fetch_addr,

    // Prediction outputs
    output logic [2:0]          bp_pred_valid,
    output logic [2:0]          bp_pred_taken,
    output logic [2:0][`SYS_XLEN-1:0] bp_pred_target
);

    // ----------------------------------------------------------------
    // Parameters and table declarations
    // ----------------------------------------------------------------
    localparam int BP_TBL_SIZE     = 1 << `BPW;   // number of entries
    localparam int BP_INDEX_OFFSET = 2;           // index bits start after skipping low 2 bits

    // Local, global, and branch_sel_cnters_next tables
    BP_ENTRY_PACKET [`BPW-1:0] local_predictor_bank,  local_predictor_bank_next;
    BP_ENTRY_PACKET [`BPW-1:0] gshare_predictor_bank, gshare_predictor_bank_next;
    logic [1:0]                branch_sel_cnters_next     [BP_TBL_SIZE-1:0],
                               chooser_next[BP_TBL_SIZE-1:0];

    // Global History Register
    logic [`SYS_BRANCH_PREDICTION-1:0]            global_history_register, global_history_reg_nxt;

    // branch_sel_cnters_next update comparison
    logic                      was_local_pred_accurate, global_correct;

    // ----------------------------------------------------------------
    // 1) Lookup: manually unroll for three fetch slots
    // ----------------------------------------------------------------
    always_comb begin
        bp_pred_valid     = '0;
        bp_pred_taken = '0;
        bp_pred_target        = '0;

        // -- SLOT 2 --
        if (bp_fetch_enable[2]) begin
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_loc_idx2 = bp_fetch_addr[2][BP_INDEX_OFFSET +: `SYS_BRANCH_PREDICTION];
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_gsh_idx2 = bp_loc_idx2 ^ global_history_register;
            if (branch_sel_cnters_next[bp_loc_idx2][1]) begin
                if (gshare_predictor_bank[bp_gsh_idx2].valid
                 && gshare_predictor_bank[bp_gsh_idx2].tag == bp_fetch_addr[2][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                    bp_pred_valid[2]     = 1;
                    bp_pred_taken[2] = (gshare_predictor_bank[bp_gsh_idx2].direction == WEAK_T
                                         || gshare_predictor_bank[bp_gsh_idx2].direction == STRONG_T);
                    bp_pred_target[2]        = bp_pred_taken[2]
                                         ? gshare_predictor_bank[bp_gsh_idx2].cs_retire_pc
                                         : '0;
                end
            end else begin
                if (local_predictor_bank[bp_loc_idx2].valid
                 && local_predictor_bank[bp_loc_idx2].tag == bp_fetch_addr[2][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                    bp_pred_valid[2]     = 1;
                    bp_pred_taken[2] = (local_predictor_bank[bp_loc_idx2].direction == WEAK_T
                                         || local_predictor_bank[bp_loc_idx2].direction == STRONG_T);
                    bp_pred_target[2]        = bp_pred_taken[2]
                                         ? local_predictor_bank[bp_loc_idx2].cs_retire_pc
                                         : '0;
                end
            end
        end

        // -- SLOT 1 --
        if (bp_fetch_enable[1]) begin
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_loc_idx1 = bp_fetch_addr[1][BP_INDEX_OFFSET +: `SYS_BRANCH_PREDICTION];
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_gsh_idx1 = bp_loc_idx1 ^ global_history_register;
            if (branch_sel_cnters_next[bp_loc_idx1][1]) begin
                if (gshare_predictor_bank[bp_gsh_idx1].valid
                 && gshare_predictor_bank[bp_gsh_idx1].tag == bp_fetch_addr[1][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                    bp_pred_valid[1]     = 1;
                    bp_pred_taken[1] = (gshare_predictor_bank[bp_gsh_idx1].direction == WEAK_T
                                         || gshare_predictor_bank[bp_gsh_idx1].direction == STRONG_T);
                    bp_pred_target[1]        = bp_pred_taken[1]
                                         ? gshare_predictor_bank[bp_gsh_idx1].cs_retire_pc
                                         : '0;
                end
            end else begin
                if (local_predictor_bank[bp_loc_idx1].valid
                 && local_predictor_bank[bp_loc_idx1].tag == bp_fetch_addr[1][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                    bp_pred_valid[1]     = 1;
                    bp_pred_taken[1] = (local_predictor_bank[bp_loc_idx1].direction == WEAK_T
                                         || local_predictor_bank[bp_loc_idx1].direction == STRONG_T);
                    bp_pred_target[1]        = bp_pred_taken[1]
                                         ? local_predictor_bank[bp_loc_idx1].cs_retire_pc
                                         : '0;
                end
            end
        end

        // -- SLOT 0 --
        if (bp_fetch_enable[0]) begin
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_loc_idx0 = bp_fetch_addr[0][BP_INDEX_OFFSET +: `SYS_BRANCH_PREDICTION];
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_gsh_idx0 = bp_loc_idx0 ^ global_history_register;
            if (branch_sel_cnters_next[bp_loc_idx0][1]) begin
                if (gshare_predictor_bank[bp_gsh_idx0].valid
                 && gshare_predictor_bank[bp_gsh_idx0].tag == bp_fetch_addr[0][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                    bp_pred_valid[0]     = 1;
                    bp_pred_taken[0] = (gshare_predictor_bank[bp_gsh_idx0].direction == WEAK_T
                                         || gshare_predictor_bank[bp_gsh_idx0].direction == STRONG_T);
                    bp_pred_target[0]        = bp_pred_taken[0]
                                         ? gshare_predictor_bank[bp_gsh_idx0].cs_retire_pc
                                         : '0;
                end
            end else begin
                if (local_predictor_bank[bp_loc_idx0].valid
                 && local_predictor_bank[bp_loc_idx0].tag == bp_fetch_addr[0][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                    bp_pred_valid[0]     = 1;
                    bp_pred_taken[0] = (local_predictor_bank[bp_loc_idx0].direction == WEAK_T
                                         || local_predictor_bank[bp_loc_idx0].direction == STRONG_T);
                    bp_pred_target[0]        = bp_pred_taken[0]
                                         ? local_predictor_bank[bp_loc_idx0].cs_retire_pc
                                         : '0;
                end
            end
        end
    end

    // ----------------------------------------------------------------
    // 2) Allocate & Update: Dispatch allocation & Branch SYS_FU_ADDR_WIDTH feedback update
    // ----------------------------------------------------------------
    always_comb begin
        // First copy all table entries to next
        for (int i = 0; i < BP_TBL_SIZE; i++) begin
            local_predictor_bank_next[i]   = local_predictor_bank[i];
            gshare_predictor_bank_next[i]  = gshare_predictor_bank[i];
            chooser_next[i] = branch_sel_cnters_next[i];
        end
        global_history_reg_nxt = global_history_register;

        // Unroll lsq_disp_mask for three slots
        if (bp_disp_enable[2]) begin
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_loc_idx2 = bp_disp_addr[2][BP_INDEX_OFFSET +: `SYS_BRANCH_PREDICTION];
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_gsh_idx2 = bp_loc_idx2 ^ global_history_register;
            if (!local_predictor_bank[bp_loc_idx2].valid
             || local_predictor_bank[bp_loc_idx2].tag != bp_disp_addr[2][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                local_predictor_bank_next[bp_loc_idx2].valid     = 1;
                local_predictor_bank_next[bp_loc_idx2].tag       = bp_disp_addr[2][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION];
                local_predictor_bank_next[bp_loc_idx2].direction = STRONG_T;
                local_predictor_bank_next[bp_loc_idx2].cs_retire_pc = '0;
            end
            if (!gshare_predictor_bank[bp_gsh_idx2].valid
             || gshare_predictor_bank[bp_gsh_idx2].tag != bp_disp_addr[2][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                gshare_predictor_bank_next[bp_gsh_idx2].valid     = 1;
                gshare_predictor_bank_next[bp_gsh_idx2].tag       = bp_disp_addr[2][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION];
                gshare_predictor_bank_next[bp_gsh_idx2].direction = STRONG_T;
                gshare_predictor_bank_next[bp_gsh_idx2].cs_retire_pc = '0;
            end
        end
        if (bp_disp_enable[1]) begin
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_loc_idx1 = bp_disp_addr[1][BP_INDEX_OFFSET +: `SYS_BRANCH_PREDICTION];
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_gsh_idx1 = bp_loc_idx1 ^ global_history_register;
            if (!local_predictor_bank[bp_loc_idx1].valid
             || local_predictor_bank[bp_loc_idx1].tag != bp_disp_addr[1][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                local_predictor_bank_next[bp_loc_idx1].valid     = 1;
                local_predictor_bank_next[bp_loc_idx1].tag       = bp_disp_addr[1][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION];
                local_predictor_bank_next[bp_loc_idx1].direction = STRONG_T;
                local_predictor_bank_next[bp_loc_idx1].cs_retire_pc = '0;
            end
            if (!gshare_predictor_bank[bp_gsh_idx1].valid
             || gshare_predictor_bank[bp_gsh_idx1].tag != bp_disp_addr[1][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                gshare_predictor_bank_next[bp_gsh_idx1].valid     = 1;
                gshare_predictor_bank_next[bp_gsh_idx1].tag       = bp_disp_addr[1][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION];
                gshare_predictor_bank_next[bp_gsh_idx1].direction = STRONG_T;
                gshare_predictor_bank_next[bp_gsh_idx1].cs_retire_pc = '0;
            end
        end
        if (bp_disp_enable[0]) begin
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_loc_idx0 = bp_disp_addr[0][BP_INDEX_OFFSET +: `SYS_BRANCH_PREDICTION];
            logic [`SYS_BRANCH_PREDICTION-1:0] bp_gsh_idx0 = bp_loc_idx0 ^ global_history_register;
            if (!local_predictor_bank[bp_loc_idx0].valid
             || local_predictor_bank[bp_loc_idx0].tag != bp_disp_addr[0][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                local_predictor_bank_next[bp_loc_idx0].valid     = 1;
                local_predictor_bank_next[bp_loc_idx0].tag       = bp_disp_addr[0][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION];
                local_predictor_bank_next[bp_loc_idx0].direction = STRONG_T;
                local_predictor_bank_next[bp_loc_idx0].cs_retire_pc = '0;
            end
            if (!gshare_predictor_bank[bp_gsh_idx0].valid
             || gshare_predictor_bank[bp_gsh_idx0].tag != bp_disp_addr[0][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                gshare_predictor_bank_next[bp_gsh_idx0].valid     = 1;
                gshare_predictor_bank_next[bp_gsh_idx0].tag       = bp_disp_addr[0][`SYS_XLEN-1:`SYS_BRANCH_PREDICTION];
                gshare_predictor_bank_next[bp_gsh_idx0].direction = STRONG_T;
                gshare_predictor_bank_next[bp_gsh_idx0].cs_retire_pc = '0;
            end
        end

        // Branch SYS_FU_ADDR_WIDTH feedback update
        if (bs_upd_en) begin
            logic [`SYS_BRANCH_PREDICTION-1:0] lidx = bs_upd_pc[BP_INDEX_OFFSET +: `SYS_BRANCH_PREDICTION];
            logic [`SYS_BRANCH_PREDICTION-1:0] gidx = lidx ^ global_history_register;
            // Local table update
            if (local_predictor_bank[lidx].valid
             && local_predictor_bank[lidx].tag == bs_upd_pc[`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                local_predictor_bank_next[lidx].cs_retire_pc = bs_upd_target;
                unique case (local_predictor_bank[lidx].direction)
                    STRONG_NT: local_predictor_bank_next[lidx].direction = bs_upd_taken ? WEAK_NT  : STRONG_NT;
                    WEAK_NT:   local_predictor_bank_next[lidx].direction = bs_upd_taken ? WEAK_T   : STRONG_NT;
                    WEAK_T:    local_predictor_bank_next[lidx].direction = bs_upd_taken ? STRONG_T : WEAK_NT;
                    STRONG_T:  local_predictor_bank_next[lidx].direction = bs_upd_taken ? STRONG_T : WEAK_T;
                endcase
            end
            // Global table update
            if (gshare_predictor_bank[gidx].valid
             && gshare_predictor_bank[gidx].tag == bs_upd_pc[`SYS_XLEN-1:`SYS_BRANCH_PREDICTION]) begin
                gshare_predictor_bank_next[gidx].cs_retire_pc = bs_upd_target;
                unique case (gshare_predictor_bank[gidx].direction)
                    STRONG_NT: gshare_predictor_bank_next[gidx].direction = bs_upd_taken ? WEAK_NT  : STRONG_NT;
                    WEAK_NT:   gshare_predictor_bank_next[gidx].direction = bs_upd_taken ? WEAK_T   : STRONG_NT;
                    WEAK_T:    gshare_predictor_bank_next[gidx].direction = bs_upd_taken ? STRONG_T : WEAK_NT;
                    STRONG_T:  gshare_predictor_bank_next[gidx].direction = bs_upd_taken ? STRONG_T : WEAK_T;
                endcase
            end
            // branch_sel_cnters_next update
            was_local_pred_accurate  = (local_predictor_bank[lidx].direction[1] == bs_upd_taken);
            global_correct = (gshare_predictor_bank[gidx].direction[1] == bs_upd_taken);
            if (was_local_pred_accurate != global_correct) begin
                if (global_correct && branch_sel_cnters_next[lidx] != 2'b11)
                    chooser_next[lidx] = branch_sel_cnters_next[lidx] + 1;
                else if (was_local_pred_accurate && branch_sel_cnters_next[lidx] != 2'b00)
                    chooser_next[lidx] = branch_sel_cnters_next[lidx] - 1;
            end
            // global_history_register update
            global_history_reg_nxt = {global_history_register[`SYS_BRANCH_PREDICTION-2:0], bs_upd_taken};
        end
    end

    // ----------------------------------------------------------------
    // 3) Sequential update: write back on posedge clk
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < BP_TBL_SIZE; i++) begin
                local_predictor_bank [i] <= `SYS_SMALL_DELAY '{default:'0};
                gshare_predictor_bank[i] <= `SYS_SMALL_DELAY '{default:'0};
                branch_sel_cnters_next     [i] <= `SYS_SMALL_DELAY 2'b10;  // initialize weak global preference
            end
            global_history_register <= `SYS_SMALL_DELAY '0;
        end else begin
            for (int i = 0; i < BP_TBL_SIZE; i++) begin
                local_predictor_bank [i] <= `SYS_SMALL_DELAY local_predictor_bank_next [i];
                gshare_predictor_bank[i] <= `SYS_SMALL_DELAY gshare_predictor_bank_next[i];
                branch_sel_cnters_next     [i] <= `SYS_SMALL_DELAY chooser_next[i];
            end
            global_history_register <= `SYS_SMALL_DELAY global_history_reg_nxt;
        end
    end

endmodule


