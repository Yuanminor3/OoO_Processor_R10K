
`timescale 1ns/100ps

//-----------------------------------------------------------------------------
// Tournament Branch Predictor (Local + GShare Hybrid)
//-----------------------------------------------------------------------------
// This predictor uses two parallel 2-bit predictors:
//   1) Local predictor, indexed by PC low bits (per‐branch history).
//   2) Global (GShare) predictor, indexed by PC low bits XOR’d with GHR.
// A 2-bit chooser table records which predictor has been more accurate
// for each PC index, and selects between local or global at fetch time.
// GHR (Global History Register) keeps the last `BP` branch outcomes.
//-----------------------------------------------------------------------------

module branch_predictor(
    input                       clock,
    input                       reset,

    // New branch entry allocation in Dispatch stage
    input       [2:0]           dispatch_EN,
    input  [2:0][`XLEN-1:0]     dispatch_pc,

    // Update signals from branch_stage
    input                       update_EN,
    input  [`XLEN-1:0]          update_pc,
    input                       update_direction,
    input  [`XLEN-1:0]          update_target,

    // Lookup in Fetch stage
    input       [2:0]           fetch_EN,
    input  [2:0][`XLEN-1:0]     fetch_pc,

    // Prediction outputs
    output logic [2:0]          predict_found,
    output logic [2:0]          predict_direction,
    output logic [2:0][`XLEN-1:0] predict_pc
);

    // ----------------------------------------------------------------
    // Parameters and table declarations
    // ----------------------------------------------------------------
    localparam int N     = 1 << `BPW;   // number of entries
    localparam int ALIGN = 2;           // index bits start after skipping low 2 bits

    // Local, global, and chooser tables
    BP_ENTRY_PACKET [`BPW-1:0] local_table,  local_next;
    BP_ENTRY_PACKET [`BPW-1:0] global_table, global_next;
    logic [1:0]                chooser     [N-1:0],
                               chooser_next[N-1:0];

    // Global History Register
    logic [`BP-1:0]            GHR, GHR_next;

    // chooser update comparison
    logic                      local_correct, global_correct;

    // ----------------------------------------------------------------
    // 1) Lookup: manually unroll for three fetch slots
    // ----------------------------------------------------------------
    always_comb begin
        predict_found     = '0;
        predict_direction = '0;
        predict_pc        = '0;

        // -- SLOT 2 --
        if (fetch_EN[2]) begin
            logic [`BP-1:0] lidx2 = fetch_pc[2][ALIGN +: `BP];
            logic [`BP-1:0] gidx2 = lidx2 ^ GHR;
            if (chooser[lidx2][1]) begin
                if (global_table[gidx2].valid
                 && global_table[gidx2].tag == fetch_pc[2][`XLEN-1:`BP]) begin
                    predict_found[2]     = 1;
                    predict_direction[2] = (global_table[gidx2].direction == WEAK_T
                                         || global_table[gidx2].direction == STRONG_T);
                    predict_pc[2]        = predict_direction[2]
                                         ? global_table[gidx2].target_pc
                                         : '0;
                end
            end else begin
                if (local_table[lidx2].valid
                 && local_table[lidx2].tag == fetch_pc[2][`XLEN-1:`BP]) begin
                    predict_found[2]     = 1;
                    predict_direction[2] = (local_table[lidx2].direction == WEAK_T
                                         || local_table[lidx2].direction == STRONG_T);
                    predict_pc[2]        = predict_direction[2]
                                         ? local_table[lidx2].target_pc
                                         : '0;
                end
            end
        end

        // -- SLOT 1 --
        if (fetch_EN[1]) begin
            logic [`BP-1:0] lidx1 = fetch_pc[1][ALIGN +: `BP];
            logic [`BP-1:0] gidx1 = lidx1 ^ GHR;
            if (chooser[lidx1][1]) begin
                if (global_table[gidx1].valid
                 && global_table[gidx1].tag == fetch_pc[1][`XLEN-1:`BP]) begin
                    predict_found[1]     = 1;
                    predict_direction[1] = (global_table[gidx1].direction == WEAK_T
                                         || global_table[gidx1].direction == STRONG_T);
                    predict_pc[1]        = predict_direction[1]
                                         ? global_table[gidx1].target_pc
                                         : '0;
                end
            end else begin
                if (local_table[lidx1].valid
                 && local_table[lidx1].tag == fetch_pc[1][`XLEN-1:`BP]) begin
                    predict_found[1]     = 1;
                    predict_direction[1] = (local_table[lidx1].direction == WEAK_T
                                         || local_table[lidx1].direction == STRONG_T);
                    predict_pc[1]        = predict_direction[1]
                                         ? local_table[lidx1].target_pc
                                         : '0;
                end
            end
        end

        // -- SLOT 0 --
        if (fetch_EN[0]) begin
            logic [`BP-1:0] lidx0 = fetch_pc[0][ALIGN +: `BP];
            logic [`BP-1:0] gidx0 = lidx0 ^ GHR;
            if (chooser[lidx0][1]) begin
                if (global_table[gidx0].valid
                 && global_table[gidx0].tag == fetch_pc[0][`XLEN-1:`BP]) begin
                    predict_found[0]     = 1;
                    predict_direction[0] = (global_table[gidx0].direction == WEAK_T
                                         || global_table[gidx0].direction == STRONG_T);
                    predict_pc[0]        = predict_direction[0]
                                         ? global_table[gidx0].target_pc
                                         : '0;
                end
            end else begin
                if (local_table[lidx0].valid
                 && local_table[lidx0].tag == fetch_pc[0][`XLEN-1:`BP]) begin
                    predict_found[0]     = 1;
                    predict_direction[0] = (local_table[lidx0].direction == WEAK_T
                                         || local_table[lidx0].direction == STRONG_T);
                    predict_pc[0]        = predict_direction[0]
                                         ? local_table[lidx0].target_pc
                                         : '0;
                end
            end
        end
    end

    // ----------------------------------------------------------------
    // 2) Allocate & Update: Dispatch allocation & Branch FU feedback update
    // ----------------------------------------------------------------
    always_comb begin
        // First copy all table entries to next
        for (int i = 0; i < N; i++) begin
            local_next[i]   = local_table[i];
            global_next[i]  = global_table[i];
            chooser_next[i] = chooser[i];
        end
        GHR_next = GHR;

        // Unroll dispatch for three slots
        if (dispatch_EN[2]) begin
            logic [`BP-1:0] lidx2 = dispatch_pc[2][ALIGN +: `BP];
            logic [`BP-1:0] gidx2 = lidx2 ^ GHR;
            if (!local_table[lidx2].valid
             || local_table[lidx2].tag != dispatch_pc[2][`XLEN-1:`BP]) begin
                local_next[lidx2].valid     = 1;
                local_next[lidx2].tag       = dispatch_pc[2][`XLEN-1:`BP];
                local_next[lidx2].direction = STRONG_T;
                local_next[lidx2].target_pc = '0;
            end
            if (!global_table[gidx2].valid
             || global_table[gidx2].tag != dispatch_pc[2][`XLEN-1:`BP]) begin
                global_next[gidx2].valid     = 1;
                global_next[gidx2].tag       = dispatch_pc[2][`XLEN-1:`BP];
                global_next[gidx2].direction = STRONG_T;
                global_next[gidx2].target_pc = '0;
            end
        end
        if (dispatch_EN[1]) begin
            logic [`BP-1:0] lidx1 = dispatch_pc[1][ALIGN +: `BP];
            logic [`BP-1:0] gidx1 = lidx1 ^ GHR;
            if (!local_table[lidx1].valid
             || local_table[lidx1].tag != dispatch_pc[1][`XLEN-1:`BP]) begin
                local_next[lidx1].valid     = 1;
                local_next[lidx1].tag       = dispatch_pc[1][`XLEN-1:`BP];
                local_next[lidx1].direction = STRONG_T;
                local_next[lidx1].target_pc = '0;
            end
            if (!global_table[gidx1].valid
             || global_table[gidx1].tag != dispatch_pc[1][`XLEN-1:`BP]) begin
                global_next[gidx1].valid     = 1;
                global_next[gidx1].tag       = dispatch_pc[1][`XLEN-1:`BP];
                global_next[gidx1].direction = STRONG_T;
                global_next[gidx1].target_pc = '0;
            end
        end
        if (dispatch_EN[0]) begin
            logic [`BP-1:0] lidx0 = dispatch_pc[0][ALIGN +: `BP];
            logic [`BP-1:0] gidx0 = lidx0 ^ GHR;
            if (!local_table[lidx0].valid
             || local_table[lidx0].tag != dispatch_pc[0][`XLEN-1:`BP]) begin
                local_next[lidx0].valid     = 1;
                local_next[lidx0].tag       = dispatch_pc[0][`XLEN-1:`BP];
                local_next[lidx0].direction = STRONG_T;
                local_next[lidx0].target_pc = '0;
            end
            if (!global_table[gidx0].valid
             || global_table[gidx0].tag != dispatch_pc[0][`XLEN-1:`BP]) begin
                global_next[gidx0].valid     = 1;
                global_next[gidx0].tag       = dispatch_pc[0][`XLEN-1:`BP];
                global_next[gidx0].direction = STRONG_T;
                global_next[gidx0].target_pc = '0;
            end
        end

        // Branch FU feedback update
        if (update_EN) begin
            logic [`BP-1:0] lidx = update_pc[ALIGN +: `BP];
            logic [`BP-1:0] gidx = lidx ^ GHR;
            // Local table update
            if (local_table[lidx].valid
             && local_table[lidx].tag == update_pc[`XLEN-1:`BP]) begin
                local_next[lidx].target_pc = update_target;
                unique case (local_table[lidx].direction)
                    STRONG_NT: local_next[lidx].direction = update_direction ? WEAK_NT  : STRONG_NT;
                    WEAK_NT:   local_next[lidx].direction = update_direction ? WEAK_T   : STRONG_NT;
                    WEAK_T:    local_next[lidx].direction = update_direction ? STRONG_T : WEAK_NT;
                    STRONG_T:  local_next[lidx].direction = update_direction ? STRONG_T : WEAK_T;
                endcase
            end
            // Global table update
            if (global_table[gidx].valid
             && global_table[gidx].tag == update_pc[`XLEN-1:`BP]) begin
                global_next[gidx].target_pc = update_target;
                unique case (global_table[gidx].direction)
                    STRONG_NT: global_next[gidx].direction = update_direction ? WEAK_NT  : STRONG_NT;
                    WEAK_NT:   global_next[gidx].direction = update_direction ? WEAK_T   : STRONG_NT;
                    WEAK_T:    global_next[gidx].direction = update_direction ? STRONG_T : WEAK_NT;
                    STRONG_T:  global_next[gidx].direction = update_direction ? STRONG_T : WEAK_T;
                endcase
            end
            // chooser update
            local_correct  = (local_table[lidx].direction[1] == update_direction);
            global_correct = (global_table[gidx].direction[1] == update_direction);
            if (local_correct != global_correct) begin
                if (global_correct && chooser[lidx] != 2'b11)
                    chooser_next[lidx] = chooser[lidx] + 1;
                else if (local_correct && chooser[lidx] != 2'b00)
                    chooser_next[lidx] = chooser[lidx] - 1;
            end
            // GHR update
            GHR_next = {GHR[`BP-2:0], update_direction};
        end
    end

    // ----------------------------------------------------------------
    // 3) Sequential update: write back on posedge clock
    // ----------------------------------------------------------------
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i = 0; i < N; i++) begin
                local_table [i] <= `SD '{default:'0};
                global_table[i] <= `SD '{default:'0};
                chooser     [i] <= `SD 2'b10;  // initialize weak global preference
            end
            GHR <= `SD '0;
        end else begin
            for (int i = 0; i < N; i++) begin
                local_table [i] <= `SD local_next [i];
                global_table[i] <= `SD global_next[i];
                chooser     [i] <= `SD chooser_next[i];
            end
            GHR <= `SD GHR_next;
        end
    end

endmodule


