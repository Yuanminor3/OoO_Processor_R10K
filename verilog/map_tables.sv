`define TEST_MODE 
`include "verilog/sys_defs.svh"

module map_table(
    input                       clock,          // Clock signal
    input                       reset,          // Reset signal
    input       [31:0][`PR-1:0] archi_maptable, // Architectural map table checkpoint
    input                       BPRecoverEN,    // Branch prediction recovery enable
    input       CDB_T_PACKET    cdb_t_in,        // Broadcast CDB packet for completion
    input       [2:0][`PR-1:0]  maptable_new_pr, // New physical registers allocated
    input       [2:0][4:0]      maptable_new_ar, // Corresponding architectural registers for new PRs
    input       [2:0][4:0]      reg1_ar,         // Source register 1 architectural names
    input       [2:0][4:0]      reg2_ar,         // Source register 2 architectural names
    output logic [2:0][`PR-1:0] reg1_tag,        // Mapped physical register tags for reg1
    output logic [2:0][`PR-1:0] reg2_tag,        // Mapped physical register tags for reg2
    output logic [2:0]          reg1_ready,      // Ready bits for reg1
    output logic [2:0]          reg2_ready,      // Ready bits for reg2
    output logic [2:0][`PR-1:0] Told_out         // Old physical register mappings
    `ifdef TEST_MODE
    , output logic [31:0][`PR-1:0] map_array_disp, // Display full map_array for debug
      output logic [31:0] ready_array_disp         // Display full ready_array for debug
    `endif
);

    // Internal map and ready arrays
    logic [31:0][`PR-1:0] map_array;          // Main mapping array
    logic [3:0][31:0][`PR-1:0] map_array_next; // Intermediate updated versions
    logic [31:0] ready_array;                 // Ready status array
    logic [3:0][31:0] ready_array_next;        // Intermediate ready updates

    logic [31:0][`PR-1:0] map_array_reset;     // Reset values for map array
    logic [31:0] ready_array_reset;            // Reset values for ready array
    logic [31:0][`PR-1:0] map_array_PS;        // Precise state recovery map
    logic [31:0] ready_array_PS;               // Precise state recovery ready bits


    // Debug outputs under TEST_MODE
    `ifdef TEST_MODE
    assign map_array_disp = map_array;
    assign ready_array_disp = ready_array;
    `endif

    // Reset computation: Identity mapping, all registers ready
    always_comb begin : Compute_reset
        for (int i = 0; i < 32; i++) begin
            map_array_reset[i] = i;
            ready_array_reset[i] = 1'b1;
        end
    end

    // Recovery computation: Use precise checkpoint
    always_comb begin : Precise_stage
        map_array_PS = archi_maptable;
        ready_array_PS = 32'hFFFFFFFF; // All ready
    end

    // Register update with clock edge
    always_ff @(posedge clock) begin : Map_table_reg
        if (reset) begin
            map_array <= `SD map_array_reset;
            ready_array <= `SD ready_array_reset;
        end
        else if (BPRecoverEN) begin
            map_array <= `SD map_array_PS;
            ready_array <= `SD ready_array_PS;
        end
        else begin
            map_array <= `SD map_array_next[0];
            ready_array <= `SD ready_array_next[0];
        end
    end

    // Map table update logic
    always_comb begin : Update_logic
        map_array_next[3] = map_array;
        ready_array_next[3] = ready_array;

        // Update ready array based on CDB broadcast completions
        for (int j = 0; j < 32; j++) begin
            if (map_array[j] == cdb_t_in.t0 ||
                map_array[j] == cdb_t_in.t1 ||
                map_array[j] == cdb_t_in.t2) begin
                ready_array_next[3][j] = 1'b1;
            end
        end

        // Update mapping for new dispatch instructions - 3 way - update one by one
        for (int i = 2; i >= 0; --i) begin
            map_array_next[i] = map_array_next[i+1];
            map_array_next[i][maptable_new_ar[i]] = maptable_new_pr[i];

            ready_array_next[i] = ready_array_next[i+1];
            if (maptable_new_ar[i] != 0) begin
                ready_array_next[i][maptable_new_ar[i]] = 1'b0; // New PR initially not ready
            end
        end
    end

    // Lookup logic for source registers and Told
    always_comb begin : Lookup_logic
        for (int i = 2; i >= 0; --i) begin
            Told_out[i] = map_array_next[i+1][maptable_new_ar[i]];
            reg1_tag[i] = map_array_next[i+1][reg1_ar[i]];
            reg2_tag[i] = map_array_next[i+1][reg2_ar[i]];
            reg1_ready[i] = ready_array_next[i+1][reg1_ar[i]];
            reg2_ready[i] = ready_array_next[i+1][reg2_ar[i]];
        end
    end

endmodule


module arch_maptable (
    input                          clock,             // Clock signal
    input                          reset,             // Reset signal
    input        [2:0][`PR-1:0]     Tnew_in,           // New physical registers to write
    input        [2:0][4:0]         Retire_AR,         // Architectural register indices to retire
    input        [2:0]              Retire_EN,         // Retire enable signals per instruction
    output logic [31:0][`PR-1:0]    archi_maptable     // Output: Current architectural map table
);

    // Internal signals
    logic [31:0][`PR-1:0] archi_maptable_reset;         // Value to reset archi_maptable
    logic [31:0][`PR-1:0] archi_maptable_next;          // Next-state value of archi_maptable

    // Compute the default/reset state: each architectural register maps to itself
    always_comb begin : Compute_reset
        for (int i = 0; i < 32; i++) begin
            archi_maptable_reset[i] = i;
        end
    end

    // Sequential block: Update archi_maptable at each clock edge
    always_ff @(posedge clock) begin : Arch_maptable_reg
        if (reset) begin
            archi_maptable <= `SD archi_maptable_reset; // Reset to initial mapping
        end else begin
            archi_maptable <= `SD archi_maptable_next;  // Otherwise update to computed next value
        end
    end

    // Combinational logic: determine the next-state mapping
    always_comb begin : Update_logic
        archi_maptable_next = archi_maptable; // Start with current map table
        for (int i = 2; i >= 0; i--) begin
            if (Retire_EN[i] == 1'b1) begin
                // If retiring an instruction, update the architectural map
                archi_maptable_next[Retire_AR[i]] = Tnew_in[i];
            end
        end
    end

endmodule


