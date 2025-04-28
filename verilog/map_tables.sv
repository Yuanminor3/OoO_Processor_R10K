`define TEST_MODE 
`include "verilog/sys_defs.svh"

module map_table (
    input                       clock,             
    input                       reset,             
    input       [31:0][`PR-1:0] archi_maptable,    // Architectural map table (checkpoint for recovery)
    input                       BPRecoverEN,       // Branch prediction recovery enable
    input       CDB_T_PACKET    cdb_t_in,          // Common Data Bus broadcast packet for completion
    input       [2:0][`PR-1:0]  maptable_new_pr,   // New physical registers allocated
    input       [2:0][4:0]      maptable_new_ar,   // Corresponding architectural registers for the new physical registers
    input       [2:0][4:0]      reg1_ar,           // Source architectural register 1
    input       [2:0][4:0]      reg2_ar,           // Source architectural register 2
    output logic [2:0][`PR-1:0] reg1_tag,          // Mapped physical register tag for reg1
    output logic [2:0][`PR-1:0] reg2_tag,          // Mapped physical register tag for reg2
    output logic [2:0]          reg1_ready,        // Ready bit for reg1
    output logic [2:0]          reg2_ready,        // Ready bit for reg2
    output logic [2:0][`PR-1:0] Told_out           // Old physical register mappings

    `ifdef TEST_MODE
    , output logic [31:0][`PR-1:0] map_array_disp  // Debug output: full map array
    , output logic [31:0]          ready_array_disp // Debug output: full ready array
    `endif 
);

    // Main mapping and readiness state
    logic [31:0][`PR-1:0] map_array;
    logic [31:0]          ready_array;

    // Intermediate pipeline updates (stage-by-stage update)
    logic [3:0][31:0][`PR-1:0] map_array_next;
    logic [3:0][31:0]          ready_array_next;

    // Reset and recovery values
    logic [31:0][`PR-1:0] map_array_reset;

    // Debug output assignment
    `ifdef TEST_MODE
    assign map_array_disp   = map_array;
    assign ready_array_disp = ready_array;
    `endif 

    // Compute reset state: identity mapping, all ready
    always_comb begin
        for (int i = 0; i < 32; i++) begin
            map_array_reset[i]   = i[`PR-1:0];
        end
    end

    // Register update on clock edge
    always_ff @(posedge clock) begin
        if (reset) begin
            map_array   <= `SD map_array_reset;
            ready_array <= `SD 32'hFFFFFFFF;
        end
        else if (BPRecoverEN) begin
            map_array   <= `SD archi_maptable;
            ready_array <= `SD 32'hFFFFFFFF;
        end
        else begin
            map_array   <= `SD map_array_next[0];
            ready_array <= `SD ready_array_next[0];
        end
    end

    // Update logic: handles dispatch and CDB broadcast
    always_comb begin
        // Start from current state
        map_array_next[3]   = map_array;
        ready_array_next[3] = ready_array;


	//below can  be changed if changing the packet CDB
        // Broadcast CDB completions: mark ready bits
        for (int j = 0; j < 32; j++) begin
            if (map_array[j] == cdb_t_in.t0 ||
                map_array[j] == cdb_t_in.t1 ||
                map_array[j] == cdb_t_in.t2) begin
                ready_array_next[3][j] = 1'b1;
            end
        end

        // Dispatch new mappings (3-way superscalar)
        for (int i = 2; i >= 0; --i) begin
            map_array_next[i] = map_array_next[i+1];
            map_array_next[i][maptable_new_ar[i]] = maptable_new_pr[i];

            ready_array_next[i] = ready_array_next[i+1];
            if (maptable_new_ar[i] != 0) begin
                ready_array_next[i][maptable_new_ar[i]] = 1'b0; // New physical register is not ready yet
            end
        end
    end

    // Lookup logic: get tags and ready bits for source registers
    always_comb begin
        for (int i = 2; i >= 0; --i) begin
            Told_out[i]    = map_array_next[i+1][maptable_new_ar[i]];
            reg1_tag[i]    = map_array_next[i+1][reg1_ar[i]];
            reg2_tag[i]    = map_array_next[i+1][reg2_ar[i]];
            reg1_ready[i]  = ready_array_next[i+1][reg1_ar[i]];
            reg2_ready[i]  = ready_array_next[i+1][reg2_ar[i]];
        end
    end

endmodule



