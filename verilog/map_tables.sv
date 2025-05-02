
`include "verilog/sys_defs.svh"

module map_table (
    input                       clk,             
    input                       rst,             
    input       [31:0][`SYS_PHYS_REG-1:0] mt_checkpoint_tbl,    // Architectural map table (checkpoint for recovery)
    input                       fch_rec_enable,       // Branch prediction recovery enable
    input       CDB_T_PACKET    cdb_t_in,          // Common Data Bus broadcast packet for completion
    input       [2:0][`SYS_PHYS_REG-1:0]  dispatch_pr_alloc_tags,   // New physical registers allocated
    input       [2:0][4:0]      mt_new_arch_regs,   // Corresponding architectural registers for the new physical registers
    input       [2:0][4:0]      dispatch_src1_arch_regs,           // Source architectural register 1
    input       [2:0][4:0]      dispatch_src2_arch_regs,           // Source architectural register 2
    output logic [2:0][`SYS_PHYS_REG-1:0] reg1_tag,          // Mapped physical register tag for dec_src1_reg
    output logic [2:0][`SYS_PHYS_REG-1:0] reg2_tag,          // Mapped physical register tag for dec_src2_reg
    output logic [2:0]          dispatch_src1_rdy,        // Ready bit for dec_src1_reg
    output logic [2:0]          dispatch_src2_rdy,        // Ready bit for dec_src2_reg
    output logic [2:0][`SYS_PHYS_REG-1:0] mt_old_pr_tags           // Old physical register mappings

);

    // Main mapping and readiness state
    logic [31:0][`SYS_PHYS_REG-1:0] mt_map_array;
    logic [31:0]          mt_ready_mask;

    // Intermediate pipeline updates (stage-by-stage update)
    logic [3:0][31:0][`SYS_PHYS_REG-1:0] mt_map_pipeline;
    logic [3:0][31:0]          mt_ready_pipeline;

    // Reset and recovery values
    logic [31:0][`SYS_PHYS_REG-1:0] mt_reset_map;

    // Compute rst state: identity mapping, all ready
    always_comb begin
        for (int i = 0; i < 32; i++) begin
            mt_reset_map[i]   = i[`SYS_PHYS_REG-1:0];
        end
    end

    // Register update on clk edge
    always_ff @(posedge clk) begin
        if (rst) begin
            mt_map_array   <= `SYS_SMALL_DELAY mt_reset_map;
            mt_ready_mask <= `SYS_SMALL_DELAY 32'hFFFFFFFF;
        end
        else if (fch_rec_enable) begin
            mt_map_array   <= `SYS_SMALL_DELAY mt_checkpoint_tbl;
            mt_ready_mask <= `SYS_SMALL_DELAY 32'hFFFFFFFF;
        end
        else begin
            mt_map_array   <= `SYS_SMALL_DELAY mt_map_pipeline[0];
            mt_ready_mask <= `SYS_SMALL_DELAY mt_ready_pipeline[0];
        end
    end

    // Update logic: handles lsq_disp_mask and CDB broadcast
    always_comb begin
        // Start from current state
        mt_map_pipeline[3]   = mt_map_array;
        mt_ready_pipeline[3] = mt_ready_mask;


	//below can  be changed if changing the packet CDB
        // Broadcast CDB completions: mark ready bits
        for (int j = 0; j < 32; j++) begin
            if (mt_map_array[j] == cdb_t_in.t0 ||
                mt_map_array[j] == cdb_t_in.t1 ||
                mt_map_array[j] == cdb_t_in.t2) begin
                mt_ready_pipeline[3][j] = 1'b1;
            end
        end

        // Dispatch new mappings (3-way superscalar)
        for (int i = 2; i >= 0; --i) begin
            mt_map_pipeline[i] = mt_map_pipeline[i+1];
            mt_map_pipeline[i][mt_new_arch_regs[i]] = dispatch_pr_alloc_tags[i];

            mt_ready_pipeline[i] = mt_ready_pipeline[i+1];
            if (mt_new_arch_regs[i] != 0) begin
                mt_ready_pipeline[i][mt_new_arch_regs[i]] = 1'b0; // New physical register is not ready yet
            end
        end
    end

    // Lookup logic: get tags and ready bits for source registers
    always_comb begin
        for (int i = 2; i >= 0; --i) begin
            mt_old_pr_tags[i]    = mt_map_pipeline[i+1][mt_new_arch_regs[i]];
            reg1_tag[i]    = mt_map_pipeline[i+1][dispatch_src1_arch_regs[i]];
            reg2_tag[i]    = mt_map_pipeline[i+1][dispatch_src2_arch_regs[i]];
            dispatch_src1_rdy[i]  = mt_ready_pipeline[i+1][dispatch_src1_arch_regs[i]];
            dispatch_src2_rdy[i]  = mt_ready_pipeline[i+1][dispatch_src2_arch_regs[i]];
        end
    end

endmodule



