`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module complete_stage_tb;

    // Clock and rst signals
    logic clk;
    logic rst;

    // Inputs to DUT
    FU_STATE_PACKET             cs_fu_done_flags;
    FU_COMPLETE_PACKET [7:0]    cs_fu_complete_pkts;

    // Outputs from DUT
    FU_STATE_PACKET             cs_stall_mask;
    CDB_T_PACKET                cs_cdb_broadcast;
    logic [2:0][`SYS_XLEN-1:0]      cs_wb_data;
    logic [2:0]                 cs_retire_valid;
    logic [2:0][`SYS_ROB_ADDR_WIDTH-1:0]       cs_retire_idx;
    logic [2:0]                 cs_finish_valid;
    logic [2:0][`SYS_FU_ADDR_WIDTH:0]          finish;
    logic [2:0][`SYS_XLEN-1:0]      cs_retire_pc;

    // Instantiate the DUT
    complete_stage dut (
        .clk(clk),
        .rst(rst),
        .cs_fu_done_flags(cs_fu_done_flags),
        .cs_fu_complete_pkts(cs_fu_complete_pkts),
        .cs_stall_mask(cs_stall_mask),
        .cs_cdb_broadcast(cs_cdb_broadcast),
        .cs_wb_data(cs_wb_data),
        .cs_retire_valid(cs_retire_valid),
        .cs_retire_idx(cs_retire_idx),
        .cs_finish_valid(cs_finish_valid),
        .finish(finish),
        .cs_retire_pc(cs_retire_pc)
    );

    // Clock generation
    always begin
        #5 clk = ~clk;
    end

    // Main test sequence
    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        cs_fu_done_flags = 0;
        cs_fu_complete_pkts = '{default:0};

        // Apply rst
        #10 rst = 0;

        // Test 1: Single ALU1 completion
        $display("\n=== Test 1: Single ALU1 completion ===");
        cs_fu_done_flags.alu_1 = 1;
        cs_fu_complete_pkts[0].valid = 1;
        cs_fu_complete_pkts[0].dispatch_allocated_prs = 5'h01;
        cs_fu_complete_pkts[0].dest_value = 32'h12345678;
        cs_fu_complete_pkts[0].rob_entry = 5'd10;
        #10;

        if (cs_cdb_broadcast.t2 == 5'h01 && cs_wb_data[2] == 32'h12345678 && cs_retire_valid[2]) begin
            $display("✅ [ALL PASS] ALU1 writeback correct: cs_cdb_broadcast.t2 = %h, cs_wb_data[2] = %h", cs_cdb_broadcast.t2, cs_wb_data[2]);
        end else begin
            $display("❌ [FAIL] ALU1: cs_cdb_broadcast.t2 = %h, cs_wb_data[2] = %h, cs_retire_valid = %b", cs_cdb_broadcast.t2, cs_wb_data[2], cs_retire_valid);
        end

        $display("\n=== Test Complete ===");
        $finish;
    end

endmodule
