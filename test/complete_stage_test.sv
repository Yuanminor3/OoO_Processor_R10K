`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module complete_stage_tb;

    // Clock and reset signals
    logic clock;
    logic reset;

    // Inputs to DUT
    FU_STATE_PACKET             fu_finish;
    FU_COMPLETE_PACKET [7:0]    fu_c_in;

    // Outputs from DUT
    FU_STATE_PACKET             fu_c_stall;
    CDB_T_PACKET                cdb_t;
    logic [2:0][`XLEN-1:0]      wb_value;
    logic [2:0]                 complete_valid;
    logic [2:0][`ROB-1:0]       complete_entry;
    logic [2:0]                 finish_valid;
    logic [2:0][`FU:0]          finish;
    logic [2:0][`XLEN-1:0]      target_pc;

    // Instantiate the DUT
    complete_stage dut (
        .clock(clock),
        .reset(reset),
        .fu_finish(fu_finish),
        .fu_c_in(fu_c_in),
        .fu_c_stall(fu_c_stall),
        .cdb_t(cdb_t),
        .wb_value(wb_value),
        .complete_valid(complete_valid),
        .complete_entry(complete_entry),
        .finish_valid(finish_valid),
        .finish(finish),
        .target_pc(target_pc)
    );

    // Clock generation
    always begin
        #5 clock = ~clock;
    end

    // Main test sequence
    initial begin
        // Initialize signals
        clock = 0;
        reset = 1;
        fu_finish = 0;
        fu_c_in = '{default:0};

        // Apply reset
        #10 reset = 0;

        // Test 1: Single ALU1 completion
        $display("\n=== Test 1: Single ALU1 completion ===");
        fu_finish.alu_1 = 1;
        fu_c_in[0].valid = 1;
        fu_c_in[0].dest_pr = 5'h01;
        fu_c_in[0].dest_value = 32'h12345678;
        fu_c_in[0].rob_entry = 5'd10;
        #10;

        if (cdb_t.t2 == 5'h01 && wb_value[2] == 32'h12345678 && complete_valid[2]) begin
            $display("✅ [PASS] ALU1 writeback correct: cdb_t.t2 = %h, wb_value[2] = %h", cdb_t.t2, wb_value[2]);
        end else begin
            $display("❌ [FAIL] ALU1: cdb_t.t2 = %h, wb_value[2] = %h, complete_valid = %b", cdb_t.t2, wb_value[2], complete_valid);
        end

        $display("\n=== Test Complete ===");
        $finish;
    end

endmodule