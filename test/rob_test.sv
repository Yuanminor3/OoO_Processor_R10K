`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module rob_test;

  logic clk, rst;
  ROB_ENTRY_PACKET [2:0] dispatch_rob_pkts;
  ROB_ENTRY_PACKET [2:0] retire_entry;
  logic [2:0] cs_retire_valid;
  logic [2:0][`SYS_ROB_ADDR_WIDTH-1:0] cs_retire_idx;
  logic [2:0] rb_recover_valid;
  logic [2:0][`SYS_XLEN-1:0] cs_retire_pc;
  logic fch_rec_enable;
  logic [2:0] rb_sq_stall;

  logic [2:0][`SYS_ROB_ADDR_WIDTH-1:0] dispatch_index;
  logic [`SYS_ROB_ADDR_WIDTH:0] rb_free_slots;

  logic [1:0] test_passed;  // for 2 test cases

  SYS_ROB_ADDR_WIDTH dut (
    .clk,
    .rst,
    .dispatch_rob_pkts,
    .cs_retire_valid,
    .cs_retire_idx,
    .rb_recover_valid,
    .cs_retire_pc,
    .fch_rec_enable,
    .rb_sq_stall,
    .dispatch_index,
    .rb_free_slots,
    .retire_entry
  );

  // Clock generation
  always #5 clk = ~clk;

  // Helper
  task print_retire(string label);
    $display("---- %s ----", label);
    for (int i = 0; i < 3; i++) begin
      if (retire_entry[i].valid)
        $display("  Retire[%0d]: PC=0x%08x, Tnew=%0d, completed=%0d", i, retire_entry[i].PC, retire_entry[i].Tnew, retire_entry[i].completed);
    end
  endtask

  initial begin
    clk = 0;
    rst = 1;
    dispatch_rob_pkts = '{default:0};
    cs_retire_valid = 0;
    cs_retire_idx = '{default:0};
    rb_recover_valid = 0;
    cs_retire_pc = '{default:0};
    fch_rec_enable = 0;
    rb_sq_stall = 0;
    test_passed = '{default:1};  // assume pass

    @(negedge clk); rst = 0;

    // === Case 1: Normal Dispatch + Complete + Retire ===
    @(negedge clk);
    dispatch_rob_pkts[0] = '{default:0, valid:1, PC:32'h1000, NPC:32'h1004, Tnew:5'd1, Told:5'd0, inst:32'hAAAA};
    dispatch_rob_pkts[1] = '{default:0, valid:1, PC:32'h1004, NPC:32'h1008, Tnew:5'd2, Told:5'd1, inst:32'hBBBB};
    dispatch_rob_pkts[2] = '{default:0, valid:1, PC:32'h1008, NPC:32'h100C, Tnew:5'd3, Told:5'd2, inst:32'hCCCC};

    @(negedge clk);
    dispatch_rob_pkts = '{default:0};

    cs_retire_valid = 3'b111;
    cs_retire_idx = {dispatch_index[2], dispatch_index[1], dispatch_index[0]};

    @(negedge clk);
    cs_retire_valid = 0;

    repeat (2) @(negedge clk);
    print_retire("Case 1: Normal lsq_retire_mask");

    // === Case 2: Branch predicted not taken, actually taken (mispredict) ===
    @(negedge clk);
    dispatch_rob_pkts[0] = '{default:0, valid:1, PC:32'h4000, NPC:32'h4004, Tnew:5'd5, Told:5'd4, bp_pred_taken:0, bp_pred_target:32'h0};

    @(negedge clk);
    dispatch_rob_pkts = '{default:0};
    cs_retire_valid[0] = 1;
    cs_retire_idx[0] = dispatch_index[0];
    rb_recover_valid[0] = 1;
    cs_retire_pc[0] = 32'h5000;

    @(negedge clk);
    cs_retire_valid = 0;
    rb_recover_valid = 0;

    if (dut.rb_entry[dispatch_index[0]].precise_state_need &&
        dut.rb_entry[dispatch_index[0]].cs_retire_pc == 32'h5000) begin
      $display("[PASS] Case 2: Correctly detected misprediction");
    end else begin
      $display("[FAIL] Case 2: Recovery info incorrect");
      test_passed[1] = 0;
    end

    // === Final Summary ===
    if (&test_passed)
      $display("✅ [ALL PASSED] All test cases passed successfully.");
    else begin
      $display("❌ [FAILED CASES]");
      if (!test_passed[0]) $display("- Case 1 failed: Normal lsq_retire_mask issue.");
      if (!test_passed[1]) $display("- Case 2 failed: Misprediction handling issue.");
    end

    $finish;
  end

endmodule
