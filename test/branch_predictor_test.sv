`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module bp_test;

  logic clk, rst;
  logic [2:0] bp_disp_enable, bp_fetch_enable;
  logic bs_upd_en;
  logic [`SYS_XLEN-1:0] bs_upd_pc, bs_upd_target;
  logic bs_upd_taken;
  logic [2:0][`SYS_XLEN-1:0] bp_disp_addr, bp_fetch_addr;
  logic [2:0] bp_pred_valid, bp_pred_taken;
  logic [2:0][`SYS_XLEN-1:0] bp_pred_target;

  logic test_passed;

  branch_predictor bp (
    .clk, .rst,
    .bp_disp_enable, .bp_disp_addr,
    .bs_upd_en, .bs_upd_pc, .bs_upd_taken, .bs_upd_target,
    .bp_fetch_enable, .bp_fetch_addr,
    .bp_pred_valid, .bp_pred_taken, .bp_pred_target
  );

  // Clock generation
  always #5 clk = ~clk;

  initial begin
    clk = 0;
    rst = 1;
    bp_disp_enable = 0;
    bp_fetch_enable = 0;
    bp_disp_addr = '{default:0};
    bp_fetch_addr = '{default:0};
    bs_upd_en = 0;
    bs_upd_pc = 0;
    bs_upd_taken = 0;
    bs_upd_target = 0;
    test_passed = 1;

    @(negedge clk); rst = 0;

    // ----------- Case 0: Fetch cold miss ------------
    @(negedge clk);
    bp_fetch_enable[0] = 1;
    bp_fetch_addr[0] = 32'h1000;

    @(negedge clk);
    if (!bp_pred_valid[0])
      $display("[PASS] Case 0: Cold fetch miss as expected");
    else begin
      $display("[FAIL] Case 0: Unexpected prediction found");
      test_passed = 0;
    end
    bp_fetch_enable = 0;

    // ---------- Summary ----------
    if (test_passed)
      $display("✅ [ALL PASSED] Cold miss test passed.");
    else
      $display("❌ [FAILED] Case 0 failed: Unexpected prediction.");

    $finish;
  end
endmodule
