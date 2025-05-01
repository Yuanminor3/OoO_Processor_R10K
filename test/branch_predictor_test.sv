`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module bp_test;

  logic clock, reset;
  logic [2:0] dispatch_EN, fetch_EN;
  logic update_EN;
  logic [`XLEN-1:0] update_pc, update_target;
  logic update_direction;
  logic [2:0][`XLEN-1:0] dispatch_pc, fetch_pc;
  logic [2:0] predict_found, predict_direction;
  logic [2:0][`XLEN-1:0] predict_pc;

  logic test_passed;

  branch_predictor bp (
    .clock, .reset,
    .dispatch_EN, .dispatch_pc,
    .update_EN, .update_pc, .update_direction, .update_target,
    .fetch_EN, .fetch_pc,
    .predict_found, .predict_direction, .predict_pc
  );

  // Clock generation
  always #5 clock = ~clock;

  initial begin
    clock = 0;
    reset = 1;
    dispatch_EN = 0;
    fetch_EN = 0;
    dispatch_pc = '{default:0};
    fetch_pc = '{default:0};
    update_EN = 0;
    update_pc = 0;
    update_direction = 0;
    update_target = 0;
    test_passed = 1;

    @(negedge clock); reset = 0;

    // ----------- Case 0: Fetch cold miss ------------
    @(negedge clock);
    fetch_EN[0] = 1;
    fetch_pc[0] = 32'h1000;

    @(negedge clock);
    if (!predict_found[0])
      $display("[PASS] Case 0: Cold fetch miss as expected");
    else begin
      $display("[FAIL] Case 0: Unexpected prediction found");
      test_passed = 0;
    end
    fetch_EN = 0;

    // ---------- Summary ----------
    if (test_passed)
      $display("✅ [ALL PASSED] Cold miss test passed.");
    else
      $display("❌ [FAILED] Case 0 failed: Unexpected prediction.");

    $finish;
  end
endmodule
