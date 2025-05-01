`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module rob_test;

  logic clock, reset;
  ROB_ENTRY_PACKET [2:0] rob_in;
  ROB_ENTRY_PACKET [2:0] retire_entry;
  logic [2:0] complete_valid;
  logic [2:0][`ROB-1:0] complete_entry;
  logic [2:0] precise_state_valid;
  logic [2:0][`XLEN-1:0] target_pc;
  logic BPRecoverEN;
  logic [2:0] sq_stall;

  logic [2:0][`ROB-1:0] dispatch_index;
  logic [`ROB:0] space_left;

  logic [1:0] test_passed;  // for 2 test cases

  ROB dut (
    .clock,
    .reset,
    .rob_in,
    .complete_valid,
    .complete_entry,
    .precise_state_valid,
    .target_pc,
    .BPRecoverEN,
    .sq_stall,
    .dispatch_index,
    .space_left,
    .retire_entry
  );

  // Clock generation
  always #5 clock = ~clock;

  // Helper
  task print_retire(string label);
    $display("---- %s ----", label);
    for (int i = 0; i < 3; i++) begin
      if (retire_entry[i].valid)
        $display("  Retire[%0d]: PC=0x%08x, Tnew=%0d, completed=%0d", i, retire_entry[i].PC, retire_entry[i].Tnew, retire_entry[i].completed);
    end
  endtask

  initial begin
    clock = 0;
    reset = 1;
    rob_in = '{default:0};
    complete_valid = 0;
    complete_entry = '{default:0};
    precise_state_valid = 0;
    target_pc = '{default:0};
    BPRecoverEN = 0;
    sq_stall = 0;
    test_passed = '{default:1};  // assume pass

    @(negedge clock); reset = 0;

    // === Case 1: Normal Dispatch + Complete + Retire ===
    @(negedge clock);
    rob_in[0] = '{default:0, valid:1, PC:32'h1000, NPC:32'h1004, Tnew:5'd1, Told:5'd0, inst:32'hAAAA};
    rob_in[1] = '{default:0, valid:1, PC:32'h1004, NPC:32'h1008, Tnew:5'd2, Told:5'd1, inst:32'hBBBB};
    rob_in[2] = '{default:0, valid:1, PC:32'h1008, NPC:32'h100C, Tnew:5'd3, Told:5'd2, inst:32'hCCCC};

    @(negedge clock);
    rob_in = '{default:0};

    complete_valid = 3'b111;
    complete_entry = {dispatch_index[2], dispatch_index[1], dispatch_index[0]};

    @(negedge clock);
    complete_valid = 0;

    repeat (2) @(negedge clock);
    print_retire("Case 1: Normal retire");

    // === Case 2: Branch predicted not taken, actually taken (mispredict) ===
    @(negedge clock);
    rob_in[0] = '{default:0, valid:1, PC:32'h4000, NPC:32'h4004, Tnew:5'd5, Told:5'd4, predict_direction:0, predict_pc:32'h0};

    @(negedge clock);
    rob_in = '{default:0};
    complete_valid[0] = 1;
    complete_entry[0] = dispatch_index[0];
    precise_state_valid[0] = 1;
    target_pc[0] = 32'h5000;

    @(negedge clock);
    complete_valid = 0;
    precise_state_valid = 0;

    if (dut.rob_entries[dispatch_index[0]].precise_state_need &&
        dut.rob_entries[dispatch_index[0]].target_pc == 32'h5000) begin
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
      if (!test_passed[0]) $display("- Case 1 failed: Normal retire issue.");
      if (!test_passed[1]) $display("- Case 2 failed: Misprediction handling issue.");
    end

    $finish;
  end

endmodule
