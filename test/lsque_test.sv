`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ps.sv"
module SQ_tb;

  parameter CLK_PERIOD = 10;

  // Clock + Reset
  logic clock;
  logic reset;

  // DUT I/Os
  logic [2:0] dispatch, retire, exe_valid;
  SQ_ENTRY_PACKET [2:0] exe_store;
  logic [2:0][`LSQ-1:0] exe_idx;
  LOAD_SQ_PACKET [1:0] load_lookup;

  logic [2:0] stall;
  logic [2:0][`LSQ-1:0] tail_pos;
  SQ_LOAD_PACKET [1:0] load_forward;
  SQ_ENTRY_PACKET [2:0] cache_wb, sq_head;
  SQ_ENTRY_PACKET [0:2**`LSQ-1] sq_display;
  logic [`LSQ-1:0] head_dis;
  logic [2:0] test_passed;
  logic [`LSQ:0] filled_num_dis;

  SQ dut (
    .clock(clock),
    .reset(reset),
    .dispatch(dispatch),
    .stall(stall),
    .tail_pos(tail_pos),
    .exe_valid(exe_valid),
    .exe_store(exe_store),
    .exe_idx(exe_idx),
    .load_lookup(load_lookup),
    .load_forward(load_forward),
    .retire(retire),
    .cache_wb(cache_wb),
    .sq_head(sq_head),
    .sq_reg_out(sq_display),
    .head_out(head_dis)
  );

  always #(CLK_PERIOD/2) clock = ~clock;

  initial begin
    clock = 0;
    reset = 1;
    dispatch = 0;
    retire = 0;
    exe_valid = 0;
    exe_store = '{default:0};
    exe_idx = '{default:0};
    load_lookup = '{default:0};
    test_passed = 3'b111;

    #(2*CLK_PERIOD);
    reset = 0;
    #(CLK_PERIOD);

    // === Case 0: Dispatch two stores ===
    dispatch = 3'b011;
    #(CLK_PERIOD);
    dispatch = 0;

    exe_valid[0] = 1;
    exe_idx[0] = tail_pos[2];
    exe_store[0] = '{ready:1, usebytes:4'b1111, addr:32'h1000, data:32'hDEADBEEF};

    exe_valid[1] = 1;
    exe_idx[1] = tail_pos[1];
    exe_store[1] = '{ready:1, usebytes:4'b1111, addr:32'h1004, data:32'hCAFEBABE};

    #(CLK_PERIOD);
    exe_valid = 0;
    #(CLK_PERIOD);

    if (dut.filled_num !== 2)
      test_passed[0] = 0;

    // === Case 1: Retire 1 entry ===
    retire = 3'b001;
    #(CLK_PERIOD);
    retire = 0;
    #(CLK_PERIOD);

    if (dut.filled_num !== 1)
      test_passed[1] = 0;

    // === Summary ===
    if (&test_passed)
      $display("\n✅ [ALL PASSED] SQ module passed all test cases.");
    else begin
      $display("\n❌ [FAILED CASES]");
      if (!test_passed[0]) $display("- Case 0 failed: Dispatch");
      if (!test_passed[1]) $display("- Case 1 failed: Retirement check");
    end

    $finish;
  end
endmodule
