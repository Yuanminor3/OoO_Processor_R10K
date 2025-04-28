`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module fu_mult_test;

  // Clock and Reset
  logic clock, reset;
  always #5 clock = ~clock; // 100MHz

  // DUT IO
  logic complete_stall;
  ISSUE_FU_PACKET fu_packet_in;
  logic fu_ready, want_to_complete;
  FU_COMPLETE_PACKET fu_packet_out;

  // Instantiate DUT
  fu_mult dut (
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall),
    .fu_packet_in(fu_packet_in),
    .fu_ready(fu_ready),
    .want_to_complete(want_to_complete),
    .fu_packet_out(fu_packet_out)
  );

  // Test control
  int num_passed = 0;
  int num_failed = 0;

  task check_result(input [`XLEN-1:0] expected, input string test_name);
    if (fu_packet_out.dest_value === expected) begin
      $display("PASS: %s | Result = 0x%h", test_name, fu_packet_out.dest_value);
      num_passed++;
    end else begin
      $display("FAIL: %s | Got 0x%h, Expected 0x%h", test_name, fu_packet_out.dest_value, expected);
      num_failed++;
    end
  endtask

  // Helper: wait until want_to_complete goes high
  task wait_for_completion();
    while (~want_to_complete) @(posedge clock);
    #1; // small delay to ensure fu_packet_out is stable
  endtask

  initial begin
    clock = 0;
    reset = 1;
    complete_stall = 0;
    fu_packet_in = 0;
    #20;
    reset = 0;

    // =============== Test Cases Start ===============

    // 1. MULT (signed low 32-bit product)
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.mult = MULT;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = 32'd7;
    fu_packet_in.r2_value = 32'd5;
    fu_packet_in.dest_pr = 5'd1;
    fu_packet_in.rob_entry = 5'd2;
    #10;
    fu_packet_in.valid = 0;
    wait_for_completion();
    check_result(32'd35, "MULT 7 * 5 = 35");

    // 2. MULH (signed high 32-bit product)
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.mult = MULH;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = -32'd2;
    fu_packet_in.r2_value = 32'd3;
    fu_packet_in.dest_pr = 5'd3;
    fu_packet_in.rob_entry = 5'd4;
    #10;
    fu_packet_in.valid = 0;
    wait_for_completion();
    check_result(32'hFFFFFFFF, "MULH -2 * 3 high bits = -1 (0xFFFFFFFF)");

    // 3. MULHU (unsigned high 32-bit product)
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.mult = MULHU;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = 32'hFFFF0000;
    fu_packet_in.r2_value = 32'h0000FFFF;
    fu_packet_in.dest_pr = 5'd5;
    fu_packet_in.rob_entry = 5'd6;
    #10;
    fu_packet_in.valid = 0;
    wait_for_completion();
    check_result(32'hFFFE, "MULHU 0xFFFF0000 * 0x0000FFFF high bits = 0xFFFE");

    // 4. MULHSU (signed * unsigned high 32-bit product)
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.mult = MULHSU;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = -32'd1;
    fu_packet_in.r2_value = 32'h0000FFFF;
    fu_packet_in.dest_pr = 5'd7;
    fu_packet_in.rob_entry = 5'd8;
    #10;
    fu_packet_in.valid = 0;
    wait_for_completion();
    check_result(32'hFFFFFFFF, "MULHSU -1 * 0xFFFF high bits = 0xFFFFFFFF");

    // =============== Test Cases End ===============

    #20;
    $display("==========================");
    if (num_failed == 0) begin
      $display("ALL TESTS PASSED (%0d tests)", num_passed);
    end else begin
      $display("%0d TESTS PASSED, %0d TESTS FAILED", num_passed, num_failed);
    end
    $display("==========================");
    $finish;
  end

endmodule
