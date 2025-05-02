`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module fu_mult_test;

  // Clock and Reset
  logic clk, rst;
  always #5 clk = ~clk; // 100MHz

  // DUT IO
  logic bs_hazard;
  ISSUE_FU_PACKET bs_in_pkt;
  logic rsb_fu_ready, fum_complete_req;
  FU_COMPLETE_PACKET bs_out_pkt;

  // Instantiate DUT
  fu_mult dut (
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard),
    .bs_in_pkt(bs_in_pkt),
    .rsb_fu_ready(rsb_fu_ready),
    .fum_complete_req(fum_complete_req),
    .bs_out_pkt(bs_out_pkt)
  );

  // Test control
  int num_passed = 0;
  int num_failed = 0;

  task check_result(input [`SYS_XLEN-1:0] expected, input string test_name);
    if (bs_out_pkt.dest_value === expected) begin
      $display("PASS: %s | Result = 0x%h", test_name, bs_out_pkt.dest_value);
      num_passed++;
    end else begin
      $display("FAIL: %s | Got 0x%h, Expected 0x%h", test_name, bs_out_pkt.dest_value, expected);
      num_failed++;
    end
  endtask

  // Helper: wait until fum_complete_req goes high
  task wait_for_completion();
    while (~fum_complete_req) @(posedge clk);
    #1; // small delay to ensure bs_out_pkt is stable
  endtask

  initial begin
    clk = 0;
    rst = 1;
    bs_hazard = 0;
    bs_in_pkt = 0;
    #20;
    rst = 0;

    // =============== Test Cases Start ===============

    // 1. MULT (signed low 32-bit product)
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.mult = MULT;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = 32'd7;
    bs_in_pkt.r2_value = 32'd5;
    bs_in_pkt.dispatch_allocated_prs = 5'd1;
    bs_in_pkt.rob_entry = 5'd2;
    #10;
    bs_in_pkt.valid = 0;
    wait_for_completion();
    check_result(32'd35, "MULT 7 * 5 = 35");

    // 2. MULH (signed high 32-bit product)
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.mult = MULH;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = -32'd2;
    bs_in_pkt.r2_value = 32'd3;
    bs_in_pkt.dispatch_allocated_prs = 5'd3;
    bs_in_pkt.rob_entry = 5'd4;
    #10;
    bs_in_pkt.valid = 0;
    wait_for_completion();
    check_result(32'hFFFFFFFF, "MULH -2 * 3 high bits = -1 (0xFFFFFFFF)");

    // 3. MULHU (unsigned high 32-bit product)
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.mult = MULHU;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = 32'hFFFF0000;
    bs_in_pkt.r2_value = 32'h0000FFFF;
    bs_in_pkt.dispatch_allocated_prs = 5'd5;
    bs_in_pkt.rob_entry = 5'd6;
    #10;
    bs_in_pkt.valid = 0;
    wait_for_completion();
    check_result(32'hFFFE, "MULHU 0xFFFF0000 * 0x0000FFFF high bits = 0xFFFE");

    // 4. MULHSU (signed * unsigned high 32-bit product)
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.mult = MULHSU;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = -32'd1;
    bs_in_pkt.r2_value = 32'h0000FFFF;
    bs_in_pkt.dispatch_allocated_prs = 5'd7;
    bs_in_pkt.rob_entry = 5'd8;
    #10;
    bs_in_pkt.valid = 0;
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
