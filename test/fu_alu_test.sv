`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

`define RV32_signext_Iimm(inst) {{20{inst[31]}}, inst[31:20]}
`define RV32_signext_Simm(inst) {{20{inst[31]}}, inst[31:25], inst[11:7]}
`define RV32_signext_Bimm(inst) {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
`define RV32_signext_Uimm(inst) {inst[31:12], 12'b0}
`define RV32_signext_Jimm(inst) {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}


module fu_alu_test;

  // Clock and Reset
  logic clock, reset;
  always #5 clock = ~clock; // 100MHz clock

  // DUT inputs/outputs
  logic complete_stall;
  ISSUE_FU_PACKET fu_packet_in;
  logic fu_ready, want_to_complete;
  FU_COMPLETE_PACKET fu_packet_out;
  logic if_store;
  SQ_ENTRY_PACKET store_pckt;
  logic [`LSQ-1:0] sq_idx;

  // Instantiate DUT
  fu_alu dut (
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall),
    .fu_packet_in(fu_packet_in),
    .fu_ready(fu_ready),
    .want_to_complete(want_to_complete),
    .fu_packet_out(fu_packet_out),
    .if_store(if_store),
    .store_pckt(store_pckt),
    .sq_idx(sq_idx)
  );

  // Test Control
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

  initial begin
    clock = 0;
    reset = 1;
    complete_stall = 0;
    #10;
    reset = 0;

    // ========= Test Cases =========

    // 1. ADD test: 5 + 3 = 8
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.alu = ALU_ADD;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = 32'd5;
    fu_packet_in.r2_value = 32'd3;
    fu_packet_in.inst = '0;
    fu_packet_in.halt = 0;
    fu_packet_in.dest_pr = 5'd1;
    fu_packet_in.rob_entry = 5'd10;
    #10;
    check_result(32'd8, "ADD 5 + 3");

    // 2. SUB test: 10 - 7 = 3
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.alu = ALU_SUB;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = 32'd10;
    fu_packet_in.r2_value = 32'd7;
    #10;
    check_result(32'd3, "SUB 10 - 7");

    // 3. AND test: 0b1100 & 0b1010 = 0b1000
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.alu = ALU_AND;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = 32'b1100;
    fu_packet_in.r2_value = 32'b1010;
    #10;
    check_result(32'b1000, "AND 0b1100 & 0b1010");

    // 4. SLT test: signed 5 < -1 ? No (0)
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.alu = ALU_SLT;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = 32'd5;
    fu_packet_in.r2_value = -32'd1;
    #10;
    check_result(32'd0, "SLT 5 < -1");

    // 5. SLT test: signed -5 < 1 ? Yes (1)
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.alu = ALU_SLT;
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_RS2;
    fu_packet_in.r1_value = -32'd5;
    fu_packet_in.r2_value = 32'd1;
    #10;
    check_result(32'd1, "SLT -5 < 1");

    // 6. SW (Store Word) test: check if store output correct
    fu_packet_in.valid = 1;
    fu_packet_in.op_sel.alu = SW; // Store operation
    fu_packet_in.opa_select = OPA_IS_RS1;
    fu_packet_in.opb_select = OPB_IS_S_IMM;
    fu_packet_in.r1_value = 32'h1000_0000;
    fu_packet_in.r2_value = 32'hDEADBEEF;
    fu_packet_in.inst = {12'd4, 5'd0, 5'd0, 3'b000, 5'd0, 7'b0100011}; // Fake instruction format
    #10;

    if (if_store && store_pckt.data === 32'hDEADBEEF && store_pckt.usebytes === 4'b1111) begin
      $display("PASS: SW store data check | Data = 0x%h", store_pckt.data);
      num_passed++;
    end else begin
      $display("FAIL: SW store data check | Got data = 0x%h, usebytes = %b", store_pckt.data, store_pckt.usebytes);
      num_failed++;
    end

    // ========= End of Test Cases =========

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
