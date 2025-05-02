`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

`define RV32_signext_Iimm(inst) {{20{inst[31]}}, inst[31:20]}
`define RV32_signext_Simm(inst) {{20{inst[31]}}, inst[31:25], inst[11:7]}
`define RV32_signext_Bimm(inst) {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
`define RV32_signext_Uimm(inst) {inst[31:12], 12'b0}
`define RV32_signext_Jimm(inst) {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}


module fu_alu_test;

  // Clock and Reset
  logic clk, rst;
  always #5 clk = ~clk; // 100MHz clk

  // DUT inputs/outputs
  logic bs_hazard;
  ISSUE_FU_PACKET bs_in_pkt;
  logic rsb_fu_ready, fum_complete_req;
  FU_COMPLETE_PACKET bs_out_pkt;
  logic alu_store_enable;
  SQ_ENTRY_PACKET alu_store_output_entry;
  logic [`SYS_LSQ_ADDR_WIDTH-1:0] alu_store_sq_index;

  // Instantiate DUT
  fu_alu dut (
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard),
    .bs_in_pkt(bs_in_pkt),
    .rsb_fu_ready(rsb_fu_ready),
    .fum_complete_req(fum_complete_req),
    .bs_out_pkt(bs_out_pkt),
    .alu_store_enable(alu_store_enable),
    .alu_store_output_entry(alu_store_output_entry),
    .alu_store_sq_index(alu_store_sq_index)
  );

  // Test Control
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

  initial begin
    clk = 0;
    rst = 1;
    bs_hazard = 0;
    #10;
    rst = 0;

    // ========= Test Cases =========

    // 1. ADD test: 5 + 3 = 8
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.alu = ALU_ADD;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = 32'd5;
    bs_in_pkt.r2_value = 32'd3;
    bs_in_pkt.inst = '0;
    bs_in_pkt.halt = 0;
    bs_in_pkt.dispatch_allocated_prs = 5'd1;
    bs_in_pkt.rob_entry = 5'd10;
    #10;
    check_result(32'd8, "ADD 5 + 3");

    // 2. SUB test: 10 - 7 = 3
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.alu = ALU_SUB;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = 32'd10;
    bs_in_pkt.r2_value = 32'd7;
    #10;
    check_result(32'd3, "SUB 10 - 7");

    // 3. AND test: 0b1100 & 0b1010 = 0b1000
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.alu = ALU_AND;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = 32'b1100;
    bs_in_pkt.r2_value = 32'b1010;
    #10;
    check_result(32'b1000, "AND 0b1100 & 0b1010");

    // 4. SLT test: signed 5 < -1 ? No (0)
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.alu = ALU_SLT;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = 32'd5;
    bs_in_pkt.r2_value = -32'd1;
    #10;
    check_result(32'd0, "SLT 5 < -1");

    // 5. SLT test: signed -5 < 1 ? Yes (1)
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.alu = ALU_SLT;
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_RS2;
    bs_in_pkt.r1_value = -32'd5;
    bs_in_pkt.r2_value = 32'd1;
    #10;
    check_result(32'd1, "SLT -5 < 1");

    // 6. SW (Store Word) test: check if store output correct
    bs_in_pkt.valid = 1;
    bs_in_pkt.dec_fu_opcode.alu = SW; // Store operation
    bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
    bs_in_pkt.dec_operandB_mux = OPB_IS_S_IMM;
    bs_in_pkt.r1_value = 32'h1000_0000;
    bs_in_pkt.r2_value = 32'hDEADBEEF;
    bs_in_pkt.inst = {12'd4, 5'd0, 5'd0, 3'b000, 5'd0, 7'b0100011}; // Fake instruction format
    #10;

    if (alu_store_enable && alu_store_output_entry.data === 32'hDEADBEEF && alu_store_output_entry.usebytes === 4'b1111) begin
      $display("PASS: SW store data check | Data = 0x%h", alu_store_output_entry.data);
      num_passed++;
    end else begin
      $display("FAIL: SW store data check | Got data = 0x%h, usebytes = %b", alu_store_output_entry.data, alu_store_output_entry.usebytes);
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
