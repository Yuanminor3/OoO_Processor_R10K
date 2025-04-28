`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

// Needed macros
`ifndef RV32_signext_Iimm
`define RV32_signext_Iimm(inst) {{20{inst[31]}}, inst[31:20]}
`define RV32_signext_Bimm(inst) {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
`define RV32_signext_Jimm(inst) {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}
`endif

module fu_load_tb;

  logic clock, reset;
  logic complete_stall;
  ISSUE_FU_PACKET fu_packet_in;
  logic fu_ready, want_to_complete;
  FU_COMPLETE_PACKET fu_packet_out;
  LOAD_SQ_PACKET sq_lookup;
  SQ_LOAD_PACKET sq_result;
  logic [`XLEN-1:0] addr;
  logic cache_read_EN;
  logic [`XLEN-1:0] cache_data_in;
  logic is_hit;
  logic broadcast_en;
  logic [`XLEN-1:0] broadcast_data;

  fu_load dut (
    .clock(clock),
    .reset(reset),
    .complete_stall(complete_stall),
    .fu_packet_in(fu_packet_in),
    .fu_ready(fu_ready),
    .want_to_complete(want_to_complete),
    .fu_packet_out(fu_packet_out),
    .sq_lookup(sq_lookup),
    .sq_result(sq_result),
    .addr(addr),
    .cache_read_EN(cache_read_EN),
    .cache_data_in(cache_data_in),
    .is_hit(is_hit),
    .broadcast_en(broadcast_en),
    .broadcast_data(broadcast_data)
  );

  int pass_count = 0;
  int fail_count = 0;
  int total_count = 0;

  // Clock generation
  always #5 clock = ~clock;

  task run_load_test(
    input LS_SELECT load_type,
    input [`XLEN-1:0] base_addr,
    input [`XLEN-1:0] fake_cache_data,
    input [`XLEN-1:0] expected_value,
    input string test_name
  );
    begin
      // Step 1: Set up load instruction
      fu_packet_in.valid = 1;
      fu_packet_in.op_sel.ls = load_type;
      fu_packet_in.opa_select = OPA_IS_RS1;
      fu_packet_in.opb_select = OPB_IS_I_IMM;
      fu_packet_in.r1_value = base_addr;
      fu_packet_in.inst = 32'b0;
      fu_packet_in.dest_pr = 5'd1;
      fu_packet_in.rob_entry = 5'd2;
      sq_result = '0;
      complete_stall = 0;

      @(posedge clock);
      fu_packet_in.valid = 0;

      // Step 2: Simulate cache response after seeing cache_read_EN
      @(posedge clock);
      if (cache_read_EN) begin
        cache_data_in <= fake_cache_data;
        is_hit <= 1;
      end

      @(posedge clock);
      is_hit <= 0;

      // Step 3: Check result
      @(posedge clock);
      total_count++;
      if (fu_packet_out.dest_value === expected_value) begin
        $display("[PASS] %s | Got: 0x%h", test_name, fu_packet_out.dest_value);
        pass_count++;
      end else begin
        $display("[FAIL] %s | Got: 0x%h Expected: 0x%h", test_name, fu_packet_out.dest_value, expected_value);
        fail_count++;
      end

      @(posedge clock);
    end
  endtask

  initial begin
    // Initialize
    clock = 0;
    reset = 1;
    fu_packet_in = '0;
    sq_result = '0;
    cache_data_in = '0;
    is_hit = 0;
    broadcast_en = 0;
    broadcast_data = 0;

    // Reset phase
    repeat(2) @(posedge clock);
    reset = 0;
    @(posedge clock);

    // Run tests
    run_load_test(LB,  32'h1000_0000, 32'h11223344, 32'h00000044, "LB load 0x44");
    run_load_test(LH,  32'h1000_0004, 32'h00007F00, 32'h00007F00, "LH load 0x7F00");
    run_load_test(LW,  32'h1000_0008, 32'hDEADBEEF, 32'hDEADBEEF, "LW load 0xDEADBEEF");
    run_load_test(LBU, 32'h1000_000C, 32'hFF223344, 32'h00000044, "LBU load 0x44");
    run_load_test(LHU, 32'h1000_0010, 32'h0000ABCD, 32'h0000ABCD, "LHU load 0xABCD");

    // Finish
    #20;
    $display("====================================");
    if (fail_count == 0)
      $display("ALL TESTS PASSED (%0d tests)", pass_count);
    else
      $display("%0d PASSED, %0d FAILED", pass_count, fail_count);
    $display("====================================");

    $finish;
  end

endmodule
