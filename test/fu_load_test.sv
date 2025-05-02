`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

// Needed macros
`ifndef RV32_signext_Iimm
`define RV32_signext_Iimm(inst) {{20{inst[31]}}, inst[31:20]}
`define RV32_signext_Bimm(inst) {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
`define RV32_signext_Jimm(inst) {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}
`endif

module fu_load_tb;

  logic clk, rst;
  logic bs_hazard;
  ISSUE_FU_PACKET bs_in_pkt;
  logic rsb_fu_ready, fum_complete_req;
  FU_COMPLETE_PACKET bs_out_pkt;
  LOAD_SQ_PACKET ld_sq_request;
  SQ_LOAD_PACKET ld_sq_response;
  logic [`SYS_XLEN-1:0] addr;
  logic ld_cache_read_enable;
  logic [`SYS_XLEN-1:0] ld_cache_data_in;
  logic exs_dcache_hit_flags;
  logic broadcast_en;
  logic [`SYS_XLEN-1:0] exs_dcache_brdcast_data;

  fu_load dut (
    .clk(clk),
    .rst(rst),
    .bs_hazard(bs_hazard),
    .bs_in_pkt(bs_in_pkt),
    .rsb_fu_ready(rsb_fu_ready),
    .fum_complete_req(fum_complete_req),
    .bs_out_pkt(bs_out_pkt),
    .ld_sq_request(ld_sq_request),
    .ld_sq_response(ld_sq_response),
    .addr(addr),
    .ld_cache_read_enable(ld_cache_read_enable),
    .ld_cache_data_in(ld_cache_data_in),
    .exs_dcache_hit_flags(exs_dcache_hit_flags),
    .broadcast_en(broadcast_en),
    .exs_dcache_brdcast_data(exs_dcache_brdcast_data)
  );

  int pass_count = 0;
  int fail_count = 0;
  int total_count = 0;

  // Clock generation
  always #5 clk = ~clk;

  task run_load_test(
    input LS_SELECT load_type,
    input [`SYS_XLEN-1:0] base_addr,
    input [`SYS_XLEN-1:0] fake_cache_data,
    input [`SYS_XLEN-1:0] expected_value,
    input string test_name
  );
    begin
      // Step 1: Set up load instruction
      bs_in_pkt.valid = 1;
      bs_in_pkt.dec_fu_opcode.ls = load_type;
      bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
      bs_in_pkt.dec_operandB_mux = OPB_IS_I_IMM;
      bs_in_pkt.r1_value = base_addr;
      bs_in_pkt.inst = 32'b0;
      bs_in_pkt.dispatch_allocated_prs = 5'd1;
      bs_in_pkt.rob_entry = 5'd2;
      ld_sq_response = '0;
      bs_hazard = 0;

      @(posedge clk);
      bs_in_pkt.valid = 0;

      // Step 2: Simulate cache response after seeing ld_cache_read_enable
      @(posedge clk);
      if (ld_cache_read_enable) begin
        ld_cache_data_in <= fake_cache_data;
        exs_dcache_hit_flags <= 1;
      end

      @(posedge clk);
      exs_dcache_hit_flags <= 0;

      // Step 3: Check result
      @(posedge clk);
      total_count++;
      if (bs_out_pkt.dest_value === expected_value) begin
        $display("[PASS] %s | Got: 0x%h", test_name, bs_out_pkt.dest_value);
        pass_count++;
      end else begin
        $display("[FAIL] %s | Got: 0x%h Expected: 0x%h", test_name, bs_out_pkt.dest_value, expected_value);
        fail_count++;
      end

      @(posedge clk);
    end
  endtask

  initial begin
    // Initialize
    clk = 0;
    rst = 1;
    bs_in_pkt = '0;
    ld_sq_response = '0;
    ld_cache_data_in = '0;
    exs_dcache_hit_flags = 0;
    broadcast_en = 0;
    exs_dcache_brdcast_data = 0;

    // Reset phase
    repeat(2) @(posedge clk);
    rst = 0;
    @(posedge clk);

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
