`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ps.sv"
module SQ_tb;

  parameter CLK_PERIOD = 10;

  // Clock + Reset
  logic clk;
  logic rst;

  // DUT I/Os
  logic [2:0] lsq_disp_mask, lsq_retire_mask, exs_store_flags;
  SQ_ENTRY_PACKET [2:0] exe_store_entries;
  logic [2:0][`SYS_LSQ_ADDR_WIDTH-1:0] exs_store_index;
  LOAD_SQ_PACKET [1:0] exs_load_req_pkts;

  logic [2:0] lsq_stall_mask;
  logic [2:0][`SYS_LSQ_ADDR_WIDTH-1:0] lsq_tail_alloc;
  SQ_LOAD_PACKET [1:0] exs_load2pkts;
  SQ_ENTRY_PACKET [2:0] lsq_wb_entry, lsq_head_entry;
  SQ_ENTRY_PACKET [0:2**`SYS_LSQ_ADDR_WIDTH-1] sq_display;
  logic [`SYS_LSQ_ADDR_WIDTH-1:0] head_dis;
  logic [2:0] test_passed;
  logic [`SYS_LSQ_ADDR_WIDTH:0] filled_num_dis;

  SQ dut (
    .clk(clk),
    .rst(rst),
    .lsq_disp_mask(lsq_disp_mask),
    .lsq_stall_mask(lsq_stall_mask),
    .lsq_tail_alloc(lsq_tail_alloc),
    .exs_store_flags(exs_store_flags),
    .exe_store_entries(exe_store_entries),
    .exs_store_index(exs_store_index),
    .exs_load_req_pkts(exs_load_req_pkts),
    .exs_load2pkts(exs_load2pkts),
    .lsq_retire_mask(lsq_retire_mask),
    .lsq_wb_entry(lsq_wb_entry),
    .lsq_head_entry(lsq_head_entry),
    .lsq_reg_snapshot(sq_display),
    .lsq_head_ptr(head_dis)
  );

  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    clk = 0;
    rst = 1;
    lsq_disp_mask = 0;
    lsq_retire_mask = 0;
    exs_store_flags = 0;
    exe_store_entries = '{default:0};
    exs_store_index = '{default:0};
    exs_load_req_pkts = '{default:0};
    test_passed = 3'b111;

    #(2*CLK_PERIOD);
    rst = 0;
    #(CLK_PERIOD);

    // === Case 0: Dispatch two stores ===
    lsq_disp_mask = 3'b011;
    #(CLK_PERIOD);
    lsq_disp_mask = 0;

    exs_store_flags[0] = 1;
    exs_store_index[0] = lsq_tail_alloc[2];
    exe_store_entries[0] = '{ready:1, usebytes:4'b1111, addr:32'h1000, data:32'hDEADBEEF};

    exs_store_flags[1] = 1;
    exs_store_index[1] = lsq_tail_alloc[1];
    exe_store_entries[1] = '{ready:1, usebytes:4'b1111, addr:32'h1004, data:32'hCAFEBABE};

    #(CLK_PERIOD);
    exs_store_flags = 0;
    #(CLK_PERIOD);

    if (dut.lsq_count !== 2)
      test_passed[0] = 0;

    // === Case 1: Retire 1 entry ===
    lsq_retire_mask = 3'b001;
    #(CLK_PERIOD);
    lsq_retire_mask = 0;
    #(CLK_PERIOD);

    if (dut.lsq_count !== 1)
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
