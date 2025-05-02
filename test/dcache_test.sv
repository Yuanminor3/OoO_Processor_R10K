`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
module dcache_tb;

  // Clock and Reset
  logic clk;
  logic rst;

  // Memory controller interface
  logic [3:0] Ctlr2proc_response;
  logic [63:0] Ctlr2proc_data;
  logic [3:0] Ctlr2proc_tag;
  
  logic [1:0] mc_dc_cmd;
  logic [`SYS_XLEN-1:0] mc_dc_addr;
  logic [63:0] mc_dc_wr_data;

  // SQ interface
  SQ_ENTRY_PACKET [2:0] sq_in;
  SQ_ENTRY_PACKET [2:0] lsq_head_entry;
  logic [2:0] rb_sq_stall;

  // LQ interface
  logic [1:0][`SYS_XLEN-1:0] ld_addr_in;
  logic [1:0] ld_start;
  logic [1:0] exs_dcache_hit_flags;
  logic [1:0][`SYS_XLEN-1:0] exs_dcache_resp_data;
  logic [1:0] exs_dcache_brdcast_mask;
  logic [`SYS_XLEN-1:0] exs_dcache_brdcast_data;

  // DUT instance
  dcache uut (
    .clk(clk),
    .rst(rst),
    .Ctlr2proc_response(Ctlr2proc_response),
    .Ctlr2proc_data(Ctlr2proc_data),
    .Ctlr2proc_tag(Ctlr2proc_tag),
    .mc_dc_cmd(mc_dc_cmd),
    .mc_dc_addr(mc_dc_addr),
    .mc_dc_wr_data(mc_dc_wr_data),
    .sq_in(sq_in),
    .lsq_head_entry(lsq_head_entry),
    .rb_sq_stall(rb_sq_stall),
    .ld_addr_in(ld_addr_in),
    .ld_start(ld_start),
    .exs_dcache_hit_flags(exs_dcache_hit_flags),
    .exs_dcache_resp_data(exs_dcache_resp_data),
    .exs_dcache_brdcast_mask(exs_dcache_brdcast_mask),
    .exs_dcache_brdcast_data(exs_dcache_brdcast_data)
  );

  // Clock generation
  always #5 clk = ~clk;

  // Pass/fail flags
  int num_fail = 0;

  task check(input string msg, input logic cond);
    if (!cond) begin
      $display("FAIL: %s", msg);
      num_fail++;
    end
  endtask

  initial begin
    clk = 0;
    rst = 1;
    #10;
    rst = 0;

    // ---------------------- Test 1: Load Hit (after fill) ----------------------
    $display("Test 1: Load Miss -> Fill -> Load Hit");

    // No store yet
    sq_in = '{default:0};
    lsq_head_entry = '{default:0};

    // First load, should MISS
    ld_addr_in[0] = 64'h0000_0000_0000_0000; // Address 0
    ld_start[0] = 1;
    ld_addr_in[1] = 0;
    ld_start[1] = 0;
    
    #10;
    ld_start[0] = 0;
    #10;

    check("Load should miss initially", exs_dcache_hit_flags[0] == 0);

    // Simulate memory controller returning data for address 0
    Ctlr2proc_response = 4'd1;
    Ctlr2proc_tag = 4'd1;
    Ctlr2proc_data = 64'hDEAD_BEEF_DEAD_BEEF;
    #10;
    Ctlr2proc_response = 4'd0;

    // Try load again, should HIT now
    ld_addr_in[0] = 64'h0000_0000_0000_0000;
    ld_start[0] = 1;
    #10;
    ld_start[0] = 0;
    #10;
    check("Load should hit after refill", exs_dcache_hit_flags[0] == 1);
    check("Load data correct", exs_dcache_resp_data[0] == 64'hDEAD_BEEF);

    // ---------------------- Test 2: Store Hit ----------------------
    $display("Test 2: Store Hit");

    // Setup SQ to write to address 0 (already cached)
    sq_in[0].addr = 64'h0000_0000_0000_0000;
    sq_in[0].data = 32'hAAAA_BBBB;
    sq_in[0].usebytes = 8'hFF;
    sq_in[0].ready = 1;
    lsq_head_entry = sq_in;

    #10;

    sq_in = '{default:0};
    lsq_head_entry = '{default:0};

    // Load again after store, data not verified here (should forward ideally)

    // ---------------------- Test 3: Load Miss + Store Miss (different addresses) ----------------------
    $display("Test 3: Load Miss + Store Miss");

    // Load to new address (should miss)
    ld_addr_in[0] = 64'h0000_0000_0000_0100; // Different address
    ld_start[0] = 1;
    #10;
    ld_start[0] = 0;
    #10;
    check("New Load should miss", exs_dcache_hit_flags[0] == 0);

    // Simulate memory controller returning new data
    Ctlr2proc_response = 4'd2;
    Ctlr2proc_tag = 4'd2;
    Ctlr2proc_data = 64'h1234_5678_ABCD_EF01;
    #10;
    Ctlr2proc_response = 4'd0;
    #10;

    // ---------------------- Test 4: Eviction + Dirty Writeback ----------------------
    $display("Test 4: Eviction and Dirty Writeback");

    // Force cache to fill different sets
    for (int i = 0; i < 16; i++) begin
      ld_addr_in[0] = {48'd0, i, 3'd0}; // Different indexes
      ld_start[0] = 1;
      #10;
      ld_start[0] = 0;
      #10;
      Ctlr2proc_response = 4'd3;
      Ctlr2proc_tag = 4'd3;
      Ctlr2proc_data = {32'h5678_5678, 32'h1234_1234};
      #10;
      Ctlr2proc_response = 4'd0;
      #10;
    end

    // Should trigger evictions and dirty writeback

    // ---------------------- Test Finish ----------------------
    #20;

    if (num_fail == 0) begin
      $display("\n===========================================");
      $display("               PASS ALL TESTS!");
      $display("===========================================\n");
    end else begin
      $display("\n===========================================");
      $display("              FAIL %0d TESTS!", num_fail);
      $display("===========================================\n");
    end

    $finish;
  end

endmodule
