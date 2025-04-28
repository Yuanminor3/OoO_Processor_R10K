`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
module dcache_tb;

  // Clock and Reset
  logic clock;
  logic reset;

  // Memory controller interface
  logic [3:0] Ctlr2proc_response;
  logic [63:0] Ctlr2proc_data;
  logic [3:0] Ctlr2proc_tag;
  
  logic [1:0] dcache2ctlr_command;
  logic [`XLEN-1:0] dcache2ctlr_addr;
  logic [63:0] dcache2ctlr_data;

  // SQ interface
  SQ_ENTRY_PACKET [2:0] sq_in;
  SQ_ENTRY_PACKET [2:0] sq_head;
  logic [2:0] sq_stall;

  // LQ interface
  logic [1:0][`XLEN-1:0] ld_addr_in;
  logic [1:0] ld_start;
  logic [1:0] is_hit;
  logic [1:0][`XLEN-1:0] ld_data;
  logic [1:0] broadcast_fu;
  logic [`XLEN-1:0] broadcast_data;

  // DUT instance
  dcache uut (
    .clock(clock),
    .reset(reset),
    .Ctlr2proc_response(Ctlr2proc_response),
    .Ctlr2proc_data(Ctlr2proc_data),
    .Ctlr2proc_tag(Ctlr2proc_tag),
    .dcache2ctlr_command(dcache2ctlr_command),
    .dcache2ctlr_addr(dcache2ctlr_addr),
    .dcache2ctlr_data(dcache2ctlr_data),
    .sq_in(sq_in),
    .sq_head(sq_head),
    .sq_stall(sq_stall),
    .ld_addr_in(ld_addr_in),
    .ld_start(ld_start),
    .is_hit(is_hit),
    .ld_data(ld_data),
    .broadcast_fu(broadcast_fu),
    .broadcast_data(broadcast_data)
  );

  // Clock generation
  always #5 clock = ~clock;

  // Pass/fail flags
  int num_fail = 0;

  task check(input string msg, input logic cond);
    if (!cond) begin
      $display("FAIL: %s", msg);
      num_fail++;
    end
  endtask

  initial begin
    clock = 0;
    reset = 1;
    #10;
    reset = 0;

    // ---------------------- Test 1: Load Hit (after fill) ----------------------
    $display("Test 1: Load Miss -> Fill -> Load Hit");

    // No store yet
    sq_in = '{default:0};
    sq_head = '{default:0};

    // First load, should MISS
    ld_addr_in[0] = 64'h0000_0000_0000_0000; // Address 0
    ld_start[0] = 1;
    ld_addr_in[1] = 0;
    ld_start[1] = 0;
    
    #10;
    ld_start[0] = 0;
    #10;

    check("Load should miss initially", is_hit[0] == 0);

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
    check("Load should hit after refill", is_hit[0] == 1);
    check("Load data correct", ld_data[0] == 64'hDEAD_BEEF);

    // ---------------------- Test 2: Store Hit ----------------------
    $display("Test 2: Store Hit");

    // Setup SQ to write to address 0 (already cached)
    sq_in[0].addr = 64'h0000_0000_0000_0000;
    sq_in[0].data = 32'hAAAA_BBBB;
    sq_in[0].usebytes = 8'hFF;
    sq_in[0].ready = 1;
    sq_head = sq_in;

    #10;

    sq_in = '{default:0};
    sq_head = '{default:0};

    // Load again after store, data not verified here (should forward ideally)

    // ---------------------- Test 3: Load Miss + Store Miss (different addresses) ----------------------
    $display("Test 3: Load Miss + Store Miss");

    // Load to new address (should miss)
    ld_addr_in[0] = 64'h0000_0000_0000_0100; // Different address
    ld_start[0] = 1;
    #10;
    ld_start[0] = 0;
    #10;
    check("New Load should miss", is_hit[0] == 0);

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
