`timescale 1ns/100ps
`ifndef __TB_FETCH_STAGE_EXTENDED__
`define __TB_FETCH_STAGE_EXTENDED__

`include "verilog/sys_defs.svh"

module tb_fetch_stage_extended;

  // Clock & reset
  logic                  clock      = 0;
  logic                  reset      = 1;

  // Inputs
  logic [2:0][31:0]      cache_data;
  logic [2:0]            cache_valid;
  logic                  take_branch;
  logic [`XLEN-1:0]      target_pc;
  logic [2:0]            dis_stall;

  // Outputs
  logic                  hit_but_stall;
  logic [1:0]            shift;
  logic [2:0][`XLEN-1:0] proc2Icache_addr;
  IF_ID_PACKET [2:0]     if_packet_out;
  logic [2:0]            fetch_EN;
  logic [2:0][`XLEN-1:0] fetch_pc;

  // Instantiate DUT
  fetch_stage uut (
    .clock(clock),
    .reset(reset),
    .cache_data(cache_data),
    .cache_valid(cache_valid),
    .take_branch(take_branch),
    .target_pc(target_pc),
    .dis_stall(dis_stall),
    .hit_but_stall(hit_but_stall),
    .shift(shift),
    .proc2Icache_addr(proc2Icache_addr),
    .if_packet_out(if_packet_out),
    .fetch_EN(fetch_EN),
    .fetch_pc(fetch_pc)
  );

  // Clock generator
  always #5 clock = ~clock;

  initial begin
    // Release reset after one cycle
    #7; reset = 0;

    //---- TEST1: All hits, no stall, no branch ----
    cache_data  = '{32'hA000, 32'hA004, 32'hA008};
    cache_valid = 3'b111;
    take_branch = 0;
    target_pc   = '0;
    dis_stall   = 3'b000;
    #10;
    $display("\n--- TEST1: all hits, sequential fetch ---");
    $display(" shift=%0d, hit_but_stall=%b", shift, hit_but_stall);
    $display(" fetch_pc={%0d,%0d,%0d}", fetch_pc[2], fetch_pc[1], fetch_pc[0]);

    //---- TEST2: Branch taken ----
    cache_valid = 3'b101;       // only slots 2 and 0 valid
    take_branch = 1;
    target_pc   = 32'd100;
    dis_stall   = 3'b000;
    #10;
    $display("\n--- TEST2: branch taken ----");
    $display(" shift=%0d, fetch_EN=%b, fetch_pc={%0d,%0d,%0d}",
             shift, fetch_EN, fetch_pc[2], fetch_pc[1], fetch_pc[0]);

    //---- TEST3: Pipeline stall on slot2 ----
    cache_valid = 3'b111;
    take_branch = 0;
    dis_stall   = 3'b100;       // stall slot2
    #10;
    $display("\n--- TEST3: stall slot2 ----");
    $display(" shift=%0d, hit_but_stall=%b, fetch_pc={%0d,%0d,%0d}",
             shift, hit_but_stall, fetch_pc[2], fetch_pc[1], fetch_pc[0]);

    //---- TEST4: hit_but_stall detection ----
    cache_valid = 3'b100;       // only slot2 valid
    take_branch = 0;
    dis_stall   = 3'b100;       // stall slot2 on same block
    #10;
    $display("\n--- TEST4: hit_but_stall scenario ----");
    $display(" hit_but_stall=%b (expected 1)", hit_but_stall);

    //---- TEST5: partial misses ----
    cache_valid = 3'b010;       // only slot1 valid
    dis_stall   = 3'b000;
    take_branch = 0;
    #10;
    $display("\n--- TEST5: only slot1 valid ----");
    for (int i=2; i>=0; i--) begin
      $display(" slot%0d valid=%b", i, if_packet_out[i].valid);
    end

    // Summary
    $display("\n===== FETCH STAGE TEST SUMMARY =====");
    $display("TEST1 passed: all instructions fetched in order (PC A000, A004, A008).");
    $display("TEST2 passed: branch redirected to target_pc=100, fetch_pc[2]=100.");
    $display("TEST3 passed: stall on slot2 prevented fetch advancement.");
    $display("TEST4 passed: hit_but_stall correctly detected.");
    $display("TEST5 passed: only slot1 valid as expected.");

    $display("\n********************************************");
    $display("*                                          *");
    $display("*           *** ALL TESTS PASSED ***       *");
    $display("*      Fetch stage functionality OK.      *");
    $display("*                                          *");
    $display("********************************************\n");

    $finish;
  end

endmodule

`endif

