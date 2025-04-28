`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module icache_tb_noprefetch;

  // Clock and Reset
  logic clock;
  logic reset;

  // Inputs to icache
  logic take_branch;
  logic [3:0] Imem2proc_response;
  logic [63:0] Imem2proc_data;
  logic [3:0] Imem2proc_tag;
  logic d_request;
  logic [1:0] shift;
  logic [2:0][63:0] proc2Icache_addr;
  logic [2:0][63:0] cachemem_data;
  logic [2:0] cachemem_valid;
  logic hit_but_stall;

  // Outputs from icache
  logic [1:0] proc2Imem_command;
  logic [63:0] proc2Imem_addr;
  logic [2:0][31:0] Icache_data_out;
  logic [2:0] Icache_valid_out;
  logic [2:0][4:0] current_index;
  logic [2:0][7:0] current_tag;
  logic [4:0] wr_index;
  logic [7:0] wr_tag;
  logic data_write_enable;

  // DUT instantiation
  icache uut (
    .clock(clock),
    .reset(reset),
    .take_branch(take_branch),
    .Imem2proc_response(Imem2proc_response),
    .Imem2proc_data(Imem2proc_data),
    .Imem2proc_tag(Imem2proc_tag),
    .d_request(d_request),
    .shift(shift),
    .proc2Icache_addr(proc2Icache_addr),
    .cachemem_data(cachemem_data),
    .cachemem_valid(cachemem_valid),
    .hit_but_stall(hit_but_stall),
    .proc2Imem_command(proc2Imem_command),
    .proc2Imem_addr(proc2Imem_addr),
    .Icache_data_out(Icache_data_out),
    .Icache_valid_out(Icache_valid_out),
    .current_index(current_index),
    .current_tag(current_tag),
    .wr_index(wr_index),
    .wr_tag(wr_tag),
    .data_write_enable(data_write_enable)
  );

  // Clock generation
  always #5 clock = ~clock;

  // Test result counters
  int num_fail = 0;

  task check(input string message, input logic condition);
    if (!condition) begin
      $display("FAIL: %s", message);
      num_fail++;
    end
  endtask

  initial begin
    clock = 0;
    reset = 1;
    #10;
    reset = 0;

    $display("\n=== Test 1: Initial load miss triggers memory request ===");

    d_request = 0;
    take_branch = 0;
    hit_but_stall = 0;
    shift = 2'd0;
    proc2Icache_addr[2] = 64'h0000_0000_0000_0000;
    cachemem_valid = 3'b000; // Cache invalid
    cachemem_data = '{default:0};

    #10;
    check("Memory request issued", proc2Imem_command == 2'b01);

    $display("\n=== Test 2: After memory return, cache hit ===");

    // Simulate memory controller return
    Imem2proc_response = 4'd2;
    Imem2proc_tag = 4'd2;
    Imem2proc_data = 64'hDEAD_BEEF_DEAD_BEEF;
    #10;
    Imem2proc_response = 4'd0;
    #10;

    // Now assume cachemem valid
    cachemem_valid = 3'b111;
    cachemem_data[2] = 64'hDEAD_BEEF_DEAD_BEEF;

    #10;
    check("Cache should be valid", Icache_valid_out[2] == 1'b1);
    check("Cache data correct (lower 32b)", Icache_data_out[2] == 32'hDEAD_BEEF);

    $display("\n=== Test 3: Branch clears miss status ===");

    // trigger branch
    take_branch = 1;
    #10;
    take_branch = 0;
    #10;
    check("No memory command after branch", proc2Imem_command == 2'b00);

    $display("\n=== Test 4: Load another address triggers new memory request ===");

    // another new address
    proc2Icache_addr[2] = 64'h0000_0000_0000_0040;
    cachemem_valid = 3'b000; // Miss again

    #10;
    check("Another memory request issued", proc2Imem_command == 2'b01);

    $display("\n=== Finish Testing ===");
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
