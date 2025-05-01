`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/prefetch.sv"

module icache_tb;

  logic clock;
  logic reset;
  logic take_branch;
  logic [3:0] Imem2proc_response;
  logic [63:0] Imem2proc_data;
  logic [3:0] Imem2proc_tag;
  logic d_request;

  logic [1:0] shift;
  logic [2:0][`XLEN-1:0] proc2Icache_addr;
  logic [2:0][63:0] cachemem_data;
  logic [2:0] cachemem_valid;
  logic hit_but_stall;

  logic [1:0] proc2Imem_command;
  logic [`XLEN-1:0] proc2Imem_addr;
  logic [2:0][31:0] Icache_data_out;
  logic [2:0] Icache_valid_out;
  logic [2:0][4:0] current_index;
  logic [2:0][7:0] current_tag;
  logic [4:0] wr_index;
  logic [7:0] wr_tag;
  logic data_write_enable;

  int pass = 0;
  int fail = 0;

  icache dut(
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

  always #5 clock = ~clock;

  task reset_dut();
    begin
      clock = 0;
      reset = 1;
      take_branch = 0;
      d_request = 0;
      hit_but_stall = 0;
      Imem2proc_response = 0;
      Imem2proc_tag = 0;
      proc2Icache_addr = '{default:0};
      cachemem_valid = 3'b000;
      cachemem_data = '{default:0};
      shift = 0;
      #20;
      reset = 0;
    end
  endtask

  task simple_check(int test_num, logic cond);
    begin
      if (cond) begin
        $display("Test%0d PASS", test_num);
        pass++;
      end else begin
        $display("Test%0d FAIL", test_num);
        fail++;
      end
    end
  endtask

  initial begin
    $display("Start icache_tb");
    reset_dut();

    // Test 1: cache miss triggers load
    proc2Icache_addr[2] = 64'h1000;
    cachemem_valid = 3'b000;
    #10;
    simple_check(1, proc2Imem_command == BUS_LOAD);

    // Test 4: branch reload
    cachemem_valid = 3'b000;
    take_branch = 1;
    #10;
    simple_check(2, proc2Imem_command == BUS_LOAD);
    take_branch = 0;

    // Test 5: shift=1 addr
    proc2Icache_addr[1] = 64'h2000;
    shift = 2'd1;
    #10;
    simple_check(3, proc2Imem_addr[12:3] == proc2Icache_addr[1][12:3]);
    shift = 0;

    // Test 6: prefetch triggers load
    cachemem_valid = 3'b000;
    #20;
    simple_check(4, proc2Imem_command == BUS_LOAD);

    $display("Finish icache_tb");
    $display("Summary: %0d PASS / %0d FAIL", pass, fail);
    $finish;
  end

endmodule
