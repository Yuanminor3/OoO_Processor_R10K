`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/prefetch.sv"

module icache_tb;

  logic clk;
  logic rst;
  logic icache_branch;
  logic [3:0] icache_mem_resp_code;
  logic [63:0] icache_mem_resp_data;
  logic [3:0] icache_mem_resp_id;
  logic mc_ic_hold_flag;

  logic [1:0] icache_shift;
  logic [2:0][`SYS_XLEN-1:0] icache_req_addr;
  logic [2:0][63:0] cachemem_data;
  logic [2:0] cachemem_valid;
  logic icache_pipeline_hold;

  logic [1:0] icache_mem_req_cmd;
  logic [`SYS_XLEN-1:0] icache_mem_req_addr;
  logic [2:0][31:0] fetch_inst_words;
  logic [2:0] fetch_inst_valids;
  logic [2:0][4:0] icache_index;
  logic [2:0][7:0] icache_id;
  logic [4:0] icache_wr_index;
  logic [7:0] icache_wr_id;
  logic icache_wrEN;

  int pass = 0;
  int fail = 0;

  icache dut(
    .clk(clk),
    .rst(rst),
    .icache_branch(icache_branch),
    .icache_mem_resp_code(icache_mem_resp_code),
    .icache_mem_resp_data(icache_mem_resp_data),
    .icache_mem_resp_id(icache_mem_resp_id),
    .mc_ic_hold_flag(mc_ic_hold_flag),
    .icache_shift(icache_shift),
    .icache_req_addr(icache_req_addr),
    .cachemem_data(cachemem_data),
    .cachemem_valid(cachemem_valid),
    .icache_pipeline_hold(icache_pipeline_hold),
    .icache_mem_req_cmd(icache_mem_req_cmd),
    .icache_mem_req_addr(icache_mem_req_addr),
    .fetch_inst_words(fetch_inst_words),
    .fetch_inst_valids(fetch_inst_valids),
    .icache_index(icache_index),
    .icache_id(icache_id),
    .icache_wr_index(icache_wr_index),
    .icache_wr_id(icache_wr_id),
    .icache_wrEN(icache_wrEN)
  );

  always #5 clk = ~clk;

  task reset_dut();
    begin
      clk = 0;
      rst = 1;
      icache_branch = 0;
      mc_ic_hold_flag = 0;
      icache_pipeline_hold = 0;
      icache_mem_resp_code = 0;
      icache_mem_resp_id = 0;
      icache_req_addr = '{default:0};
      cachemem_valid = 3'b000;
      cachemem_data = '{default:0};
      icache_shift = 0;
      #20;
      rst = 0;
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
    icache_req_addr[2] = 64'h1000;
    cachemem_valid = 3'b000;
    #10;
    simple_check(1, icache_mem_req_cmd == BUS_LOAD);

    // Test 4: branch reload
    cachemem_valid = 3'b000;
    icache_branch = 1;
    #10;
    simple_check(2, icache_mem_req_cmd == BUS_LOAD);
    icache_branch = 0;

    // Test 5: icache_shift=1 addr
    icache_req_addr[1] = 64'h2000;
    icache_shift = 2'd1;
    #10;
    simple_check(3, icache_mem_req_addr[12:3] == icache_req_addr[1][12:3]);
    icache_shift = 0;

    // Test 6: prefetch triggers load
    cachemem_valid = 3'b000;
    #20;
    simple_check(4, icache_mem_req_cmd == BUS_LOAD);

    $display("Finish icache_tb");
    $display("Summary: %0d PASS / %0d FAIL", pass, fail);
    $finish;
  end

endmodule
