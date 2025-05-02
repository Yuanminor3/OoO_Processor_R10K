`timescale 1ns/100ps
`include "verilog/sys_defs.svh"


module prefetch_simple_tb;
    // 时钟与复位信号
    logic clk;
    logic rst;

    // DUT 输入
    logic [3:0] Imem2pref_response;
    logic [3:0] Imem2pref_tag;
    logic       pf_bus_priority;
    logic       branch;
    logic [2:0][`SYS_XLEN-1:0] icache_req_addr;
    logic [2:0] cachemem_valid;
    logic       want_to_fetch;

    // DUT 输出
    logic icache_pref_done;
    logic [1:0] icache_pref_cmd;
    logic [`SYS_XLEN-1:0] icache_pref_addr;
    logic [4:0] icache_pref_idx;
    logic [7:0] icache_pref_id;
    logic icache_pref_wEN;

    // 实例化 DUT
    prefetch uut (
        .clk(clk),
        .rst(rst),
        .Imem2pref_response(Imem2pref_response),
        .Imem2pref_tag(Imem2pref_tag),
        .pf_bus_priority(pf_bus_priority),
        .branch(branch),
        .icache_req_addr(icache_req_addr),
        .cachemem_valid(cachemem_valid),
        .want_to_fetch(want_to_fetch),
        .icache_pref_done(icache_pref_done),
        .icache_pref_cmd(icache_pref_cmd),
        .icache_pref_addr(icache_pref_addr),
        .icache_pref_idx(icache_pref_idx),
        .icache_pref_id(icache_pref_id),
        .icache_pref_wEN(icache_pref_wEN)
    );

    // 时钟生成
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // 简单测试：检查 icache_pref_cmd
    initial begin
        // 1) 复位
        rst = 1;
        Imem2pref_response = 0;
        Imem2pref_tag      = 0;
        pf_bus_priority           = 0;
        branch             = 0;
        icache_req_addr   = '{32'h0,32'h4,32'h8};
        cachemem_valid     = 3'b000;
        want_to_fetch      = 0;
        #20;

        // 2) 取消复位并触发预取
        rst = 0;
        want_to_fetch = 1;
        #10;
        want_to_fetch = 0;
        #10;

        // 3) 验证只要 icache_pref_cmd 为 BUS_LOAD 就 PASS
        if (icache_pref_cmd == 2'h1) begin
            $display("ALL PASS!");
        end else begin
            $display("FAIL: icache_pref_cmd = %0h", icache_pref_cmd);
        end
        $finish;
    end
endmodule
