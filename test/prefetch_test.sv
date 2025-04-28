`timescale 1ns/100ps
`include "verilog/sys_defs.svh"


module prefetch_simple_tb;
    // 时钟与复位信号
    logic clock;
    logic reset;

    // DUT 输入
    logic [3:0] Imem2pref_response;
    logic [3:0] Imem2pref_tag;
    logic       give_way;
    logic       branch;
    logic [2:0][`XLEN-1:0] proc2Icache_addr;
    logic [2:0] cachemem_valid;
    logic       want_to_fetch;

    // DUT 输出
    logic already_fetched;
    logic [1:0] prefetch_command;
    logic [`XLEN-1:0] prefetch_addr;
    logic [4:0] prefetch_index;
    logic [7:0] prefetch_tag;
    logic prefetch_wr_enable;

    // 实例化 DUT
    prefetch uut (
        .clock(clock),
        .reset(reset),
        .Imem2pref_response(Imem2pref_response),
        .Imem2pref_tag(Imem2pref_tag),
        .give_way(give_way),
        .branch(branch),
        .proc2Icache_addr(proc2Icache_addr),
        .cachemem_valid(cachemem_valid),
        .want_to_fetch(want_to_fetch),
        .already_fetched(already_fetched),
        .prefetch_command(prefetch_command),
        .prefetch_addr(prefetch_addr),
        .prefetch_index(prefetch_index),
        .prefetch_tag(prefetch_tag),
        .prefetch_wr_enable(prefetch_wr_enable)
    );

    // 时钟生成
    initial begin
        clock = 0;
        forever #5 clock = ~clock;
    end

    // 简单测试：检查 prefetch_command
    initial begin
        // 1) 复位
        reset = 1;
        Imem2pref_response = 0;
        Imem2pref_tag      = 0;
        give_way           = 0;
        branch             = 0;
        proc2Icache_addr   = '{32'h0,32'h4,32'h8};
        cachemem_valid     = 3'b000;
        want_to_fetch      = 0;
        #20;

        // 2) 取消复位并触发预取
        reset = 0;
        want_to_fetch = 1;
        #10;
        want_to_fetch = 0;
        #10;

        // 3) 验证只要 prefetch_command 为 BUS_LOAD 就 PASS
        if (prefetch_command == 2'h1) begin
            $display("PASS");
        end else begin
            $display("FAIL: prefetch_command = %0h", prefetch_command);
        end
        $finish;
    end
endmodule
