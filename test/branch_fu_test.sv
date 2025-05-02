`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

`define RV32_signext_Iimm(inst) {{20{inst[31]}}, inst[31:20]}
`define RV32_signext_Bimm(inst) {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
`define RV32_signext_Jimm(inst) {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}

module branch_fu_tb;

    // DUT signals
    logic clk;
    logic rst;
    logic bs_hazard;
    ISSUE_FU_PACKET bs_in_pkt;
    logic rsb_fu_ready;
    logic bs_fire;
    FU_COMPLETE_PACKET bs_out_pkt;
    logic bs_upd_en;
    logic [`SYS_XLEN-1:0] bs_upd_pc;
    logic bs_upd_taken;
    logic [`SYS_XLEN-1:0] bs_upd_target;

    branch_stage dut (
        .clk(clk),
        .rst(rst),
        .bs_hazard(bs_hazard),
        .bs_in_pkt(bs_in_pkt),
        .rsb_fu_ready(rsb_fu_ready),
        .bs_fire(bs_fire),
        .bs_out_pkt(bs_out_pkt),
        .bs_upd_en(bs_upd_en),
        .bs_upd_pc(bs_upd_pc),
        .bs_upd_taken(bs_upd_taken),
        .bs_upd_target(bs_upd_target)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test counters
    integer pass_count = 0;
    integer fail_count = 0;
    integer total_count = 0;

    // Task for easy testing
    task test_branch(
        input [`SYS_XLEN-1:0] rs1_val,
        input [`SYS_XLEN-1:0] rs2_val,
        input BR_SELECT br_type,
        input logic expected_taken,
        input string test_name
    );
        begin
            bs_in_pkt.valid = 1'b1;
            bs_in_pkt.PC = 32'h1000_0000;
            bs_in_pkt.NPC = 32'h1000_0004;
            bs_in_pkt.r1_value = rs1_val;
            bs_in_pkt.r2_value = rs2_val;
            bs_in_pkt.dec_fu_opcode.br = br_type;
            bs_in_pkt.dec_operandA_mux = OPA_IS_RS1;
            bs_in_pkt.dec_operandB_mux = OPB_IS_I_IMM;
            bs_in_pkt.inst = 32'b0;

            @(posedge clk);
            #1;

            total_count++;

            if (bs_out_pkt.if_take_branch === expected_taken) begin
                $display("[PASS] %s | Expected: %0d, Got: %0d", test_name, expected_taken, bs_out_pkt.if_take_branch);
                pass_count++;
            end else begin
                $display("[FAIL] %s | Expected: %0d, Got: %0d", test_name, expected_taken, bs_out_pkt.if_take_branch);
                fail_count++;
            end
        end
    endtask

    initial begin
        rst = 1'b1;
        bs_hazard = 1'b0;
        bs_in_pkt = '0;

        @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // ----------- Begin Tests ------------
        test_branch(32'd10, 32'd10, BEQ, 1'b1, "BEQ taken");
        test_branch(32'd10, 32'd20, BEQ, 1'b0, "BEQ not taken");

        test_branch(32'd10, 32'd20, BNE, 1'b1, "BNE taken");
        test_branch(32'd10, 32'd10, BNE, 1'b0, "BNE not taken");

        test_branch(32'd5, 32'd10, BLT, 1'b1, "BLT taken");
        test_branch(32'd10, 32'd5, BLT, 1'b0, "BLT not taken");

        test_branch(32'd10, 32'd5, BGE, 1'b1, "BGE taken");
        test_branch(32'd5, 32'd10, BGE, 1'b0, "BGE not taken");

        test_branch(32'd5, 32'd10, BLTU, 1'b1, "BLTU taken (unsigned)");
        test_branch(32'd10, 32'd5, BLTU, 1'b0, "BLTU not taken (unsigned)");

        test_branch(32'd10, 32'd5, BGEU, 1'b1, "BGEU taken (unsigned)");
        test_branch(32'd5, 32'd10, BGEU, 1'b0, "BGEU not taken (unsigned)");

        test_branch(32'd123, 32'd456, UNCOND, 1'b1, "UNCOND always taken");
        // ----------- End of Tests ------------

        @(posedge clk);
        $display("====================================");
        $display("Branch Functional Unit Test Summary:");
        if (fail_count == 0) begin
            $display("All PASS (%0d tests)", total_count);
        end else begin
            $display("PASS: %0d", pass_count);
            $display("FAIL: %0d", fail_count);
        end
        $display("====================================");

        $finish;
    end

endmodule

