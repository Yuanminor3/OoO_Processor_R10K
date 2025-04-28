`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

`define RV32_signext_Iimm(inst) {{20{inst[31]}}, inst[31:20]}
`define RV32_signext_Bimm(inst) {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
`define RV32_signext_Jimm(inst) {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}

module branch_fu_tb;

    // DUT signals
    logic clock;
    logic reset;
    logic complete_stall;
    ISSUE_FU_PACKET fu_packet_in;
    logic fu_ready;
    logic want_to_complete_branch;
    FU_COMPLETE_PACKET fu_packet_out;
    logic update_EN;
    logic [`XLEN-1:0] update_pc;
    logic update_direction;
    logic [`XLEN-1:0] update_target;

    branch_stage dut (
        .clock(clock),
        .reset(reset),
        .complete_stall(complete_stall),
        .fu_packet_in(fu_packet_in),
        .fu_ready(fu_ready),
        .want_to_complete_branch(want_to_complete_branch),
        .fu_packet_out(fu_packet_out),
        .update_EN(update_EN),
        .update_pc(update_pc),
        .update_direction(update_direction),
        .update_target(update_target)
    );

    // Clock generation
    initial begin
        clock = 0;
        forever #5 clock = ~clock;
    end

    // Test counters
    integer pass_count = 0;
    integer fail_count = 0;
    integer total_count = 0;

    // Task for easy testing
    task test_branch(
        input [`XLEN-1:0] rs1_val,
        input [`XLEN-1:0] rs2_val,
        input BR_SELECT br_type,
        input logic expected_taken,
        input string test_name
    );
        begin
            fu_packet_in.valid = 1'b1;
            fu_packet_in.PC = 32'h1000_0000;
            fu_packet_in.NPC = 32'h1000_0004;
            fu_packet_in.r1_value = rs1_val;
            fu_packet_in.r2_value = rs2_val;
            fu_packet_in.op_sel.br = br_type;
            fu_packet_in.opa_select = OPA_IS_RS1;
            fu_packet_in.opb_select = OPB_IS_I_IMM;
            fu_packet_in.inst = 32'b0;

            @(posedge clock);
            #1;

            total_count++;

            if (fu_packet_out.if_take_branch === expected_taken) begin
                $display("[PASS] %s | Expected: %0d, Got: %0d", test_name, expected_taken, fu_packet_out.if_take_branch);
                pass_count++;
            end else begin
                $display("[FAIL] %s | Expected: %0d, Got: %0d", test_name, expected_taken, fu_packet_out.if_take_branch);
                fail_count++;
            end
        end
    endtask

    initial begin
        reset = 1'b1;
        complete_stall = 1'b0;
        fu_packet_in = '0;

        @(posedge clock);
        reset = 1'b0;
        @(posedge clock);

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

        @(posedge clock);
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

