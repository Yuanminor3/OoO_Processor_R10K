`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module tb_map_table;

    // Clock & Reset
    logic clock;
    logic reset;

    // Inputs
    logic [31:0][`PR-1:0] archi_maptable;
    logic BPRecoverEN;
    CDB_T_PACKET cdb_t_in;
    logic [2:0][`PR-1:0] maptable_new_pr;
    logic [2:0][4:0] maptable_new_ar;
    logic [2:0][4:0] reg1_ar;
    logic [2:0][4:0] reg2_ar;

    // Outputs
    logic [2:0][`PR-1:0] reg1_tag;
    logic [2:0][`PR-1:0] reg2_tag;
    logic [2:0] reg1_ready;
    logic [2:0] reg2_ready;
    logic [2:0][`PR-1:0] Told_out;

    // Instantiate the DUT
    map_table dut (
        .clock(clock),
        .reset(reset),
        .archi_maptable(archi_maptable),
        .BPRecoverEN(BPRecoverEN),
        .cdb_t_in(cdb_t_in),
        .maptable_new_pr(maptable_new_pr),
        .maptable_new_ar(maptable_new_ar),
        .reg1_ar(reg1_ar),
        .reg2_ar(reg2_ar),
        .reg1_tag(reg1_tag),
        .reg2_tag(reg2_tag),
        .reg1_ready(reg1_ready),
        .reg2_ready(reg2_ready),
        .Told_out(Told_out)
    );

    // Clock generation
    always #5 clock = ~clock;

    initial begin
        clock = 0;
        reset = 1;
        BPRecoverEN = 0;
        archi_maptable = '{default:0};
        cdb_t_in = '{t0: 0, t1: 0, t2: 0};
        maptable_new_pr = '{default:0};
        maptable_new_ar = '{default:0};
        reg1_ar = '{default:0};
        reg2_ar = '{default:0};

        // Apply reset
        #10; reset = 0;

        // === Test 1: Mapping and readiness after new allocation ===
        maptable_new_ar = '{5'd1, 5'd2, 5'd3};
        maptable_new_pr = '{6'd10, 6'd11, 6'd12};
        reg1_ar = '{5'd1, 5'd2, 5'd3};
        reg2_ar = '{5'd0, 5'd1, 5'd2};
        #10;

        $display("\n=== Test 1: PR Mapping ===");
        $display("reg1_tag = %p", reg1_tag);
        $display("reg2_tag = %p", reg2_tag);
        $display("Told_out = %p", Told_out);
        $display("reg1_ready = %b", reg1_ready);
        $display("reg2_ready = %b", reg2_ready);

        // === Test 2: CDB broadcast sets ready ===
        cdb_t_in.t0 = 6'd11;
        cdb_t_in.t1 = 6'd12;
        cdb_t_in.t2 = 6'd0;
        #10;

        $display("\n=== Test 2: CDB Ready Check ===");
        $display("reg1_ready = %b", reg1_ready);
        $display("reg2_ready = %b", reg2_ready);

        // === Test 3: Recovery with architectural table ===
        BPRecoverEN = 1;
        for (int i = 0; i < 32; i++) begin
            archi_maptable[i] = i[`PR-1:0];
        end
        #10; BPRecoverEN = 0;

        $display("\n=== Test 3: Recovery ===");
        $display("reg1_tag = %p", reg1_tag);
        $display("reg1_ready = %b", reg1_ready);

        $display("\n✅ [ALL TESTS COMPLETED - Check values above]");
        $finish;
    end

endmodule
