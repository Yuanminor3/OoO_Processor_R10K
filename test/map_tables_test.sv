`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module tb_map_table;

    // Clock & Reset
    logic clk;
    logic rst;

    // Inputs
    logic [31:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] mt_checkpoint_tbl;
    logic fch_rec_enable;
    CDB_T_PACKET cdb_t_in;
    logic [2:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] dispatch_pr_alloc_tags;
    logic [2:0][4:0] mt_new_arch_regs;
    logic [2:0][4:0] dispatch_src1_arch_regs;
    logic [2:0][4:0] dispatch_src2_arch_regs;

    // Outputs
    logic [2:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] reg1_tag;
    logic [2:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] reg2_tag;
    logic [2:0] dispatch_src1_rdy;
    logic [2:0] dispatch_src2_rdy;
    logic [2:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] mt_old_pr_tags;

    // Instantiate the DUT
    map_table dut (
        .clk(clk),
        .rst(rst),
        .mt_checkpoint_tbl(mt_checkpoint_tbl),
        .fch_rec_enable(fch_rec_enable),
        .cdb_t_in(cdb_t_in),
        .dispatch_pr_alloc_tags(dispatch_pr_alloc_tags),
        .mt_new_arch_regs(mt_new_arch_regs),
        .dispatch_src1_arch_regs(dispatch_src1_arch_regs),
        .dispatch_src2_arch_regs(dispatch_src2_arch_regs),
        .reg1_tag(reg1_tag),
        .reg2_tag(reg2_tag),
        .dispatch_src1_rdy(dispatch_src1_rdy),
        .dispatch_src2_rdy(dispatch_src2_rdy),
        .mt_old_pr_tags(mt_old_pr_tags)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rst = 1;
        fch_rec_enable = 0;
        mt_checkpoint_tbl = '{default:0};
        cdb_t_in = '{t0: 0, t1: 0, t2: 0};
        dispatch_pr_alloc_tags = '{default:0};
        mt_new_arch_regs = '{default:0};
        dispatch_src1_arch_regs = '{default:0};
        dispatch_src2_arch_regs = '{default:0};

        // Apply rst
        #10; rst = 0;

        // === Test 1: Mapping and readiness after new allocation ===
        mt_new_arch_regs = '{5'd1, 5'd2, 5'd3};
        dispatch_pr_alloc_tags = '{6'd10, 6'd11, 6'd12};
        dispatch_src1_arch_regs = '{5'd1, 5'd2, 5'd3};
        dispatch_src2_arch_regs = '{5'd0, 5'd1, 5'd2};
        #10;

        $display("\n=== Test 1: SYS_PHYS_REG_ADDR_WIDTH Mapping ===");
        $display("reg1_tag = %p", reg1_tag);
        $display("reg2_tag = %p", reg2_tag);
        $display("mt_old_pr_tags = %p", mt_old_pr_tags);
        $display("dispatch_src1_rdy = %b", dispatch_src1_rdy);
        $display("dispatch_src2_rdy = %b", dispatch_src2_rdy);

        // === Test 2: CDB broadcast sets ready ===
        cdb_t_in.t0 = 6'd11;
        cdb_t_in.t1 = 6'd12;
        cdb_t_in.t2 = 6'd0;
        #10;

        $display("\n=== Test 2: CDB Ready Check ===");
        $display("dispatch_src1_rdy = %b", dispatch_src1_rdy);
        $display("dispatch_src2_rdy = %b", dispatch_src2_rdy);

        // === Test 3: Recovery with architectural table ===
        fch_rec_enable = 1;
        for (int i = 0; i < 32; i++) begin
            mt_checkpoint_tbl[i] = i[`SYS_PHYS_REG_ADDR_WIDTH-1:0];
        end
        #10; fch_rec_enable = 0;

        $display("\n=== Test 3: Recovery ===");
        $display("reg1_tag = %p", reg1_tag);
        $display("dispatch_src1_rdy = %b", dispatch_src1_rdy);

        $display("\n✅ [ALL TESTS COMPLETED - Check values above]");
        $finish;
    end

endmodule
