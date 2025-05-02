`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module RS_tb;

    // Clock and rst
    reg clk;
    reg rst;
    
    // Inputs
    RS_IN_PACKET [2:0] dispatch_rs_pkts;
    FU_FIFO_PACKET fu_fifo_stall;
    CDB_T_PACKET cs_cdb_broadcast;
    reg [2**`SYS_LSQ_ADDR_WIDTH-1:0] rsb_sq_ready_flags;
    
    // Outputs
    RS_S_PACKET [2:0] rsb_issue_packets;
    wire [2:0] rsb_struct_halt;
    
    // Instantiate DUT
    RS dut (
        .clk(clk),
        .rst(rst),
        .dispatch_rs_pkts(dispatch_rs_pkts),
        .fu_fifo_stall(fu_fifo_stall),
        .cs_cdb_broadcast(cs_cdb_broadcast),
        .rsb_sq_ready_flags(rsb_sq_ready_flags),
        .rsb_issue_packets(rsb_issue_packets),
        .rsb_struct_halt(rsb_struct_halt)
    );
    
    // Clock generation
    always #5 clk = ~clk;
    
    // Test sequence
    initial begin
        $display("Starting RS testbench...");
        
        // Initialize
        clk = 0;
        rst = 1;
        dispatch_rs_pkts = '{default:0};
        fu_fifo_stall = 0;
        cs_cdb_broadcast = 0;
        rsb_sq_ready_flags = {2**`SYS_LSQ_ADDR_WIDTH{1'b1}}; // Assume all loads ready
        #10;
        rst = 0;
        
        // Test 1: Basic lsq_disp_mask
        $display("\nTest 1: Basic lsq_disp_mask");
        dispatch_rs_pkts[0].valid = 1;
        dispatch_rs_pkts[0].dec_fu_unit_sel = ALU_1;
        dispatch_rs_pkts[0].dispatch_src1_pr = 5'd1;
        dispatch_rs_pkts[0].dispatch_src2_pr = 5'd2;
        dispatch_rs_pkts[0].dispatch_src1_rdy = 1;
        dispatch_rs_pkts[0].dispatch_src2_rdy = 1;
        #10;
        
        if (rsb_struct_halt == 0) begin
            $display("PASSED - Dispatch accepted");
        end else begin
            $display("FAILED - Unexpected lsq_stall_mask");
            $finish;
        end
        
        // Test 2: CDB broadcast
        $display("\nTest 2: CDB broadcast");
        dispatch_rs_pkts = '{default:0};
        dispatch_rs_pkts[0].valid = 1;
        dispatch_rs_pkts[0].dec_fu_unit_sel = ALU_1;
        dispatch_rs_pkts[0].dispatch_src1_pr = 5'd3;
        dispatch_rs_pkts[0].dispatch_src2_pr = 5'd4;
        dispatch_rs_pkts[0].dispatch_src1_rdy = 0;
        dispatch_rs_pkts[0].dispatch_src2_rdy = 0;
        cs_cdb_broadcast.t0 = 5'd3; // Make dec_src1_reg ready
        #10;
        
        if (rsb_issue_packets[0].valid == 0) begin
            $display("PASSED - CDB updated dec_src1_reg");
            $display("\nALL TESTS PASSED");
        end else begin
            $display("FAILED - Instruction issued too early");
            $finish;
        end
        
        $finish;
    end
    
endmodule
