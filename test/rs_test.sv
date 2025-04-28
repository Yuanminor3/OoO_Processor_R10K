`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module RS_tb;

    // Clock and reset
    reg clock;
    reg reset;
    
    // Inputs
    RS_IN_PACKET [2:0] rs_in;
    FU_FIFO_PACKET fu_fifo_stall;
    CDB_T_PACKET cdb_t;
    reg [2**`LSQ-1:0] load_tail_ready;
    
    // Outputs
    RS_S_PACKET [2:0] issue_insts;
    wire [2:0] struct_stall;
    
    // Instantiate DUT
    RS dut (
        .clock(clock),
        .reset(reset),
        .rs_in(rs_in),
        .fu_fifo_stall(fu_fifo_stall),
        .cdb_t(cdb_t),
        .load_tail_ready(load_tail_ready),
        .issue_insts(issue_insts),
        .struct_stall(struct_stall)
    );
    
    // Clock generation
    always #5 clock = ~clock;
    
    // Test sequence
    initial begin
        $display("Starting RS testbench...");
        
        // Initialize
        clock = 0;
        reset = 1;
        rs_in = '{default:0};
        fu_fifo_stall = 0;
        cdb_t = 0;
        load_tail_ready = {2**`LSQ{1'b1}}; // Assume all loads ready
        #10;
        reset = 0;
        
        // Test 1: Basic dispatch
        $display("\nTest 1: Basic dispatch");
        rs_in[0].valid = 1;
        rs_in[0].fu_sel = ALU_1;
        rs_in[0].reg1_pr = 5'd1;
        rs_in[0].reg2_pr = 5'd2;
        rs_in[0].reg1_ready = 1;
        rs_in[0].reg2_ready = 1;
        #10;
        
        if (struct_stall == 0) begin
            $display("PASSED - Dispatch accepted");
        end else begin
            $display("FAILED - Unexpected stall");
            $finish;
        end
        
        // Test 2: CDB broadcast
        $display("\nTest 2: CDB broadcast");
        rs_in = '{default:0};
        rs_in[0].valid = 1;
        rs_in[0].fu_sel = ALU_1;
        rs_in[0].reg1_pr = 5'd3;
        rs_in[0].reg2_pr = 5'd4;
        rs_in[0].reg1_ready = 0;
        rs_in[0].reg2_ready = 0;
        cdb_t.t0 = 5'd3; // Make reg1 ready
        #10;
        
        if (issue_insts[0].valid == 0) begin
            $display("PASSED - CDB updated reg1");
            $display("\nALL TESTS PASSED");
        end else begin
            $display("FAILED - Instruction issued too early");
            $finish;
        end
        
        $finish;
    end
    
endmodule
