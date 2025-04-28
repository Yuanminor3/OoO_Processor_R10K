`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module complete_stage_tb;

    // Clock and reset signals
    logic clock;
    logic reset;
    
    // Inputs to DUT
    FU_STATE_PACKET             fu_finish;
    FU_COMPLETE_PACKET [7:0]    fu_c_in;
    
    // Outputs from DUT
    FU_STATE_PACKET             fu_c_stall;
    CDB_T_PACKET                cdb_t;
    logic [2:0][`XLEN-1:0]      wb_value;
    
    // Instantiate the DUT
    complete_stage dut (
        .clock(clock),
        .reset(reset),
        .fu_finish(fu_finish),
        .fu_c_in(fu_c_in),
        .fu_c_stall(fu_c_stall),
        .cdb_t(cdb_t),
        .wb_value(wb_value),
        .complete_valid(),
        .complete_entry(),
        .precise_state_valid(),
        .target_pc()
    );
    
    // Clock generation
    always begin
        #5 clock = ~clock;
    end
    
    // Main test sequence
    initial begin
        // Initialize signals
        clock = 0;
        reset = 1;
        fu_finish = 0;
        fu_c_in = '{default:0};
        
        // Apply reset
        #10 reset = 0;
        
        // Test: Single ALU1 completion
        $display("Starting Test 1: Single ALU1 completion");
        fu_finish.alu_1 = 1;
        fu_c_in[0].valid = 1;
        fu_c_in[0].dest_pr = 5'h01;
        fu_c_in[0].dest_value = 32'h12345678;
        #10;
        
        // Verify output
        if (cdb_t == 5'h01) begin
            $display("cdb_t = %h, wb_value = %h", cdb_t, wb_value);
            $display("\nPASSED\n");
        end else begin
            $display("cdb_t = %h, wb_value = %h", cdb_t, wb_value);
            $display("\nFAILED\n");
        end
        
        $finish;
    end
    
endmodule
