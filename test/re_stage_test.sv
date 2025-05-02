`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module retire_stage_tb;

    // Inputs
    logic [`SYS_ROB_ADDR_WIDTH-1:0] fl_free_count;
    ROB_ENTRY_PACKET [2:0] rob_head_entry;
    logic [31:0][`SYS_PHYS_REG_ADDR_WIDTH-1:0] mt_checkpoint_tbl;
    
    // Outputs
    logic fch_rec_enable;
    logic [`SYS_XLEN-1:0] cs_retire_pc;
    logic [2:0] Retire_EN;
    logic halt;
    
    // Instantiate DUT
    retire_stage dut (
        .fl_free_count(fl_free_count),
        .fch_rec_enable(fch_rec_enable),
        .cs_retire_pc(cs_retire_pc),
        .rob_head_entry(rob_head_entry),
        .mt_checkpoint_tbl(mt_checkpoint_tbl),
        .recover_maptable(),
        .fl_retire_en_mask(Retire_EN),
        .SQRetireEN(),
        .halt(halt),
	.retired_inst_cnt()
    );
    
    initial begin
        $display("Starting retire_stage test...");
        
        // Initialize
        fl_free_count = 0;
        rob_head_entry = '{default:0};
        mt_checkpoint_tbl = '{default:0};
        
        // Test: Branch recovery
        rob_head_entry = '{default:0};
        rob_head_entry[2].completed = 1'b1;
        rob_head_entry[2].precise_state_need = 1'b1;
        rob_head_entry[2].cs_retire_pc = 32'h80000000;
        #10;
        
        if (fch_rec_enable && cs_retire_pc == 32'h80000000) begin
            $display("\nPASSED - Test: Branch recovery");
        end else begin
            $display("\nFAILED - Test: fch_rec_enable=%b, cs_retire_pc=%h, Retire_EN=%b",
                     fch_rec_enable, cs_retire_pc, Retire_EN);
            $finish;
        end
        
        $finish;
    end
    
endmodule
