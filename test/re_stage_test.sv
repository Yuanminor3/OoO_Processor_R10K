`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module retire_stage_tb;

    // Inputs
    logic [`ROB-1:0] fl_distance;
    ROB_ENTRY_PACKET [2:0] rob_head_entry;
    logic [31:0][`PR-1:0] archi_maptable;
    
    // Outputs
    logic BPRecoverEN;
    logic [`XLEN-1:0] target_pc;
    logic [2:0] Retire_EN;
    logic halt;
    
    // Instantiate DUT
    retire_stage dut (
        .fl_distance(fl_distance),
        .BPRecoverEN(BPRecoverEN),
        .target_pc(target_pc),
        .rob_head_entry(rob_head_entry),
        .archi_maptable(archi_maptable),
        .recover_maptable(),
        .Retire_EN(Retire_EN),
        .SQRetireEN(),
        .halt(halt)
    );
    
    initial begin
        $display("Starting retire_stage test...");
        
        // Initialize
        fl_distance = 0;
        rob_head_entry = '{default:0};
        archi_maptable = '{default:0};
        
        // Test: Branch recovery
        rob_head_entry = '{default:0};
        rob_head_entry[2].completed = 1'b1;
        rob_head_entry[2].precise_state_need = 1'b1;
        rob_head_entry[2].target_pc = 32'h80000000;
        #10;
        
        if (BPRecoverEN && target_pc == 32'h80000000) begin
            $display("\nPASSED - Test: Branch recovery");
        end else begin
            $display("\nFAILED - Test: BPRecoverEN=%b, target_pc=%h, Retire_EN=%b",
                     BPRecoverEN, target_pc, Retire_EN);
            $finish;
        end
        
        $finish;
    end
    
endmodule
