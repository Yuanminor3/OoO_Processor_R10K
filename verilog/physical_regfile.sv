`ifndef __PR_V__
`define __PR_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module physical_regfile(
    input   clk,
    input   rst,
    input  [2:0][`SYS_PHYS_REG-1:0] 	rd_index_1, rd_index_2,   // read index
    input  [2:0][`SYS_XLEN-1:0] 	write_data,   // write data
    input   CDB_T_PACKET 	write_idx,

    output logic [2:0][`SYS_XLEN-1:0] rd_result_1, rd_result_2    // read data

);

logic [2**`SYS_PHYS_REG-1:0][`SYS_XLEN-1:0] phy_regs;
logic [2**`SYS_PHYS_REG-1:0][`SYS_XLEN-1:0] phy_regs_next;

// Write 
always_comb begin
    phy_regs_next = phy_regs;
    if (write_idx.t0 != `SYS_ZERO_PHYS_REG)
        phy_regs_next[write_idx.t0] = write_data[0];
    if (write_idx.t1 != `SYS_ZERO_PHYS_REG)
        phy_regs_next[write_idx.t1] = write_data[1];
    if (write_idx.t2 != `SYS_ZERO_PHYS_REG)
        phy_regs_next[write_idx.t2] = write_data[2];
end

// Read 
always_comb begin
    for(int i=0; i<3; i++) begin
        rd_result_1[i] = phy_regs[rd_index_1[i]];
        rd_result_2[i] = phy_regs[rd_index_2[i]];
    end
end
//sync rst
always_ff @(posedge clk) begin
    if (rst)
        phy_regs <= `SYS_SMALL_DELAY 0;
    else phy_regs <= `SYS_SMALL_DELAY phy_regs_next;
end

endmodule; // physical_regfile

`endif //__PR_V__

