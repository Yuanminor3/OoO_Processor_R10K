`ifndef __PR_V__
`define __PR_V__
//`define TEST_MODE

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module physical_regfile(
    input   clock,
    input   reset,
    input  [2:0][`PR-1:0] 	read_idx_1, read_idx_2,   // read index
    input  [2:0][`XLEN-1:0] 	write_data,   // write data
    input   CDB_T_PACKET 	write_idx,

    output logic [2:0][`XLEN-1:0] read_out_1, read_out_2    // read data

`ifdef TEST_MODE
    , output logic [2**`PR-1:0][`XLEN-1:0] registers_display           
`endif 
);

logic [2**`PR-1:0][`XLEN-1:0] registers;
logic [2**`PR-1:0][`XLEN-1:0] registers_next;

`ifdef TEST_MODE
assign registers_display = registers;
`endif 

// Write 
always_comb begin
    registers_next = registers;
    if (write_idx.t0 != `ZERO_PR)
        registers_next[write_idx.t0] = write_data[0];
    if (write_idx.t1 != `ZERO_PR)
        registers_next[write_idx.t1] = write_data[1];
    if (write_idx.t2 != `ZERO_PR)
        registers_next[write_idx.t2] = write_data[2];
end

// Read 
always_comb begin
    for(int i=0; i<3; i++) begin
        read_out_1[i] = registers[read_idx_1[i]];
        read_out_2[i] = registers[read_idx_2[i]];
    end
end
//sync reset
always_ff @(posedge clock) begin
    if (reset)
        registers <= `SD 0;
    else registers <= `SD registers_next;
end

endmodule; // physical_regfile

`endif //__PR_V__

