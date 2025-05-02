/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  regfile.v                                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`ifndef __REGFILE_V__
`define __REGFILE_V__

`include "verilog/sys_defs.svh"

module regfile(
        input   [4:0] rda_idx, rdb_idx, write_idx,    // read/write index
        input  [`SYS_XLEN-1:0] write_data,                // write data
        input  wr_en, wr_clk,

        output logic [`SYS_XLEN-1:0] rda_out, rdb_out    // read data
      );
  
  logic    [31:0] [`SYS_XLEN-1:0] registers;   // 32, 64-bit Registers

  wire   [`SYS_XLEN-1:0] rda_reg = registers[rda_idx];
  wire   [`SYS_XLEN-1:0] rdb_reg = registers[rdb_idx];

  // Read port A
  always_comb
    if (rda_idx == `SYS_ZERO_ARCH_REG)
      rda_out = 0;
    else if (wr_en && (write_idx == rda_idx))
      rda_out = write_data;  // internal forwarding
    else
      rda_out = rda_reg;

  // Read port B
  always_comb
    if (rdb_idx == `SYS_ZERO_ARCH_REG)
      rdb_out = 0;
    else if (wr_en && (write_idx == rdb_idx))
      rdb_out = write_data;  // internal forwarding
    else
      rdb_out = rdb_reg;

  // Write port
  always_ff @(posedge wr_clk)
    if (wr_en) begin
      registers[write_idx] <= `SYS_SMALL_DELAY write_data;
    end

endmodule // regfile
`endif //__REGFILE_V__
