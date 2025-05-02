// cachemem4x64, 3 reading ports, 1 writing port

`timescale 1ns/100ps

module cache(
        input clk, rst, dc_sq_wrEN,             // <- icache.icache_wrEN
        input  [4:0] dc_sq_wr_index,                   // <- icache.icache_wr_index
        input  [2:0][4:0] dc_Id_index,              // <- icache.icache_index
        input  [7:0] dc_sq_wr_id,                   // <- icache.icache_wr_id
        input  [2:0][7:0] dc_Id_id,              // <- icache.icache_id
        input  [63:0] dc_sq_wr_data,                 // <- mem.mem_resp_data

        output [2:0][63:0] dc_Id_data,            // -> icache.cachemem_data
        output [2:0] dc_Id_valid                  // -> icache.cachemem_valid
);

  logic [31:0] [63:0] icmem_data;
  logic [31:0] [7:0]  icmem_tags;
  logic [31:0]        icmem_valids;

  assign dc_Id_data[2] = icmem_data[dc_Id_index[2]];
  assign dc_Id_data[1] = icmem_data[dc_Id_index[1]];
  assign dc_Id_data[0] = icmem_data[dc_Id_index[0]];
  assign dc_Id_valid[2] = icmem_valids[dc_Id_index[2]] && (icmem_tags[dc_Id_index[2]] == dc_Id_id[2]);
  assign dc_Id_valid[1] = icmem_valids[dc_Id_index[1]] && (icmem_tags[dc_Id_index[1]] == dc_Id_id[1]);
  assign dc_Id_valid[0] = icmem_valids[dc_Id_index[0]] && (icmem_tags[dc_Id_index[0]] == dc_Id_id[0]);

  always_ff @(posedge clk) begin
    if(rst)
      icmem_valids <= `SYS_SMALL_DELAY 32'b0;
    else if(dc_sq_wrEN) 
      icmem_valids[dc_sq_wr_index] <= `SYS_SMALL_DELAY 1;
  end
  
  always_ff @(posedge clk) begin
    if(dc_sq_wrEN) begin
      icmem_data[dc_sq_wr_index] <= `SYS_SMALL_DELAY dc_sq_wr_data;
      icmem_tags[dc_sq_wr_index] <= `SYS_SMALL_DELAY dc_sq_wr_id;
    end
  end

endmodule
