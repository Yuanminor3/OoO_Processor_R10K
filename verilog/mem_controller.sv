///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//  Modulename : mem_controller.sv                                           //
//                                                                           //
//                                                                           // 
///////////////////////////////////////////////////////////////////////////////

`ifndef __MEM_CONTROLLER_SV__
`define __MEM_CONTROLLER_SV__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module mem_controller (
    /* to mem */
    input [3:0] mc_mem_resp,  // <- mem
    input [63:0] mc_mem_rd_data,     // <- mem
    input [3:0] mc_mem_rd_id,       // <- mem

    output logic [1:0] mc_to_mem_cmd,  // -> mem
    output logic [`SYS_XLEN-1:0] mc_to_mem_addr, // ->mem
    output logic [63:0] mc_to_mem_wr_data,

    /* to Icache */
    input [1:0] mc_ic_cmd,      
    input [`SYS_XLEN-1:0] mc_ic_addr,    

    output  logic [3:0] mc_to_ic_resp,           
    output  logic [63:0] mc_to_ic_rd_data,              
    output  logic [3:0] mc_to_ic_rd_id,   // directly assign
    output  logic mc_ic_hold_flag,             // if high, mem is assigned to Dcache

    /* to Dcache */
    input [1:0] mc_dc_cmd,      
    input [`SYS_XLEN-1:0] mc_dc_addr,  
    input [63:0] mc_dc_wr_data,

    output  logic [3:0] mc_to_dc_resp,           
    output  logic [63:0] mc_to_dc_rd_data,              
    output  logic [3:0] mc_to_dc_rd_id

);

    logic mc_ic_req_active;
    assign mc_ic_req_active = mc_ic_cmd != BUS_NONE;
    assign mc_ic_hold_flag = mc_dc_cmd != BUS_NONE;

    //pass through 
    assign  mc_to_ic_resp = mc_mem_resp;
    assign  mc_to_ic_rd_data = mc_mem_rd_data;
    assign  mc_to_ic_rd_id = mc_mem_rd_id;

    assign  mc_to_dc_resp = mc_mem_resp;
    assign  mc_to_dc_rd_data = mc_mem_rd_data;
    assign  mc_to_dc_rd_id = mc_mem_rd_id;

    always_comb begin
        if (mc_ic_hold_flag) begin
            mc_to_mem_cmd = mc_dc_cmd;
            mc_to_mem_addr = mc_dc_addr;
            mc_to_mem_wr_data = mc_dc_wr_data;
        end
        else if (mc_ic_req_active) begin
            mc_to_mem_cmd = mc_ic_cmd;
            mc_to_mem_addr = mc_ic_addr;
            mc_to_mem_wr_data = 0;
        end
        else begin
            mc_to_mem_cmd = BUS_NONE;
            mc_to_mem_addr = 0;
            mc_to_mem_wr_data = 0;
        end
    end

endmodule

`endif

