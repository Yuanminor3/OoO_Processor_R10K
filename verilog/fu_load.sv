
`ifndef __LOAD_V__
`define __LOAD_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

// ===============================================
// Module: Load
// Description: Handle load instructions execution
// ===============================================

typedef struct packed {
    logic [`SYS_LSQ_ADDR_WIDTH-1:0]    lsq_tail_alloc;
    FU_COMPLETE_PACKET  result;
    logic [`SYS_XLEN-1:0]   addr;
    LS_SELECT           load_sel;
    logic [3:0]		    usebytes, forward_bytes;
    logic [`SYS_XLEN-1:0]   aligned_data;
} LD_STAGE_REGISTER;

typedef enum logic [1:0] {
    WAITING_INPUT = 0,
    WAITING_SQ,
    WAITING_CACHE,
    WAITING_OUTPUT
} LD_STAGE_STATE;

module fu_load(
    input                       clk,
    input                       rst,
    input                       bs_hazard,
    input ISSUE_FU_PACKET       bs_in_pkt,
    output logic                rsb_fu_ready,
    output logic                fum_complete_req,
    output FU_COMPLETE_PACKET   bs_out_pkt,

    // Interface to Store Queue (SQ)
    output LOAD_SQ_PACKET       ld_sq_request,
    input SQ_LOAD_PACKET        ld_sq_response,

    // Interface to Data Cache
    output logic [`SYS_XLEN-1:0]    addr,
    output logic                ld_cache_read_enable,             
    input [`SYS_XLEN-1:0]           ld_cache_data_in,
    input                       exs_dcache_hit_flags, // If hit, get the data from cache. If not, get from CDB
    input                       broadcast_en,
    input [`SYS_XLEN-1:0]           exs_dcache_brdcast_data
);

// Internal state
LD_STAGE_STATE status;
LD_STAGE_REGISTER ld_pipeline_reg;

// Handle input readiness
assign rsb_fu_ready = ~bs_in_pkt.valid && (status == WAITING_INPUT);

// Calculate load address
logic [`SYS_XLEN-1:0] ld_calc_addr;
assign ld_calc_addr = bs_in_pkt.r1_value + `RV32_signext_Iimm(bs_in_pkt.inst);


// Build output packet basic fields
FU_COMPLETE_PACKET ld_initial_res_pkt;
always_comb begin
    ld_initial_res_pkt = 0;
    ld_initial_res_pkt.valid = bs_in_pkt.valid;
    ld_initial_res_pkt.dispatch_allocated_prs = bs_in_pkt.dispatch_allocated_prs;
    ld_initial_res_pkt.rob_entry = bs_in_pkt.rob_entry;
end

// Decode which bytes are needed from memory
logic [3:0] ld_decode_usebytes;
always_comb begin
    unique case(bs_in_pkt.dec_fu_opcode.ls)
    LB, LBU: ld_decode_usebytes = 4'b0001 << ld_calc_addr[1:0];
    LH, LHU: ld_decode_usebytes = (ld_calc_addr[1]) ? 4'b1100 : 4'b0011;
    LW: ld_decode_usebytes = 4'b1111;
    default: ld_decode_usebytes = 4'b0000;
    endcase
end

// Store Queue Lookup
assign ld_sq_request.addr = {ld_pipeline_reg.addr[`SYS_XLEN-1:2], 2'b0};
assign ld_sq_request.lsq_tail_alloc = ld_pipeline_reg.lsq_tail_alloc;

// Check if Store Queue can fully forward the requested load data
logic [3:0] ld_sq_fwd_mask = ld_pipeline_reg.usebytes & ld_sq_response.usebytes;
logic ld_sq_forward_ok;
always_comb begin
    if (ld_sq_fwd_mask == ld_pipeline_reg.usebytes) ld_sq_forward_ok = 1'b1;
    else ld_sq_forward_ok = 1'b0;
end

// Cache Access
integer i;
assign addr = {ld_pipeline_reg.addr[`SYS_XLEN-1:2], 2'b0};
logic [`SYS_XLEN-1:0] ld_cache_fetched_data, data_after_cache;
assign ld_cache_fetched_data = exs_dcache_hit_flags ? ld_cache_data_in : 0;
// Merge forwarded bytes from SQ and cache data
always_comb begin
    data_after_cache = ld_cache_fetched_data;
    for (i = 0; i < 4; i = i + 1) begin
        if (ld_pipeline_reg.forward_bytes[i]) data_after_cache[8*i +: 8] = ld_pipeline_reg.aligned_data[8*i +: 8];
    end
end

// Output value reconstruction (load data formatting)
logic [`SYS_XLEN-1:0] wb_data;
always_comb begin
    wb_data = 0;
    case(ld_pipeline_reg.load_sel)
    LB: wb_data = {{24{ld_pipeline_reg.aligned_data[8*ld_pipeline_reg.addr[1:0] + 7]}}, ld_pipeline_reg.aligned_data[8*ld_pipeline_reg.addr[1:0] +: 8]};
    LH: wb_data = ld_pipeline_reg.addr[1] ? {{16{ld_pipeline_reg.aligned_data[31]}}, ld_pipeline_reg.aligned_data[31:16]} : {{16{ld_pipeline_reg.aligned_data[15]}}, ld_pipeline_reg.aligned_data[15:0]};
    LW: wb_data = ld_pipeline_reg.aligned_data;
    LBU: wb_data[7:0] = ld_pipeline_reg.aligned_data[8*ld_pipeline_reg.addr[1:0] +: 8];
    LHU: wb_data[15:0] = ld_pipeline_reg.addr[1] ? ld_pipeline_reg.aligned_data[31:16] : ld_pipeline_reg.aligned_data[15:0];
    endcase
end

// Final complete packet with loaded data
FU_COMPLETE_PACKET updated_result;
always_comb begin
    updated_result = ld_pipeline_reg.result;
    updated_result.dest_value = wb_data;
end

assign bs_out_pkt = updated_result;
assign fum_complete_req = status == WAITING_OUTPUT;


// Main sequential logic
always_ff @(posedge clk) begin
    if (rst) begin
        ld_pipeline_reg <= `SYS_SMALL_DELAY 0;
        status <= `SYS_SMALL_DELAY WAITING_INPUT;
        ld_cache_read_enable <= `SYS_SMALL_DELAY 0;
    end else begin
        // Default setting
        ld_cache_read_enable <= `SYS_SMALL_DELAY 0;
        case (status)
        WAITING_INPUT: begin
            status <= `SYS_SMALL_DELAY bs_in_pkt.valid && ~bs_hazard ? WAITING_SQ : WAITING_INPUT;
            ld_pipeline_reg.result <= `SYS_SMALL_DELAY ld_initial_res_pkt;
            ld_pipeline_reg.addr <= `SYS_SMALL_DELAY ld_calc_addr;
            ld_pipeline_reg.lsq_tail_alloc <= `SYS_SMALL_DELAY bs_in_pkt.sq_tail;
            ld_pipeline_reg.load_sel <= `SYS_SMALL_DELAY bs_in_pkt.dec_fu_opcode.ls;
            ld_pipeline_reg.usebytes <= `SYS_SMALL_DELAY ld_decode_usebytes;
            ld_pipeline_reg.forward_bytes <= `SYS_SMALL_DELAY 0;
            ld_pipeline_reg.aligned_data <= `SYS_SMALL_DELAY 0;
        end
        WAITING_SQ: begin
            status <= `SYS_SMALL_DELAY   ld_sq_response.lsq_stall_mask ? WAITING_SQ :
                            ld_sq_forward_ok ? WAITING_OUTPUT
                            : WAITING_CACHE;
            ld_pipeline_reg.forward_bytes <= `SYS_SMALL_DELAY ld_sq_response.usebytes;
            ld_pipeline_reg.aligned_data <= `SYS_SMALL_DELAY ld_sq_response.data;
            if (~ld_sq_response.lsq_stall_mask && ~ld_sq_forward_ok) ld_cache_read_enable <= `SYS_SMALL_DELAY 1;
        end
        WAITING_CACHE: begin
            status <= `SYS_SMALL_DELAY (!exs_dcache_hit_flags) ? WAITING_CACHE : WAITING_OUTPUT;
            ld_pipeline_reg.aligned_data <= `SYS_SMALL_DELAY data_after_cache;
        end
        WAITING_OUTPUT: begin
            status <= `SYS_SMALL_DELAY WAITING_INPUT;
        end
        endcase
    end
end

endmodule

`endif 

