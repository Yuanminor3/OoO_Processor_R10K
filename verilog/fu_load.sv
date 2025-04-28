
`ifndef __LOAD_V__
`define __LOAD_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

// ===============================================
// Module: Load
// Description: Handle load instructions execution
// ===============================================

typedef struct packed {
    logic [`LSQ-1:0]    tail_pos;
    FU_COMPLETE_PACKET  result;
    logic [`XLEN-1:0]   addr;
    LS_SELECT           load_sel;
    logic [3:0]		    usebytes, forward_bytes;
    logic [`XLEN-1:0]   aligned_data;
} LOAD_STAGE_REG;

typedef enum logic [1:0] {
    WAITING_INPUT = 0,
    WAITING_SQ,
    WAITING_CACHE,
    WAITING_OUTPUT
} LOAD_STAGE_STATUS;

module fu_load(
    input                       clock,
    input                       reset,
    input                       complete_stall,
    input ISSUE_FU_PACKET       fu_packet_in,
    output logic                fu_ready,
    output logic                want_to_complete,
    output FU_COMPLETE_PACKET   fu_packet_out,

    // Interface to Store Queue (SQ)
    output LOAD_SQ_PACKET       sq_lookup,
    input SQ_LOAD_PACKET        sq_result,

    // Interface to Data Cache
    output logic [`XLEN-1:0]    addr,
    output logic                cache_read_EN,             
    input [`XLEN-1:0]           cache_data_in,
    input                       is_hit, // If hit, get the data from cache. If not, get from CDB
    input                       broadcast_en,
    input [`XLEN-1:0]           broadcast_data
);

// Internal state
LOAD_STAGE_STATUS status;
LOAD_STAGE_REG ins_reg;

// Handle input readiness
assign fu_ready = ~fu_packet_in.valid && (status == WAITING_INPUT);

// Calculate load address
logic [`XLEN-1:0] new_addr;
assign new_addr = fu_packet_in.r1_value + `RV32_signext_Iimm(fu_packet_in.inst);


// Build output packet basic fields
FU_COMPLETE_PACKET new_result_pckt;
always_comb begin
    new_result_pckt = 0;
    new_result_pckt.valid = fu_packet_in.valid;
    new_result_pckt.dest_pr = fu_packet_in.dest_pr;
    new_result_pckt.rob_entry = fu_packet_in.rob_entry;
end

// Decode which bytes are needed from memory
logic [3:0] new_usebytes;
always_comb begin
    unique case(fu_packet_in.op_sel.ls)
    LB, LBU: new_usebytes = 4'b0001 << new_addr[1:0];
    LH, LHU: new_usebytes = (new_addr[1]) ? 4'b1100 : 4'b0011;
    LW: new_usebytes = 4'b1111;
    default: new_usebytes = 4'b0000;
    endcase
end

// Store Queue Lookup
assign sq_lookup.addr = {ins_reg.addr[`XLEN-1:2], 2'b0};
assign sq_lookup.tail_pos = ins_reg.tail_pos;

// Check if Store Queue can fully forward the requested load data
logic [3:0] sq_forward_bytes = ins_reg.usebytes & sq_result.usebytes;
logic sq_forward;
always_comb begin
    if (sq_forward_bytes == ins_reg.usebytes) sq_forward = 1'b1;
    else sq_forward = 1'b0;
end

// Cache Access
integer i;
assign addr = {ins_reg.addr[`XLEN-1:2], 2'b0};
logic [`XLEN-1:0] cache_data, data_after_cache;
assign cache_data = is_hit ? cache_data_in : 0;
// Merge forwarded bytes from SQ and cache data
always_comb begin
    data_after_cache = cache_data;
    for (i = 0; i < 4; i = i + 1) begin
        if (ins_reg.forward_bytes[i]) data_after_cache[8*i +: 8] = ins_reg.aligned_data[8*i +: 8];
    end
end

// Output value reconstruction (load data formatting)
logic [`XLEN-1:0] wb_data;
always_comb begin
    wb_data = 0;
    case(ins_reg.load_sel)
    LB: wb_data = {{24{ins_reg.aligned_data[8*ins_reg.addr[1:0] + 7]}}, ins_reg.aligned_data[8*ins_reg.addr[1:0] +: 8]};
    LH: wb_data = ins_reg.addr[1] ? {{16{ins_reg.aligned_data[31]}}, ins_reg.aligned_data[31:16]} : {{16{ins_reg.aligned_data[15]}}, ins_reg.aligned_data[15:0]};
    LW: wb_data = ins_reg.aligned_data;
    LBU: wb_data[7:0] = ins_reg.aligned_data[8*ins_reg.addr[1:0] +: 8];
    LHU: wb_data[15:0] = ins_reg.addr[1] ? ins_reg.aligned_data[31:16] : ins_reg.aligned_data[15:0];
    endcase
end

// Final complete packet with loaded data
FU_COMPLETE_PACKET updated_result;
always_comb begin
    updated_result = ins_reg.result;
    updated_result.dest_value = wb_data;
end

assign fu_packet_out = updated_result;
assign want_to_complete = status == WAITING_OUTPUT;


// Main sequential logic
always_ff @(posedge clock) begin
    if (reset) begin
        ins_reg <= `SD 0;
        status <= `SD WAITING_INPUT;
        cache_read_EN <= `SD 0;
    end else begin
        // Default setting
        cache_read_EN <= `SD 0;
        case (status)
        WAITING_INPUT: begin
            status <= `SD fu_packet_in.valid && ~complete_stall ? WAITING_SQ : WAITING_INPUT;
            ins_reg.result <= `SD new_result_pckt;
            ins_reg.addr <= `SD new_addr;
            ins_reg.tail_pos <= `SD fu_packet_in.sq_tail;
            ins_reg.load_sel <= `SD fu_packet_in.op_sel.ls;
            ins_reg.usebytes <= `SD new_usebytes;
            ins_reg.forward_bytes <= `SD 0;
            ins_reg.aligned_data <= `SD 0;
        end
        WAITING_SQ: begin
            status <= `SD   sq_result.stall ? WAITING_SQ :
                            sq_forward ? WAITING_OUTPUT
                            : WAITING_CACHE;
            ins_reg.forward_bytes <= `SD sq_result.usebytes;
            ins_reg.aligned_data <= `SD sq_result.data;
            if (~sq_result.stall && ~sq_forward) cache_read_EN <= `SD 1;
        end
        WAITING_CACHE: begin
            status <= `SD (!is_hit) ? WAITING_CACHE : WAITING_OUTPUT;
            ins_reg.aligned_data <= `SD data_after_cache;
        end
        WAITING_OUTPUT: begin
            status <= `SD WAITING_INPUT;
        end
        endcase
    end
end

endmodule

`endif 

