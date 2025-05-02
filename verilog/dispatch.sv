
`ifndef __DISPATCH_V__
`define __DISPATCH_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/decoder.sv"

module dispatch_stage (

    //================================
    //        Input Packets
    //================================
    input IF_ID_PACKET [2:0] dispatch_if_pkts,

    //================================
    //        Free Register Allocation
    //================================
    input [2:0][`SYS_PHYS_REG-1:0] dispatch_free_prs,  // Free physical registers from Free List

    //================================
    //        Map Table Lookup
    //================================
    input [2:0][`SYS_PHYS_REG-1:0] dispatch_src1_pr,     // SYS_PHYS_REG mapping for dec_src1_reg
    input [2:0][`SYS_PHYS_REG-1:0] dispatch_src2_pr,     // SYS_PHYS_REG mapping for dec_src2_reg
    input [2:0] dispatch_src1_rdy,           // dec_src1_reg ready status
    input [2:0] dispatch_src2_rdy,           // dec_src2_reg ready status
    input [2:0][`SYS_PHYS_REG-1:0] dispatch_oldprs, // Old physical register mapping for destination

    //================================
    //        SYS_ROB_ADDR_WIDTH Info
    //================================
    input [2:0][`SYS_ROB_ADDR_WIDTH-1:0] dispatch_idx,  // Allocated SYS_ROB_ADDR_WIDTH index for dispatched instructions

    //================================
    //        SQ Info
    //================================
    input [2:0][`SYS_LSQ_ADDR_WIDTH-1:0] dispatch_pointer_tail, // Store Queue fl_tail_reg pointer position

    /* Dispatch Stall */
    input logic [2:0] dispatch_stall_mask,          // Stall signals per instruction

    //================================
    //        Outputs
    //================================

    /* RS Allocation */
    output RS_S_PACKET [2:0] dispatch_rs_pkts, 

    /* SYS_ROB_ADDR_WIDTH Allocation */
    output ROB_ENTRY_PACKET [2:0] dispatch_rob_pkts,

    /* Free Register Control */
    output logic [2:0] dispatch_pr_allocEN,        // New SYS_PHYS_REG allocation enable
    output logic [2:0][`SYS_PHYS_REG-1:0] dispatch_pr_alloc_tags, // New SYS_PHYS_REG for destination arch reg
    output logic [2:0][4:0] dispatch_arch_regs, // Destination architectural reg

    /* Map Table Lookup */
    output logic [2:0][4:0] dispatch_src1_arch_regs,     // Source register 1 (arch reg)
    output logic [2:0][4:0] dispatch_src2_arch_regs,     // Source register 2 (arch reg)

    /* SQ Allocation */
    output logic [2:0] dispatch_sq_flags,         // Store Queue allocation signal

    output 	FU_SELECT [2:0] dispatc_unit_sel,
    output IF_ID_PACKET [2:0] dispatch_if_pkts_out

);

// Internal signals
IF_ID_PACKET [2:0] dispatch_pkt_buffer;
logic [2:0] halt;
logic [2:0] dec_is_store_flag;
FU_SELECT [2:0] dec_fu_unit_sel;
OP_SELECT [2:0] dec_fu_opcode;
ALU_OPA_SELECT [2:0] dec_operandA_mux;
ALU_OPB_SELECT [2:0] dec_operandB_mux;
logic [2:0][4:0] dispatch_dest_regs, dispatch_src1_arch_regs_local, dispatch_src2_arch_regs_local;
logic [2:0][`SYS_PHYS_REG-1:0] dispatch_allocated_prs;

assign dispatc_unit_sel = dec_fu_unit_sel;
assign dispatch_if_pkts_out = dispatch_pkt_buffer;
//to do --> change to for loop

always_comb begin
    dispatch_pkt_buffer = dispatch_if_pkts;

    for (int i = 2; i >= 0; i--) begin
        if (i == 2) begin
            dispatch_pkt_buffer[i].valid = dispatch_if_pkts[i].valid && !dispatch_stall_mask[i];
        end
        else if (i == 1) begin
            if (dispatch_if_pkts[2].bp_pred_taken)
                dispatch_pkt_buffer[i].valid = 0;
            else
                dispatch_pkt_buffer[i].valid = dispatch_if_pkts[i].valid && !dispatch_stall_mask[i];
        end
        else if (i == 0) begin
            if (dispatch_if_pkts[2].bp_pred_taken || dispatch_if_pkts[1].bp_pred_taken)
                dispatch_pkt_buffer[i].valid = 0;
            else
                dispatch_pkt_buffer[i].valid = dispatch_if_pkts[i].valid && !dispatch_stall_mask[i];
        end
    end
end

// Decode each instruction
// (Each decoder outputs ALU control, dest reg, src regs, etc.)
decoder decode_0(
    .decode_pkt(dispatch_pkt_buffer[0]),
    .dec_fu_unit_sel(dec_fu_unit_sel[0]), .dec_fu_opcode(dec_fu_opcode[0]),
    .dec_operandA_mux(dec_operandA_mux[0]), .dec_operandB_mux(dec_operandB_mux[0]),
    .dec_dst_arch_reg(dispatch_dest_regs[0]), .dec_src1_reg(dispatch_src1_arch_regs_local[0]), .dec_src2_reg(dispatch_src2_arch_regs_local[0]),
    .halt(halt[0]), .dec_is_store_flag(dec_is_store_flag[0])
);
decoder decode_1(
    .decode_pkt(dispatch_pkt_buffer[1]),
    .dec_fu_unit_sel(dec_fu_unit_sel[1]), .dec_fu_opcode(dec_fu_opcode[1]),
    .dec_operandA_mux(dec_operandA_mux[1]), .dec_operandB_mux(dec_operandB_mux[1]),
    .dec_dst_arch_reg(dispatch_dest_regs[1]), .dec_src1_reg(dispatch_src1_arch_regs_local[1]), .dec_src2_reg(dispatch_src2_arch_regs_local[1]),
    .halt(halt[1]), .dec_is_store_flag(dec_is_store_flag[1])
);
decoder decode_2(
    .decode_pkt(dispatch_pkt_buffer[2]),
    .dec_fu_unit_sel(dec_fu_unit_sel[2]), .dec_fu_opcode(dec_fu_opcode[2]),
    .dec_operandA_mux(dec_operandA_mux[2]), .dec_operandB_mux(dec_operandB_mux[2]),
    .dec_dst_arch_reg(dispatch_dest_regs[2]), .dec_src1_reg(dispatch_src1_arch_regs_local[2]), .dec_src2_reg(dispatch_src2_arch_regs_local[2]),
    .halt(halt[2]), .dec_is_store_flag(dec_is_store_flag[2])
);

always_comb begin
    for (int i = 0; i < 3; i++) begin
        dispatch_pr_allocEN[i]       = dispatch_pkt_buffer[i].valid && (dispatch_dest_regs[i] != `SYS_ZERO_ARCH_REG);
        dispatch_allocated_prs[i]         = dispatch_pr_allocEN[i] ? dispatch_free_prs[i] : `SYS_ZERO_ARCH_REG;
        dispatch_arch_regs[i]     = dispatch_pkt_buffer[i].valid ? dispatch_dest_regs[i] : `SYS_ZERO_ARCH_REG;
        dispatch_pr_alloc_tags[i] = dispatch_allocated_prs[i];
    end
end


assign dispatch_src1_arch_regs = dispatch_src1_arch_regs_local;
assign dispatch_src2_arch_regs = dispatch_src2_arch_regs_local;

// Allocate store queue entries
always_comb begin
    dispatch_sq_flags = 0;
    for (int i = 0; i < 3; i++) begin
        if (dec_is_store_flag[i] && dispatch_pkt_buffer[i].valid)
            dispatch_sq_flags[i] = 1;
    end
end

// Fill SYS_ROB_ADDR_WIDTH entries
always_comb begin
    for (int i = 0; i < 3; i++) begin
        dispatch_rob_pkts[i].NPC              = dispatch_pkt_buffer[i].NPC;
        dispatch_rob_pkts[i].PC               = dispatch_pkt_buffer[i].PC;
        dispatch_rob_pkts[i].valid            = dispatch_pkt_buffer[i].valid;
        dispatch_rob_pkts[i].Tnew             = dispatch_allocated_prs[i];
        dispatch_rob_pkts[i].Told             = dispatch_oldprs[i];
        dispatch_rob_pkts[i].halt             = halt[i];
        dispatch_rob_pkts[i].arch_reg         = dispatch_dest_regs[i];
        dispatch_rob_pkts[i].completed        = 0;
        dispatch_rob_pkts[i].precise_state_need = 0;
        dispatch_rob_pkts[i].dec_is_store_flag         = dec_is_store_flag[i];
        dispatch_rob_pkts[i].cs_retire_pc        = 0;
        dispatch_rob_pkts[i].inst             = dispatch_pkt_buffer[i].inst;
    end
end

always_comb begin
    // Packet 2: always included
    dispatch_rob_pkts[2].bp_pred_taken = dispatch_pkt_buffer[2].bp_pred_taken;
    dispatch_rob_pkts[2].bp_pred_target        = dispatch_pkt_buffer[2].bp_pred_taken ? dispatch_pkt_buffer[2].bp_pred_target : '0;

    // Packet 1: only included if packet 2 is not taken
    if (!dispatch_pkt_buffer[2].bp_pred_taken) begin
        dispatch_rob_pkts[1].bp_pred_taken = dispatch_pkt_buffer[1].bp_pred_taken;
        dispatch_rob_pkts[1].bp_pred_target        = dispatch_pkt_buffer[1].bp_pred_taken ? dispatch_pkt_buffer[1].bp_pred_target : '0;
    end else begin
        dispatch_rob_pkts[1].bp_pred_taken = '0;
        dispatch_rob_pkts[1].bp_pred_target        = '0;
    end

    // Packet 0: only included if packet 2 and 1 are not taken
    if (!dispatch_pkt_buffer[2].bp_pred_taken && !dispatch_pkt_buffer[1].bp_pred_taken) begin
        dispatch_rob_pkts[0].bp_pred_taken = dispatch_pkt_buffer[0].bp_pred_taken;
        dispatch_rob_pkts[0].bp_pred_target        = dispatch_pkt_buffer[0].bp_pred_taken ? dispatch_pkt_buffer[0].bp_pred_target : '0;
    end else begin
        dispatch_rob_pkts[0].bp_pred_taken = '0;
        dispatch_rob_pkts[0].bp_pred_target        = '0;
    end
end

// Fill RS entries
always_comb begin
    for (int i = 0; i < 3; i++) begin
        dispatch_rs_pkts[i].valid        = dispatch_pkt_buffer[i].valid;
        dispatch_rs_pkts[i].dec_fu_unit_sel       = dec_fu_unit_sel[i];
        dispatch_rs_pkts[i].dec_fu_opcode       = dec_fu_opcode[i];
        dispatch_rs_pkts[i].NPC          = dispatch_pkt_buffer[i].NPC;
        dispatch_rs_pkts[i].PC           = dispatch_pkt_buffer[i].PC;
        dispatch_rs_pkts[i].dec_operandA_mux   = dec_operandA_mux[i];
        dispatch_rs_pkts[i].dec_operandB_mux   = dec_operandB_mux[i];
        dispatch_rs_pkts[i].inst         = dispatch_pkt_buffer[i].inst;
        dispatch_rs_pkts[i].halt         = halt;
        dispatch_rs_pkts[i].rob_entry    = dispatch_idx[i];
        dispatch_rs_pkts[i].sq_tail      = dispatch_pointer_tail[i];
        dispatch_rs_pkts[i].dispatch_allocated_prs      = dispatch_allocated_prs[i];
        dispatch_rs_pkts[i].dispatch_src1_pr      = dispatch_src1_pr[i];
        dispatch_rs_pkts[i].dispatch_src1_rdy   = dispatch_src1_rdy[i];
        dispatch_rs_pkts[i].dispatch_src2_pr      = dispatch_src2_pr[i];
        dispatch_rs_pkts[i].dispatch_src2_rdy   = dispatch_src2_rdy[i];
    end
end

endmodule
`endif // _DISPATCH_V_

