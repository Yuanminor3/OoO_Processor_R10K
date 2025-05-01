
`ifndef __DISPATCH_V__
`define __DISPATCH_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/decoder.sv"

module dispatch_stage (

    //================================
    //        Input Packets
    //================================
    input IF_ID_PACKET [2:0] dis_packet_in,

    //================================
    //        Free Register Allocation
    //================================
    input [2:0][`PR-1:0] free_pr_in,  // Free physical registers from Free List

    //================================
    //        Map Table Lookup
    //================================
    input [2:0][`PR-1:0] reg1_pr,     // PR mapping for reg1
    input [2:0][`PR-1:0] reg2_pr,     // PR mapping for reg2
    input [2:0] reg1_ready,           // reg1 ready status
    input [2:0] reg2_ready,           // reg2 ready status
    input [2:0][`PR-1:0] maptable_old_pr, // Old physical register mapping for destination

    //================================
    //        ROB Info
    //================================
    input [2:0][`ROB-1:0] rob_index,  // Allocated ROB index for dispatched instructions

    //================================
    //        SQ Info
    //================================
    input [2:0][`LSQ-1:0] sq_tail_pos, // Store Queue tail pointer position

    /* Dispatch Stall */
    input logic [2:0] d_stall,          // Stall signals per instruction

    //================================
    //        Outputs
    //================================

    /* RS Allocation */
    output RS_IN_PACKET [2:0] rs_in, 

    /* ROB Allocation */
    output ROB_ENTRY_PACKET [2:0] rob_in,

    /* Free Register Control */
    output logic [2:0] new_pr_en,        // New PR allocation enable
    output logic [2:0][`PR-1:0] maptable_new_pr, // New PR for destination arch reg
    output logic [2:0][4:0] maptable_ar, // Destination architectural reg

    /* Map Table Lookup */
    output logic [2:0][4:0] reg1_ar,     // Source register 1 (arch reg)
    output logic [2:0][4:0] reg2_ar,     // Source register 2 (arch reg)

    /* SQ Allocation */
    output logic [2:0] sq_alloc,         // Store Queue allocation signal

    output 	FU_SELECT [2:0] fu_sel_out,
    output IF_ID_PACKET [2:0] dis_packet_out

);

// Internal signals
IF_ID_PACKET [2:0] dis_packet;
logic [2:0] halt;
logic [2:0] is_store;
FU_SELECT [2:0] fu_sel;
OP_SELECT [2:0] op_sel;
ALU_OPA_SELECT [2:0] opa_select;
ALU_OPB_SELECT [2:0] opb_select;
logic [2:0][4:0] dest_arch, reg1_arch, reg2_arch;
logic [2:0][`PR-1:0] dest_pr;

assign fu_sel_out = fu_sel;
assign dis_packet_out = dis_packet;
//to do --> change to for loop

always_comb begin
    dis_packet = dis_packet_in;

    for (int i = 2; i >= 0; i--) begin
        if (i == 2) begin
            dis_packet[i].valid = dis_packet_in[i].valid && !d_stall[i];
        end
        else if (i == 1) begin
            if (dis_packet_in[2].predict_direction)
                dis_packet[i].valid = 0;
            else
                dis_packet[i].valid = dis_packet_in[i].valid && !d_stall[i];
        end
        else if (i == 0) begin
            if (dis_packet_in[2].predict_direction || dis_packet_in[1].predict_direction)
                dis_packet[i].valid = 0;
            else
                dis_packet[i].valid = dis_packet_in[i].valid && !d_stall[i];
        end
    end
end

// Decode each instruction
// (Each decoder outputs ALU control, dest reg, src regs, etc.)
decoder decode_0(
    .if_packet(dis_packet[0]),
    .fu_sel(fu_sel[0]), .op_sel(op_sel[0]),
    .opa_select(opa_select[0]), .opb_select(opb_select[0]),
    .dest_reg(dest_arch[0]), .reg1(reg1_arch[0]), .reg2(reg2_arch[0]),
    .halt(halt[0]), .is_store(is_store[0])
);
decoder decode_1(
    .if_packet(dis_packet[1]),
    .fu_sel(fu_sel[1]), .op_sel(op_sel[1]),
    .opa_select(opa_select[1]), .opb_select(opb_select[1]),
    .dest_reg(dest_arch[1]), .reg1(reg1_arch[1]), .reg2(reg2_arch[1]),
    .halt(halt[1]), .is_store(is_store[1])
);
decoder decode_2(
    .if_packet(dis_packet[2]),
    .fu_sel(fu_sel[2]), .op_sel(op_sel[2]),
    .opa_select(opa_select[2]), .opb_select(opb_select[2]),
    .dest_reg(dest_arch[2]), .reg1(reg1_arch[2]), .reg2(reg2_arch[2]),
    .halt(halt[2]), .is_store(is_store[2])
);

always_comb begin
    for (int i = 0; i < 3; i++) begin
        new_pr_en[i]       = dis_packet[i].valid && (dest_arch[i] != `ZERO_REG);
        dest_pr[i]         = new_pr_en[i] ? free_pr_in[i] : `ZERO_REG;
        maptable_ar[i]     = dis_packet[i].valid ? dest_arch[i] : `ZERO_REG;
        maptable_new_pr[i] = dest_pr[i];
    end
end


assign reg1_ar = reg1_arch;
assign reg2_ar = reg2_arch;

// Allocate store queue entries
always_comb begin
    sq_alloc = 0;
    for (int i = 0; i < 3; i++) begin
        if (is_store[i] && dis_packet[i].valid)
            sq_alloc[i] = 1;
    end
end

// Fill ROB entries
always_comb begin
    for (int i = 0; i < 3; i++) begin
        rob_in[i].NPC              = dis_packet[i].NPC;
        rob_in[i].PC               = dis_packet[i].PC;
        rob_in[i].valid            = dis_packet[i].valid;
        rob_in[i].Tnew             = dest_pr[i];
        rob_in[i].Told             = maptable_old_pr[i];
        rob_in[i].halt             = halt[i];
        rob_in[i].arch_reg         = dest_arch[i];
        rob_in[i].completed        = 0;
        rob_in[i].precise_state_need = 0;
        rob_in[i].is_store         = is_store[i];
        rob_in[i].target_pc        = 0;
        rob_in[i].inst             = dis_packet[i].inst;
    end
end

always_comb begin
    // Packet 2: always included
    rob_in[2].predict_direction = dis_packet[2].predict_direction;
    rob_in[2].predict_pc        = dis_packet[2].predict_direction ? dis_packet[2].predict_pc : '0;

    // Packet 1: only included if packet 2 is not taken
    if (!dis_packet[2].predict_direction) begin
        rob_in[1].predict_direction = dis_packet[1].predict_direction;
        rob_in[1].predict_pc        = dis_packet[1].predict_direction ? dis_packet[1].predict_pc : '0;
    end else begin
        rob_in[1].predict_direction = '0;
        rob_in[1].predict_pc        = '0;
    end

    // Packet 0: only included if packet 2 and 1 are not taken
    if (!dis_packet[2].predict_direction && !dis_packet[1].predict_direction) begin
        rob_in[0].predict_direction = dis_packet[0].predict_direction;
        rob_in[0].predict_pc        = dis_packet[0].predict_direction ? dis_packet[0].predict_pc : '0;
    end else begin
        rob_in[0].predict_direction = '0;
        rob_in[0].predict_pc        = '0;
    end
end

// Fill RS entries
always_comb begin
    for (int i = 0; i < 3; i++) begin
        rs_in[i].valid        = dis_packet[i].valid;
        rs_in[i].fu_sel       = fu_sel[i];
        rs_in[i].op_sel       = op_sel[i];
        rs_in[i].NPC          = dis_packet[i].NPC;
        rs_in[i].PC           = dis_packet[i].PC;
        rs_in[i].opa_select   = opa_select[i];
        rs_in[i].opb_select   = opb_select[i];
        rs_in[i].inst         = dis_packet[i].inst;
        rs_in[i].halt         = halt;
        rs_in[i].rob_entry    = rob_index[i];
        rs_in[i].sq_tail      = sq_tail_pos[i];
        rs_in[i].dest_pr      = dest_pr[i];
        rs_in[i].reg1_pr      = reg1_pr[i];
        rs_in[i].reg1_ready   = reg1_ready[i];
        rs_in[i].reg2_pr      = reg2_pr[i];
        rs_in[i].reg2_ready   = reg2_ready[i];
    end
end

endmodule
`endif // _DISPATCH_V_

