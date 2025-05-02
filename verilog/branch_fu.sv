//////////////////////////////////////////////////////////////////////////
//                                                                      //
//   Modulename :  branch_fu.sv                                         //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
`ifndef __BRANCH_FU_SV__
`define __BRANCH_FU_SV__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

// brcond: compute branch-taken flag using parallel comparisons
module brcond(
    input  logic [`SYS_XLEN-1:0] cmp_opA,    // first operand
    input  logic [`SYS_XLEN-1:0] cmp_opB,    // second operand
    input  BR_SELECT        br,      // branch type selector
    output logic            cond     // result: 1 => branch taken
);
    // preserve signed views for signed comparisons
    logic signed [`SYS_XLEN-1:0] signed_rs1;
    logic signed [`SYS_XLEN-1:0] signed_rs2;
    assign signed_rs1 = cmp_opA;
    assign signed_rs2 = cmp_opB;

    // parallel comparison signals
    logic cmp_eq_flag, ne, cmp_lt_flag, cmp_ge_flag, cmp_ltu_flag, cmp_geu_flag;
    assign cmp_eq_flag  = (signed_rs1 == signed_rs2);
    assign ne  = (signed_rs1 != signed_rs2);
    assign cmp_lt_flag  = (signed_rs1 <  signed_rs2);
    assign cmp_ge_flag  = (signed_rs1 >= signed_rs2);
    assign cmp_ltu_flag = (cmp_opA < cmp_opB);
    assign cmp_geu_flag = (cmp_opA >= cmp_opB);

    // combine comparisons based on branch type
    always_comb begin
        cond = 1'b0;
        cond |= (br == UNCOND)  ? 1'b1 : 1'b0;
        cond |= (br == BEQ)     ? cmp_eq_flag  : 1'b0;
        cond |= (br == BNE)     ? ne  : 1'b0;
        cond |= (br == BLT)     ? cmp_lt_flag  : 1'b0;
        cond |= (br == BGE)     ? cmp_ge_flag  : 1'b0;
        cond |= (br == BLTU)    ? cmp_ltu_flag : 1'b0;
        cond |= (br == BGEU)    ? cmp_geu_flag : 1'b0;
    end
endmodule 

// branch_stage: retain same two-level MUX + brcond + adder structure
module branch_stage(
    input                  clk,
    input                  rst,
    input                  bs_hazard,       // structural hazard
    input  ISSUE_FU_PACKET bs_in_pkt,         // SYS_FU_ADDR_WIDTH input packet
    output logic           rsb_fu_ready,             // always ready
    output logic           bs_fire,
    output FU_COMPLETE_PACKET bs_out_pkt,     // to lsq_retire_mask
    output logic           bs_upd_en,            // predictor update enable
    output logic [`SYS_XLEN-1:0] bs_upd_pc,
    output logic           bs_upd_taken,
    output logic [`SYS_XLEN-1:0] bs_upd_target
);

    // flag when SYS_FU_ADDR_WIDTH result is valid
    wire fu_complete = bs_in_pkt.valid;
    assign bs_fire = fu_complete;
    assign rsb_fu_ready               = 1'b1;

    // operand-A multiplexer (1st MUX)
    logic [`SYS_XLEN-1:0] opa_mux_out;
    always_comb begin
        opa_mux_out = `SYS_XLEN'h0;
        case (bs_in_pkt.dec_operandA_mux)
            OPA_IS_RS1:  opa_mux_out = bs_in_pkt.r1_value;
            OPA_IS_NPC:  opa_mux_out = bs_in_pkt.NPC;
            OPA_IS_PC:   opa_mux_out = bs_in_pkt.PC;
            OPA_IS_ZERO: opa_mux_out = '0;
        endcase
    end

    // operand-B multiplexer (2nd MUX)
    logic [`SYS_XLEN-1:0] opb_mux_out;
    always_comb begin
        opb_mux_out = `SYS_XLEN'h0;
        case (bs_in_pkt.dec_operandB_mux)
            OPB_IS_RS2:   opb_mux_out = bs_in_pkt.r2_value;
            OPB_IS_I_IMM: opb_mux_out = `RV32_signext_Iimm(bs_in_pkt.inst);
            OPB_IS_B_IMM: opb_mux_out = `RV32_signext_Bimm(bs_in_pkt.inst);
            OPB_IS_J_IMM: opb_mux_out = `RV32_signext_Jimm(bs_in_pkt.inst);
        endcase
    end

    // branch condition checker instance
    logic brcond_result;
    brcond brcond_inst (
        .cmp_opA  (bs_in_pkt.r1_value),
        .cmp_opB  (bs_in_pkt.r2_value),
        .br   (bs_in_pkt.dec_fu_opcode.br),
        .cond (brcond_result)
    );

    // adder result for branch target
    logic [`SYS_XLEN-1:0] sum_addr;
    always_comb begin
        sum_addr = opa_mux_out + opb_mux_out;
    end

    // final outputs and predictor updates
    always_comb begin
        // predictor interface
        bs_upd_en        = fu_complete;
        bs_upd_pc        = bs_in_pkt.PC;
        bs_upd_taken = brcond_result;
        bs_upd_target    = brcond_result ? sum_addr : '0;

        // completion packet
        bs_out_pkt.dispatch_allocated_prs        = bs_in_pkt.dispatch_allocated_prs;
        bs_out_pkt.rob_entry      = bs_in_pkt.rob_entry;
        bs_out_pkt.halt           = bs_in_pkt.halt;
        bs_out_pkt.valid          = fu_complete;
        bs_out_pkt.if_take_branch = brcond_result;
        bs_out_pkt.cs_retire_pc      = brcond_result ? sum_addr : '0;
        // unconditional branch writes fall-through NPC
        bs_out_pkt.dest_value     =
        (bs_in_pkt.dec_fu_opcode.br == UNCOND) ? bs_in_pkt.NPC : '0;
    end

endmodule 

`endif // __BRANCH_FU_SV__


