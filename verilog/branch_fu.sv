//////////////////////////////////////////////////////////////////////////
//                                                                      //
//   Modulename :  branch_fu.sv                                         //
//                                                                      //
//  Description :  Combinational branch functional unit                 //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
`ifndef __BRANCH_FU_SV__
`define __BRANCH_FU_SV__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

// brcond: compute branch-taken flag using parallel comparisons
module brcond(
    input  logic [`XLEN-1:0] rs1,    // first operand
    input  logic [`XLEN-1:0] rs2,    // second operand
    input  BR_SELECT        br,      // branch type selector
    output logic            cond     // result: 1 => branch taken
);
    // preserve signed views for signed comparisons
    logic signed [`XLEN-1:0] signed_rs1;
    logic signed [`XLEN-1:0] signed_rs2;
    assign signed_rs1 = rs1;
    assign signed_rs2 = rs2;

    // parallel comparison signals
    logic eq, ne, lt, ge, ltu, geu;
    assign eq  = (signed_rs1 == signed_rs2);
    assign ne  = (signed_rs1 != signed_rs2);
    assign lt  = (signed_rs1 <  signed_rs2);
    assign ge  = (signed_rs1 >= signed_rs2);
    assign ltu = (rs1 < rs2);
    assign geu = (rs1 >= rs2);

    // combine comparisons based on branch type
    always_comb begin
        cond = 1'b0;
        cond |= (br == UNCOND)  ? 1'b1 : 1'b0;
        cond |= (br == BEQ)     ? eq  : 1'b0;
        cond |= (br == BNE)     ? ne  : 1'b0;
        cond |= (br == BLT)     ? lt  : 1'b0;
        cond |= (br == BGE)     ? ge  : 1'b0;
        cond |= (br == BLTU)    ? ltu : 1'b0;
        cond |= (br == BGEU)    ? geu : 1'b0;
    end
endmodule 

// branch_stage: retain same two-level MUX + brcond + adder structure
module branch_stage(
    input                  clock,
    input                  reset,
    input                  complete_stall,       // structural hazard
    input  ISSUE_FU_PACKET fu_packet_in,         // FU input packet
    output logic           fu_ready,             // always ready
    output logic           want_to_complete_branch,
    output FU_COMPLETE_PACKET fu_packet_out,     // to retire
    output logic           update_EN,            // predictor update enable
    output logic [`XLEN-1:0] update_pc,
    output logic           update_direction,
    output logic [`XLEN-1:0] update_target
);

    // flag when FU result is valid
    wire fu_complete = fu_packet_in.valid;
    assign want_to_complete_branch = fu_complete;
    assign fu_ready               = 1'b1;

    // operand-A multiplexer (1st MUX)
    logic [`XLEN-1:0] opa_mux_out;
    always_comb begin
        opa_mux_out = `XLEN'h0;
        case (fu_packet_in.opa_select)
            OPA_IS_RS1:  opa_mux_out = fu_packet_in.r1_value;
            OPA_IS_NPC:  opa_mux_out = fu_packet_in.NPC;
            OPA_IS_PC:   opa_mux_out = fu_packet_in.PC;
            OPA_IS_ZERO: opa_mux_out = '0;
        endcase
    end

    // operand-B multiplexer (2nd MUX)
    logic [`XLEN-1:0] opb_mux_out;
    always_comb begin
        opb_mux_out = `XLEN'h0;
        case (fu_packet_in.opb_select)
            OPB_IS_RS2:   opb_mux_out = fu_packet_in.r2_value;
            OPB_IS_I_IMM: opb_mux_out = `RV32_signext_Iimm(fu_packet_in.inst);
            OPB_IS_B_IMM: opb_mux_out = `RV32_signext_Bimm(fu_packet_in.inst);
            OPB_IS_J_IMM: opb_mux_out = `RV32_signext_Jimm(fu_packet_in.inst);
        endcase
    end

    // branch condition checker instance
    logic brcond_result;
    brcond brcond_inst (
        .rs1  (fu_packet_in.r1_value),
        .rs2  (fu_packet_in.r2_value),
        .br   (fu_packet_in.op_sel.br),
        .cond (brcond_result)
    );

    // adder result for branch target
    logic [`XLEN-1:0] sum_addr;
    always_comb begin
        sum_addr = opa_mux_out + opb_mux_out;
    end

    // final outputs and predictor updates
    always_comb begin
        // predictor interface
        update_EN        = fu_complete;
        update_pc        = fu_packet_in.PC;
        update_direction = brcond_result;
        update_target    = brcond_result ? sum_addr : '0;

        // completion packet
        fu_packet_out.dest_pr        = fu_packet_in.dest_pr;
        fu_packet_out.rob_entry      = fu_packet_in.rob_entry;
        fu_packet_out.halt           = fu_packet_in.halt;
        fu_packet_out.valid          = fu_complete;
        fu_packet_out.if_take_branch = brcond_result;
        fu_packet_out.target_pc      = brcond_result ? sum_addr : '0;
        // unconditional branch writes fall-through NPC
        fu_packet_out.dest_value     =
        (fu_packet_in.op_sel.br == UNCOND) ? fu_packet_in.NPC : '0;
    end

endmodule 

`endif // __BRANCH_FU_SV__


