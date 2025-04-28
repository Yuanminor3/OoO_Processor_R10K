//////////////////////////////////////////////////////////////////////////
//                                                                      //
//   Modulename :  alu.sv                                               //
//                                                                      //
//  Description :  instruction execute (EX) stage of the pipeline;      //
//                 given the instruction command code CMD, select the   //
//                 proper input A and B for the ALU, compute the result,// 
//                 and compute the condition for branches, and pass all //
//                 the results down the pipeline. MWB                   // 
//                                                                      //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
`ifndef __ALU_V__
`define __ALU_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module alu(
	input [`XLEN-1:0]   opa,
	input [`XLEN-1:0]   opb,
	ALU_SELECT          func,

	output logic [`XLEN-1:0] result
);
	wire signed [`XLEN-1:0] signed_opa, signed_opb;
	assign signed_opa = opa;
	assign signed_opb = opb;

	always_comb begin
		case (func)
			ALU_ADD:      result = opa + opb;
			ALU_SUB:      result = opa - opb;
			ALU_AND:      result = opa & opb;
			ALU_SLT:      result = signed_opa < signed_opb;
			ALU_SLTU:     result = opa < opb;
			ALU_OR:       result = opa | opb;
			ALU_XOR:      result = opa ^ opb;
			ALU_SRL:      result = opa >> opb[4:0];
			ALU_SLL:      result = opa << opb[4:0];
			ALU_SRA:      result = signed_opa >>> opb[4:0]; // arithmetic from logical shift
			default:      result = `XLEN'hfacebeec;  // here to prevent latches
		endcase
	end
endmodule // alu

`timescale 1ns/100ps

module fu_alu (
    input  logic                  clock,
    input  logic                  reset,
    input  logic                  complete_stall,
    input  ISSUE_FU_PACKET        fu_packet_in,

    output logic                  fu_ready,
    output logic                  want_to_complete,
    output FU_COMPLETE_PACKET     fu_packet_out,

    output logic                  if_store,
    output SQ_ENTRY_PACKET        store_pckt,
    output logic [`LSQ-1:0]       sq_idx
);

  // Operand MUX
  logic [`XLEN-1:0] opa, opb;
  ALU_SELECT alu_func;
  logic [`XLEN-1:0] alu_result;

  // Store detection
  assign if_store = fu_packet_in.valid && (fu_packet_in.op_sel.alu inside {SB, SH, SW});
  assign sq_idx   = fu_packet_in.sq_tail;
  assign alu_func = if_store ? ALU_ADD : fu_packet_in.op_sel.alu;

  // opa selection
  always_comb begin
    unique case (fu_packet_in.opa_select)
      OPA_IS_RS1:   opa = fu_packet_in.r1_value;
      OPA_IS_NPC:   opa = fu_packet_in.NPC;
      OPA_IS_PC:    opa = fu_packet_in.PC;
      OPA_IS_ZERO:  opa = '0;
      default:      opa = `XLEN'hDEADFACE;
    endcase
  end

  // opb selection
  always_comb begin
    unique case (fu_packet_in.opb_select)
      OPB_IS_RS2:   opb = fu_packet_in.r2_value;
      OPB_IS_I_IMM: opb = `RV32_signext_Iimm(fu_packet_in.inst);
      OPB_IS_S_IMM: opb = `RV32_signext_Simm(fu_packet_in.inst);
      OPB_IS_B_IMM: opb = `RV32_signext_Bimm(fu_packet_in.inst);
      OPB_IS_U_IMM: opb = `RV32_signext_Uimm(fu_packet_in.inst);
      OPB_IS_J_IMM: opb = `RV32_signext_Jimm(fu_packet_in.inst);
      default:      opb = `XLEN'hF00DBEEF;
    endcase
  end

  // ALU logic
  alu alu_unit (
    .opa   (opa),
    .opb   (opb),
    .func  (alu_func),
    .result(alu_result)
  );

  // Complete packet result
  FU_COMPLETE_PACKET result;
  always_comb begin
    result.valid         = fu_packet_in.valid;
    result.halt          = fu_packet_in.halt;
    result.if_take_branch= 1'b0;
    result.target_pc     = '0;
    result.dest_pr       = fu_packet_in.dest_pr;
    result.rob_entry     = fu_packet_in.rob_entry;
    result.dest_value    = alu_result;
  end

  assign want_to_complete = fu_packet_in.valid;
  assign fu_ready          = ~complete_stall;
  assign fu_packet_out     = result;

  // Store Packet
  logic [`XLEN-1:0] store_val;
  assign store_val = fu_packet_in.r2_value;

  always_comb begin
    store_pckt.addr  = {alu_result[`XLEN-1:2], 2'b00};
    store_pckt.ready = 1'b1;

    case (fu_packet_in.op_sel.alu)
      SB: begin
        store_pckt.usebytes = 4'b0001 << alu_result[1:0];
        store_pckt.data     = {24'b0, store_val[7:0]} << (8 * alu_result[1:0]);
      end
      SH: begin
        store_pckt.usebytes = (alu_result[1] == 1'b0) ? 4'b0011 : 4'b1100;
        store_pckt.data     = {16'b0, store_val[15:0]} << (16 * alu_result[1]);
      end
      SW: begin
        store_pckt.usebytes = 4'b1111;
        store_pckt.data     = store_val;
      end
      default: begin
        store_pckt.usebytes = 4'b0000;
        store_pckt.data     = '0;
      end
    endcase
  end

endmodule
`endif // __ALU_V__

