//////////////////////////////////////////////////////////////////////////
//                                                                      //
//   Modulename :  alu.sv                                               // 
//                                                                      //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
`ifndef __ALU_V__
`define __ALU_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module alu(
	input [`SYS_XLEN-1:0]   opa,
	input [`SYS_XLEN-1:0]   opb,
	ALU_SELECT          func,

	output logic [`SYS_XLEN-1:0] result
);
	wire signed [`SYS_XLEN-1:0] alu_operandA_signed, alu_operandB_signed;
	assign alu_operandA_signed = opa;
	assign alu_operandB_signed = opb;

	always_comb begin
		case (func)
			ALU_ADD:      result = opa + opb;
			ALU_SUB:      result = opa - opb;
			ALU_AND:      result = opa & opb;
			ALU_SLT:      result = alu_operandA_signed < alu_operandB_signed;
			ALU_SLTU:     result = opa < opb;
			ALU_OR:       result = opa | opb;
			ALU_XOR:      result = opa ^ opb;
			ALU_SRL:      result = opa >> opb[4:0];
			ALU_SLL:      result = opa << opb[4:0];
			ALU_SRA:      result = alu_operandA_signed >>> opb[4:0]; // arithmetic from logical icache_shift
			default:      result = `SYS_XLEN'hfacebeec;  // here to prevent latches
		endcase
	end
endmodule // alu

`timescale 1ns/100ps

module fu_alu (
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  bs_hazard,
    input  ISSUE_FU_PACKET        bs_in_pkt,

    output logic                  rsb_fu_ready,
    output logic                  fum_complete_req,
    output FU_COMPLETE_PACKET     bs_out_pkt,

    output logic                  alu_store_enable,
    output SQ_ENTRY_PACKET        alu_store_output_entry,
    output logic [`SYS_LSQ_ADDR_WIDTH-1:0]       alu_store_sq_index
);

  // Operand MUX
  logic [`SYS_XLEN-1:0] opa, opb;
  ALU_SELECT alu_operation_code;
  logic [`SYS_XLEN-1:0] alu_exec_result;

  // Store detection
  assign alu_store_enable = bs_in_pkt.valid && (bs_in_pkt.dec_fu_opcode.alu inside {SB, SH, SW});
  assign alu_store_sq_index   = bs_in_pkt.sq_tail;
  assign alu_operation_code = alu_store_enable ? ALU_ADD : bs_in_pkt.dec_fu_opcode.alu;

  // opa selection
  always_comb begin
    unique case (bs_in_pkt.dec_operandA_mux)
      OPA_IS_RS1:   opa = bs_in_pkt.r1_value;
      OPA_IS_NPC:   opa = bs_in_pkt.NPC;
      OPA_IS_PC:    opa = bs_in_pkt.PC;
      OPA_IS_ZERO:  opa = '0;
      default:      opa = `SYS_XLEN'hDEADFACE;
    endcase
  end

  // opb selection
  always_comb begin
    unique case (bs_in_pkt.dec_operandB_mux)
      OPB_IS_RS2:   opb = bs_in_pkt.r2_value;
      OPB_IS_I_IMM: opb = `RV32_signext_Iimm(bs_in_pkt.inst);
      OPB_IS_S_IMM: opb = `RV32_signext_Simm(bs_in_pkt.inst);
      OPB_IS_B_IMM: opb = `RV32_signext_Bimm(bs_in_pkt.inst);
      OPB_IS_U_IMM: opb = `RV32_signext_Uimm(bs_in_pkt.inst);
      OPB_IS_J_IMM: opb = `RV32_signext_Jimm(bs_in_pkt.inst);
      default:      opb = `SYS_XLEN'hF00DBEEF;
    endcase
  end

  // ALU logic
  alu alu_unit (
    .opa   (opa),
    .opb   (opb),
    .func  (alu_operation_code),
    .result(alu_exec_result)
  );

  // Complete packet result
  FU_COMPLETE_PACKET result;
  always_comb begin
    result.valid         = bs_in_pkt.valid;
    result.halt          = bs_in_pkt.halt;
    result.if_take_branch= 1'b0;
    result.cs_retire_pc     = '0;
    result.dispatch_allocated_prs       = bs_in_pkt.dispatch_allocated_prs;
    result.rob_entry     = bs_in_pkt.rob_entry;
    result.dest_value    = alu_exec_result;
  end

  assign fum_complete_req = bs_in_pkt.valid;
  assign rsb_fu_ready          = ~bs_hazard;
  assign bs_out_pkt     = result;

  // Store Packet
  logic [`SYS_XLEN-1:0] alu_value_store;
  assign alu_value_store = bs_in_pkt.r2_value;

  always_comb begin
    alu_store_output_entry.addr  = {alu_exec_result[`SYS_XLEN-1:2], 2'b00};
    alu_store_output_entry.ready = 1'b1;

    case (bs_in_pkt.dec_fu_opcode.alu)
      SB: begin
        alu_store_output_entry.usebytes = 4'b0001 << alu_exec_result[1:0];
        alu_store_output_entry.data     = {24'b0, alu_value_store[7:0]} << (8 * alu_exec_result[1:0]);
      end
      SH: begin
        alu_store_output_entry.usebytes = (alu_exec_result[1] == 1'b0) ? 4'b0011 : 4'b1100;
        alu_store_output_entry.data     = {16'b0, alu_value_store[15:0]} << (16 * alu_exec_result[1]);
      end
      SW: begin
        alu_store_output_entry.usebytes = 4'b1111;
        alu_store_output_entry.data     = alu_value_store;
      end
      default: begin
        alu_store_output_entry.usebytes = 4'b0000;
        alu_store_output_entry.data     = '0;
      end
    endcase
  end

endmodule
`endif // __ALU_V__

