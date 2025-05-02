
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module decoder(
    // Input: packet from IF stage containing instruction and valid bit
	input IF_ID_PACKET 		decode_pkt,
    // Outputs: control signals after decoding
	output ALU_OPA_SELECT 	dec_operandA_mux,
	output ALU_OPB_SELECT 	dec_operandB_mux,
	output logic [4:0]   	dec_dst_arch_reg,    // Destination architectural register number
	output logic [4:0]		dec_src1_reg,        // Source register 1 (architectural)
	output logic [4:0]		dec_src2_reg,        // Source register 2 (architectural)
	output FU_SELECT      	dec_fu_unit_sel,      // Which functional unit (ALU, BR, MULT, LS) to use
	output OP_SELECT	  	dec_fu_opcode,      // Specific operation to perform inside SYS_FU_ADDR_WIDTH
	output logic 			halt,        // Asserted if instruction is a HALT
	output logic 			dec_valid_flag,  // Valid instruction flag (0 for HALT/dec_illegal_flag)
	output logic 			dec_illegal_flag,     // Asserted for dec_illegal_flag instructions
	output logic 			dec_is_store_flag     // Asserted for store instructions
);

// Internal wires
	INST inst; // unpacked instruction format
	logic valid_inst_in; // whether instruction is valid
	
	// Unpack fields from decode_pkt
	assign inst          = decode_pkt.inst;
	assign valid_inst_in = decode_pkt.valid;
	assign dec_valid_flag    = valid_inst_in;
	
	always_comb begin
		// Default values for outputs (SYS_INST_NOP behavior)
		dec_operandA_mux = OPA_IS_RS1;         // Default: use RS1 as OPA
		dec_operandB_mux = OPB_IS_RS2;         // Default: use RS2 as OPB
		dec_dst_arch_reg = `SYS_ZERO_ARCH_REG;            // No destination register
		dec_src1_reg = `SYS_ZERO_ARCH_REG;                // No source register 1
		dec_src2_reg = `SYS_ZERO_ARCH_REG;                // No source register 2
		dec_fu_unit_sel = ALU_1;                  // Default to ALU_1
		dec_fu_opcode = 0;                      // Default to no operation
		halt = `FALSE;                   // Not a halt
		dec_illegal_flag = `FALSE;                // Not dec_illegal_flag
		dec_is_store_flag = `FALSE;               // Not a store

		// Only decode if valid
		if(valid_inst_in) begin
			casez (inst) // Decode based on instruction opcode (wildcard match)
				// --------------- Immediate instructions ---------------
				`RV32_LUI: begin
					dec_fu_unit_sel = ALU_1; dec_fu_opcode.alu = ALU_ADD;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_operandA_mux = OPA_IS_ZERO; dec_operandB_mux = OPB_IS_U_IMM;
				end

				`RV32_AUIPC: begin
					dec_fu_unit_sel = ALU_1; dec_fu_opcode.alu = ALU_ADD;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_operandA_mux = OPA_IS_PC; dec_operandB_mux = OPB_IS_U_IMM;
				end

				// --------------- Control Transfer instructions ---------------
				`RV32_JAL: begin
					dec_fu_unit_sel = BRANCH; dec_fu_opcode.br = UNCOND;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_operandA_mux = OPA_IS_PC; dec_operandB_mux = OPB_IS_J_IMM;
				end

				`RV32_JALR: begin
					dec_fu_unit_sel = BRANCH; dec_fu_opcode.br = UNCOND;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1; dec_operandB_mux = OPB_IS_I_IMM;
				end

				// --------------- Conditional Branch instructions ---------------
				`RV32_BEQ: begin
					dec_fu_opcode.br = BEQ;
					dec_fu_unit_sel = BRANCH;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux  = OPA_IS_PC;
					dec_operandB_mux  = OPB_IS_B_IMM;
				end
				`RV32_BNE: begin
					dec_fu_opcode.br = BNE;
					dec_fu_unit_sel = BRANCH;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux  = OPA_IS_PC;
					dec_operandB_mux  = OPB_IS_B_IMM;
				end
				`RV32_BLT: begin
					dec_fu_opcode.br = BLT;
					dec_fu_unit_sel = BRANCH;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux  = OPA_IS_PC;
					dec_operandB_mux  = OPB_IS_B_IMM;
				end
				`RV32_BGE: begin
					dec_fu_opcode.br = BGE;
					dec_fu_unit_sel = BRANCH;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux  = OPA_IS_PC;
					dec_operandB_mux  = OPB_IS_B_IMM;
				end
				`RV32_BLTU: begin
					dec_fu_opcode.br = BLTU;
					dec_fu_unit_sel = BRANCH;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux  = OPA_IS_PC;
					dec_operandB_mux  = OPB_IS_B_IMM;
				end
				`RV32_BGEU: begin
					dec_fu_opcode.br = BGEU;
					dec_fu_unit_sel = BRANCH;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux  = OPA_IS_PC;
					dec_operandB_mux  = OPB_IS_B_IMM;
				end
				
				// --------------- Load instructions ---------------
				`RV32_LB: begin
					dec_fu_unit_sel = LS_1;
					dec_fu_opcode.ls = LB;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end 
				`RV32_LH: begin
					dec_fu_unit_sel = LS_1;
					dec_fu_opcode.ls = LH;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end 
				`RV32_LW: begin
					dec_fu_unit_sel = LS_1;
					dec_fu_opcode.ls = LW;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_LBU: begin
					dec_fu_unit_sel = LS_1;
					dec_fu_opcode.ls = LBU;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_LHU: begin
					dec_fu_unit_sel = LS_1;
					dec_fu_opcode.ls = LHU;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end

				// --------------- Store instructions ---------------
				`RV32_SB: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = SB;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_S_IMM;
					dec_is_store_flag = `TRUE;
				end 
				`RV32_SH: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = SH;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_S_IMM;
					dec_is_store_flag = `TRUE;
				end
				`RV32_SW: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = SW;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_S_IMM;
					dec_is_store_flag = `TRUE;
				end

				// --------------- ALU Immediate instructions ---------------
				`RV32_ADDI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_ADD;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_SLTI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SLT;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_SLTIU: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SLTU;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_ANDI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_AND;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_ORI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_OR;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_XORI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_XOR;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_SLLI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SLL;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_SRLI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SRL;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end
				`RV32_SRAI: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SRA;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_I_IMM;
				end

				// --------------- ALU Register instructions ---------------
				`RV32_ADD: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_ADD;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_SUB: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SUB;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_SLT: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SLT;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_SLTU: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SLTU;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_AND: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_AND;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_OR: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_OR;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_XOR: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_XOR;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_SLL: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SLL;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_SRL: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SRL;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_SRA: begin
					dec_fu_unit_sel = ALU_1;
					dec_fu_opcode.alu = ALU_SRA;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end

				// --------------- Multiply instructions ---------------
				`RV32_MUL: begin
					dec_fu_unit_sel = MULT_1;
					dec_fu_opcode.mult = MULT;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_MULH: begin
					dec_fu_unit_sel = MULT_1;
					dec_fu_opcode.mult = MULH;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_MULHSU: begin
					dec_fu_unit_sel = MULT_1;
					dec_fu_opcode.mult = MULHSU;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				`RV32_MULHU: begin
					dec_fu_unit_sel = MULT_1;
					dec_fu_opcode.mult = MULHU;
					dec_dst_arch_reg = inst.cmp_type.rd;
					dec_src1_reg = inst.cmp_type.cmp_opA; 
					dec_src2_reg = inst.cmp_type.cmp_opB;
					dec_operandA_mux = OPA_IS_RS1;
					dec_operandB_mux = OPB_IS_RS2;
				end
				// --------------- Halt instruction ---------------
				`WFI: begin
					halt = `TRUE; // Mark halt
				end

				// --------------- Default: Illegal instruction ---------------
				default: begin
					dec_illegal_flag = `TRUE; // Unrecognized opcode
				end
			endcase
		end
	end
endmodule

