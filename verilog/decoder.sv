
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module decoder(
    // Input: packet from IF stage containing instruction and valid bit
	input IF_ID_PACKET 		if_packet,
    // Outputs: control signals after decoding
	output ALU_OPA_SELECT 	opa_select,
	output ALU_OPB_SELECT 	opb_select,
	output logic [4:0]   	dest_reg,    // Destination architectural register number
	output logic [4:0]		reg1,        // Source register 1 (architectural)
	output logic [4:0]		reg2,        // Source register 2 (architectural)
	output FU_SELECT      	fu_sel,      // Which functional unit (ALU, BR, MULT, LS) to use
	output OP_SELECT	  	op_sel,      // Specific operation to perform inside FU
	output logic 			halt,        // Asserted if instruction is a HALT
	output logic 			valid_inst,  // Valid instruction flag (0 for HALT/illegal)
	output logic 			illegal,     // Asserted for illegal instructions
	output logic 			is_store     // Asserted for store instructions
);

// Internal wires
	INST inst; // unpacked instruction format
	logic valid_inst_in; // whether instruction is valid
	
	// Unpack fields from if_packet
	assign inst          = if_packet.inst;
	assign valid_inst_in = if_packet.valid;
	assign valid_inst    = valid_inst_in;
	
	always_comb begin
		// Default values for outputs (NOP behavior)
		opa_select = OPA_IS_RS1;         // Default: use RS1 as OPA
		opb_select = OPB_IS_RS2;         // Default: use RS2 as OPB
		dest_reg = `ZERO_REG;            // No destination register
		reg1 = `ZERO_REG;                // No source register 1
		reg2 = `ZERO_REG;                // No source register 2
		fu_sel = ALU_1;                  // Default to ALU_1
		op_sel = 0;                      // Default to no operation
		halt = `FALSE;                   // Not a halt
		illegal = `FALSE;                // Not illegal
		is_store = `FALSE;               // Not a store

		// Only decode if valid
		if(valid_inst_in) begin
			casez (inst) // Decode based on instruction opcode (wildcard match)
				// --------------- Immediate instructions ---------------
				`RV32_LUI: begin
					fu_sel = ALU_1; op_sel.alu = ALU_ADD;
					dest_reg = inst.r.rd;
					opa_select = OPA_IS_ZERO; opb_select = OPB_IS_U_IMM;
				end

				`RV32_AUIPC: begin
					fu_sel = ALU_1; op_sel.alu = ALU_ADD;
					dest_reg = inst.r.rd;
					opa_select = OPA_IS_PC; opb_select = OPB_IS_U_IMM;
				end

				// --------------- Control Transfer instructions ---------------
				`RV32_JAL: begin
					fu_sel = BRANCH; op_sel.br = UNCOND;
					dest_reg = inst.r.rd;
					opa_select = OPA_IS_PC; opb_select = OPB_IS_J_IMM;
				end

				`RV32_JALR: begin
					fu_sel = BRANCH; op_sel.br = UNCOND;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1; opb_select = OPB_IS_I_IMM;
				end

				// --------------- Conditional Branch instructions ---------------
				`RV32_BEQ: begin
					op_sel.br = BEQ;
					fu_sel = BRANCH;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select  = OPA_IS_PC;
					opb_select  = OPB_IS_B_IMM;
				end
				`RV32_BNE: begin
					op_sel.br = BNE;
					fu_sel = BRANCH;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select  = OPA_IS_PC;
					opb_select  = OPB_IS_B_IMM;
				end
				`RV32_BLT: begin
					op_sel.br = BLT;
					fu_sel = BRANCH;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select  = OPA_IS_PC;
					opb_select  = OPB_IS_B_IMM;
				end
				`RV32_BGE: begin
					op_sel.br = BGE;
					fu_sel = BRANCH;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select  = OPA_IS_PC;
					opb_select  = OPB_IS_B_IMM;
				end
				`RV32_BLTU: begin
					op_sel.br = BLTU;
					fu_sel = BRANCH;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select  = OPA_IS_PC;
					opb_select  = OPB_IS_B_IMM;
				end
				`RV32_BGEU: begin
					op_sel.br = BGEU;
					fu_sel = BRANCH;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select  = OPA_IS_PC;
					opb_select  = OPB_IS_B_IMM;
				end
				
				// --------------- Load instructions ---------------
				`RV32_LB: begin
					fu_sel = LS_1;
					op_sel.ls = LB;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end 
				`RV32_LH: begin
					fu_sel = LS_1;
					op_sel.ls = LH;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end 
				`RV32_LW: begin
					fu_sel = LS_1;
					op_sel.ls = LW;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_LBU: begin
					fu_sel = LS_1;
					op_sel.ls = LBU;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_LHU: begin
					fu_sel = LS_1;
					op_sel.ls = LHU;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end

				// --------------- Store instructions ---------------
				`RV32_SB: begin
					fu_sel = ALU_1;
					op_sel.alu = SB;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_S_IMM;
					is_store = `TRUE;
				end 
				`RV32_SH: begin
					fu_sel = ALU_1;
					op_sel.alu = SH;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_S_IMM;
					is_store = `TRUE;
				end
				`RV32_SW: begin
					fu_sel = ALU_1;
					op_sel.alu = SW;
					reg1 = inst.r.rs1;
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_S_IMM;
					is_store = `TRUE;
				end

				// --------------- ALU Immediate instructions ---------------
				`RV32_ADDI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_ADD;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_SLTI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SLT;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_SLTIU: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SLTU;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_ANDI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_AND;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_ORI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_OR;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_XORI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_XOR;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_SLLI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SLL;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_SRLI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SRL;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end
				`RV32_SRAI: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SRA;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_I_IMM;
				end

				// --------------- ALU Register instructions ---------------
				`RV32_ADD: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_ADD;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_SUB: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SUB;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_SLT: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SLT;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_SLTU: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SLTU;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_AND: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_AND;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_OR: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_OR;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_XOR: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_XOR;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_SLL: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SLL;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_SRL: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SRL;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_SRA: begin
					fu_sel = ALU_1;
					op_sel.alu = ALU_SRA;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end

				// --------------- Multiply instructions ---------------
				`RV32_MUL: begin
					fu_sel = MULT_1;
					op_sel.mult = MULT;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_MULH: begin
					fu_sel = MULT_1;
					op_sel.mult = MULH;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_MULHSU: begin
					fu_sel = MULT_1;
					op_sel.mult = MULHSU;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				`RV32_MULHU: begin
					fu_sel = MULT_1;
					op_sel.mult = MULHU;
					dest_reg = inst.r.rd;
					reg1 = inst.r.rs1; 
					reg2 = inst.r.rs2;
					opa_select = OPA_IS_RS1;
					opb_select = OPB_IS_RS2;
				end
				// --------------- Halt instruction ---------------
				`WFI: begin
					halt = `TRUE; // Mark halt
				end

				// --------------- Default: Illegal instruction ---------------
				default: begin
					illegal = `TRUE; // Unrecognized opcode
				end
			endcase
		end
	end
endmodule

