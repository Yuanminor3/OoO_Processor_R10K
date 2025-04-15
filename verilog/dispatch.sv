`define TEST_MODE
`ifndef __DISPATCH_V__
`define __DISPATCH_V__

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

module dispatch_stage (

    //================================
    //        Input Packets
    //================================
    input IF_ID_PACKET [2:0] if_id_packet_in,

    //================================
    //        Stall Signals
    //================================
    input [2:0] rs_stall,             // RS structural hazard (reservation station full)
    input [2:0] rob_stall,            // ROB structural hazard (reorder buffer full)
    input [2:0] free_reg_valid,       // Free physical register valid
    input [2:0] sq_stall,             // Store queue stall

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

    /* Dispatch Stall */
    output logic [2:0] d_stall,          // Stall signals per instruction

    /* Branch Predictor Info */
    output logic [2:0] bp_EN,            // Enable branch predictor update
    output logic [2:0][`XLEN-1:0] bp_pc  // PC to update branch predictor

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

// Calculate structural hazard stalls
assign d_stall = rs_stall | rob_stall | ~free_reg_valid | sq_stall;

// Invalidate packets based on branch prediction and stalls
always_comb begin
    dis_packet = if_id_packet_in;
    dis_packet[2].valid = if_id_packet_in[2].valid & ~d_stall[2];
    dis_packet[1].valid = (if_id_packet_in[2].predict_direction) ? 0 : if_id_packet_in[1].valid & ~d_stall[1];
    dis_packet[0].valid = (if_id_packet_in[2].predict_direction | if_id_packet_in[1].predict_direction) ? 0 : if_id_packet_in[0].valid & ~d_stall[0];
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

// Allocate new physical registers if dest_reg != x0
always_comb begin
    for (int i = 0; i < 3; i++) begin
        new_pr_en[i] = dis_packet[i].valid && (dest_arch[i] != `ZERO_REG);
        dest_pr[i]   = (new_pr_en[i]) ? free_pr_in[i] : `ZERO_REG;
    end
end

// Update map table
always_comb begin
    for (int i = 0; i < 3; i++) begin
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

    // Handle branch prediction info specially for each instruction slot
    rob_in[2].predict_direction = dis_packet[2].predict_direction;
    rob_in[2].predict_pc        = (dis_packet[2].predict_direction) ? dis_packet[2].predict_pc : 0;
    rob_in[1].predict_direction = (~dis_packet[2].predict_direction) ? dis_packet[1].predict_direction : 0;
    rob_in[1].predict_pc        = (~dis_packet[2].predict_direction && dis_packet[1].predict_direction) ? dis_packet[1].predict_pc : 0;
    rob_in[0].predict_direction = (~dis_packet[2].predict_direction && ~dis_packet[1].predict_direction) ? dis_packet[0].predict_direction : 0;
    rob_in[0].predict_pc        = (~dis_packet[2].predict_direction && ~dis_packet[1].predict_direction && dis_packet[0].predict_direction) ? dis_packet[0].predict_pc : 0;
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

// Send branch prediction information to BP unit
always_comb begin
    bp_EN = 0;
    bp_pc = 0;
    for (int i = 0; i < 3; i++) begin
        if (dis_packet[i].valid && fu_sel[i] == BRANCH) begin
            bp_EN[i] = 1;
            bp_pc[i] = dis_packet[i].PC;
        end
    end
end

endmodule
`endif // _DISPATCH_V_
