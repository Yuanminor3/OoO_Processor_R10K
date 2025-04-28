/////////////////////////////////////////////////////////////////////////
//                                                                     //
//                   Modulename :  sys_defs.vh                         //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`ifndef __SYS_DEFS_VH__
`define __SYS_DEFS_VH__
`timescale 1ns/100ps

//////////////////////////////////////////////
//					    //
//          Parameter definitions  	    //
//					    //
//////////////////////////////////////////////
// PR and Value
`define XLEN 32		// 32 value bits 
`define PR 6  		// 2^6 = 64 Physical registers (32AR+32ROB)
`define ZERO_PR 0  	// PR[0] = AR[0], always read 0 and don't write.
`define ZERO_REG 5'd0
// ROB
`define ROB 5     	
`define ROBW 32		// 32 rob entries
// FU
`define FU 3 		// 2^3 = 8 fu in total
`define OP 4
`define MUL_STAGE 4
// RS
`define RSW 16 		// 16 RS entry
// LSQ
`define LSQ 3		// 2^3 = 8 sq entry
// MHSRS
`define MHSRS 3
`define MHSRS_W 8  	// = 4**`MHSRS
// IS
`define IS_FIFO_DEPTH 32
// PREFETCH
`define PREF 12   	// prefetch 16 lines ahead
// BP
`define BP 5
`define BPW 32
// SMALL DELAY
`define SD #1
// Boolean
`define FALSE  1'h0
`define TRUE  1'h1
// Basic NOP instruction
`define NOP 32'h00000013

//////////////////////////////////////////////
//					    //
// 		RISCV ISA        	    //
//					    //
//////////////////////////////////////////////
typedef union packed {
	logic [31:0] inst;
	struct packed {
		logic [6:0] funct7;
		logic [4:0] rs2;
		logic [4:0] rs1;
		logic [2:0] funct3;
		logic [4:0] rd;
		logic [6:0] opcode;
	} r; //register to register instructions
	struct packed {
		logic [11:0] imm;
		logic [4:0]  rs1; 
		logic [2:0]  funct3;
		logic [4:0]  rd;  
		logic [6:0]  opcode;
	} i; //immediate or load instructions
	struct packed {
		logic [6:0] off; 
		logic [4:0] rs2; 
		logic [4:0] rs1; 
		logic [2:0] funct3;
		logic [4:0] set; 
		logic [6:0] opcode;
	} s; //store instructions
	struct packed {
		logic       of; 
		logic [5:0] s;   
		logic [4:0] rs2;
		logic [4:0] rs1;
		logic [2:0] funct3;
		logic [3:0] et; 
		logic       f;  
		logic [6:0] opcode;
	} b; //branch instructions
	struct packed {
		logic [19:0] imm;
		logic [4:0]  rd;
		logic [6:0]  opcode;
	} u; //upper immediate instructions
	struct packed {
		logic       of; 
		logic [9:0] et; 
		logic       s;  
		logic [7:0] f;	
		logic [4:0] rd; 
		logic [6:0] opcode;
	} j;  //jump instructions
`ifdef ATOMIC_EXT
	struct packed {
		logic [4:0] funct5;
		logic       aq;
		logic       rl;
		logic [4:0] rs2;
		logic [4:0] rs1;
		logic [2:0] funct3;
		logic [4:0] rd;
		logic [6:0] opcode;
	} a; //atomic instructions
`endif
`ifdef SYSTEM_EXT
	struct packed {
		logic [11:0] csr;
		logic [4:0]  rs1;
		logic [2:0]  funct3;
		logic [4:0]  rd;
		logic [6:0]  opcode;
	} sys; //system call instructions
`endif
} INST; 

//////////////////////////////////////////////
//					    //
// 		    FU_Packet		    //
//					    //
//////////////////////////////////////////////

typedef struct packed{
	logic branch;
	logic mult_2;
	logic mult_1;
	logic loadstore_2;
	logic loadstore_1;
	logic alu_3;
	logic alu_2;
	logic alu_1; 
}FU_STATE_PACKET;

typedef enum logic [`FU-1:0] {
	ALU_1 = 0,
	ALU_2 = 1, 
	ALU_3 = 2,
	LS_1 = 3,
	LS_2 = 4,
	MULT_1 = 5,
	MULT_2 = 6,
	BRANCH = 7
} FU_SELECT;

typedef struct packed{
	logic alu;
	logic ls;
	logic mult;
	logic branch;
} FU_FIFO_PACKET;

/* OP select for different fu */
typedef enum logic[`OP-1:0] {
	ALU_ADD = 0,
	ALU_SUB,
	ALU_SLT,
	ALU_SLTU,
	ALU_AND,
	ALU_OR,
	ALU_XOR,
	ALU_SLL,
	ALU_SRL,
	ALU_SRA,
	SB, 
	SH,
        SW
} ALU_SELECT;

typedef enum logic[`OP-1:0]{
	MULT,
	MULH,
	MULHSU,
	MULHU
} MULT_SELECT;

typedef enum logic [`OP-1:0]{ 
	UNCOND,
	BEQ,
	BNE,
	BLT,
	BGE,
	BLTU,
	BGEU
} BR_SELECT;

typedef enum logic[`OP-1:0]{
    LB,
    LH,
    LW,
    LBU,
    LHU
} LS_SELECT;

typedef union packed{
	ALU_SELECT alu;
	MULT_SELECT mult;
	LS_SELECT ls;
	BR_SELECT br;
}OP_SELECT;

typedef enum logic[1:0]{
		EMPTY = 0,
		INUSED = 1,
		COMPLETE = 2
}ROB_STATE;

typedef enum logic[1:0]{
		STRONG_NT = 0,
		WEAK_NT = 1,
		WEAK_T = 2,
		STRONG_T = 3
} BP_STATE;

// ALU opA input mux selects
typedef enum logic [1:0] {
	OPA_IS_RS1  = 2'h0,
	OPA_IS_NPC  = 2'h1,
	OPA_IS_PC   = 2'h2,
	OPA_IS_ZERO = 2'h3
} ALU_OPA_SELECT;

// ALU opB input mux selects
typedef enum logic [3:0] {
	OPB_IS_RS2    = 4'h0,
	OPB_IS_I_IMM  = 4'h1,
	OPB_IS_S_IMM  = 4'h2,
	OPB_IS_B_IMM  = 4'h3,
	OPB_IS_U_IMM  = 4'h4,
	OPB_IS_J_IMM  = 4'h5
} ALU_OPB_SELECT;

// Destination register select
typedef enum logic [1:0] {
	DEST_RD = 2'h0,
	DEST_NONE  = 2'h1
} DEST_REG_SEL;

//////////////////////////////////////////////
//					    //
//        Pipeline Stage Packets	    //
//					    //
//////////////////////////////////////////////

// IF2ID
typedef struct packed {
	logic valid; 		// If low, the data in this struct is garbage
        INST  inst;  		// fetched instruction out
	logic [`XLEN-1:0] NPC;  // PC + 4
	logic [`XLEN-1:0] PC;   // PC
	logic                predict_direction;
        logic [`XLEN-1:0]    predict_pc;
} IF_ID_PACKET;

// ID2RS
typedef struct packed {
    logic               valid; 
    FU_SELECT           fu_sel;
    OP_SELECT           op_sel;
    logic [`XLEN-1:0]   NPC;   // PC + 4
    logic [`XLEN-1:0]   PC;    // PC
    ALU_OPA_SELECT      opa_select; // ALU opa mux select 
    ALU_OPB_SELECT      opb_select; // ALU opb mux select 
    INST          	inst;
    logic               halt;         
    logic [`ROB-1:0] 	rob_entry;
    logic [`LSQ-1:0]	sq_tail; 
    logic [`PR-1:0]     dest_pr;
    logic [`PR-1:0]     reg1_pr;
    logic               reg1_ready;
    logic [`PR-1:0]     reg2_pr;
    logic               reg2_ready;
} RS_IN_PACKET;

// RS2IS
typedef struct packed {
    logic               valid;
    FU_SELECT           fu_sel;
    OP_SELECT           op_sel;
    logic [`XLEN-1:0]   NPC;   // PC + 4
    logic [`XLEN-1:0]   PC;    // PC
    ALU_OPA_SELECT      opa_select; // ALU opa mux select 
    ALU_OPB_SELECT      opb_select; // ALU opb mux select
    INST          	inst;
    logic               halt;          
    logic [`ROB-1:0] 	rob_entry;
    logic [`LSQ-1:0]	sq_tail;
    logic [`PR-1:0]     dest_pr;
    logic [`PR-1:0]     reg1_pr;
    logic [`PR-1:0]     reg2_pr;
} RS_S_PACKET;

// IS2FU
typedef struct packed{
	logic 			valid;
	OP_SELECT		op_sel;
	logic [`XLEN-1:0]   	NPC;   // PC + 4
        logic [`XLEN-1:0]       PC;    // PC
	ALU_OPA_SELECT          opa_select; // ALU opa mux select 
        ALU_OPB_SELECT          opb_select; // ALU opb mux select 
	INST          		inst;
	logic 			halt;
	logic [`ROB-1:0] 	rob_entry;
	logic [`LSQ-1:0]	sq_tail;
	logic [`PR-1:0] 	dest_pr;
	logic [`XLEN-1:0]	r1_value;
	logic [`XLEN-1:0] 	r2_value;
} ISSUE_FU_PACKET;

// FU2COMPLETE
typedef struct packed{
	logic if_take_branch;
	logic valid;
	logic halt; 
	logic [`XLEN-1:0] target_pc;
	logic [`PR-1:0]   dest_pr;
	logic [`XLEN-1:0] dest_value;
	logic [`ROB-1:0]  rob_entry;
} FU_COMPLETE_PACKET;

// EX2MEM
typedef struct packed {
	logic [`XLEN-1:0] alu_result; 	// alu_result
	logic [`XLEN-1:0] NPC; 		//pc + 4
	logic             take_branch;  // is this a taken branch?
	//pass throughs from decode stage
	logic [`XLEN-1:0] rs2_value;
	logic             rd_mem, wr_mem;
	logic [4:0]       dest_reg_idx;
	logic             halt, illegal, csr_op, valid;
	logic [2:0]       mem_size; 	// byte, half-word or word
} EX_MEM_PACKET;

//////////////////////////////////////////////
//					    //
//        Individual Module Packets	    //
//					    //
//////////////////////////////////////////////

// OTHER MODULE
typedef struct packed{
    logic [`PR-1:0] t0;
    logic [`PR-1:0] t1;
    logic [`PR-1:0] t2;
 }CDB_T_PACKET;

typedef struct packed {
	logic [`XLEN-1:0]   NPC;   // PC + 4
        logic [`XLEN-1:0]   PC;    // PC
	logic 		    valid;
	logic [`PR-1:0]     Tnew;
	logic [`PR-1:0]     Told;
	logic 		    halt;
	logic [4:0] 	    arch_reg;
	logic 		    precise_state_need;
	logic 		    is_store; 
	logic [`XLEN-1:0]   target_pc;
	logic 		    completed;
	logic [`XLEN-1:0]   inst;
	logic               predict_direction;
        logic [`XLEN-1:0]   predict_pc;
} ROB_ENTRY_PACKET;

typedef struct packed {
    logic                   ready;
    logic [3:0]             usebytes;
    logic [`XLEN-1:0]       addr; // must be aligned with words
    logic [`XLEN-1:0]       data;
} SQ_ENTRY_PACKET;

typedef struct packed {
	logic		    stall;
	logic [3:0]	    usebytes;
	logic [`XLEN-1:0]   data;
} SQ_LOAD_PACKET;

typedef struct packed {
	logic [`LSQ-1:0]    tail_pos; // the tail position when load is dispatched
	logic [`XLEN-1:0]   addr; // must align with word! 
} LOAD_SQ_PACKET;

typedef struct packed{
     logic [3:0]            usebytes;
     logic [`XLEN-1:0]      addr; // must be aligned with words
     logic [`XLEN-1:0]      data;
} CACHE_IN_PACKET;

typedef struct packed{
    logic [3:0]             validbtyes;
    logic                   addr_ready;
    logic [3:0]		    tag;
    logic [`XLEN-1:0]       addr; // must be aligned with words
    logic [`XLEN-1:0]       data;
} LQ_ENTRY_PACKET;

typedef struct packed {
	logic               valid;
	logic [`XLEN-1:0]   tag;
	BP_STATE	    direction;
	logic [`XLEN-1:0]   target_pc;
	logic		    uncondition;
} BP_ENTRY_PACKET;

//////////////////////////////////////////////
//					    //
//             Memory definitions  	    //
//					    //
//////////////////////////////////////////////

`define CACHE_MODE //removes the byte-level interface from the memory mode, DO NOT MODIFY!
`define NUM_MEM_TAGS           15

`define MEM_SIZE_IN_BYTES      (64*1024)
`define MEM_64BIT_LINES        (`MEM_SIZE_IN_BYTES/8)

//you can change the clock period to whatever, 10 is just fine
`define VERILOG_CLOCK_PERIOD   13.1
`define SYNTH_CLOCK_PERIOD     13.1 // Clock period for synth and memory latency

`define MEM_LATENCY_IN_CYCLES (100.0/`SYNTH_CLOCK_PERIOD+0.49999)
// the 0.49999 is to force ceiling(100/period).  The default behavior for
// float to integer conversion is rounding to nearest

typedef union packed {
    logic [7:0][7:0] byte_level;
    logic [3:0][15:0] half_level;
    logic [1:0][31:0] word_level;
} EXAMPLE_CACHE_BLOCK;

typedef struct packed{
	logic [`XLEN-1:0]	addr;  // must be double-aligned
	logic [1:0]		command;
	logic [3:0]		mem_tag;
	logic 			left_or_right;  //If 1, left 4 bytes, if 0, right 4 bytes.
	logic [63:0]		data;
	logic 			issued;
	logic [1:0]		broadcast_fu;
	logic [7:0]		usebytes;
	logic 			dirty;
} MHSRS_ENTRY_PACKET;

// Memory bus commands control signals
typedef enum logic [1:0] {
	BUS_NONE     = 2'h0,
	BUS_LOAD     = 2'h1,
	BUS_STORE    = 2'h2
} BUS_COMMAND;

`ifndef CACHE_MODE
typedef enum logic [1:0] {
	BYTE = 2'h0,
	HALF = 2'h1,
	WORD = 2'h2,
	DOUBLE = 2'h3
} MEM_SIZE;
`endif

//////////////////////////////////////////////
//             				    //
//		Exception code		    //
//             				    //
//////////////////////////////////////////////

typedef enum logic [3:0] {
	INST_ADDR_MISALIGN  = 4'h0,
	INST_ACCESS_FAULT   = 4'h1,
	ILLEGAL_INST        = 4'h2,
	BREAKPOINT          = 4'h3,
	LOAD_ADDR_MISALIGN  = 4'h4,
	LOAD_ACCESS_FAULT   = 4'h5,
	STORE_ADDR_MISALIGN = 4'h6,
	STORE_ACCESS_FAULT  = 4'h7,
	ECALL_U_MODE        = 4'h8,
	ECALL_S_MODE        = 4'h9,
	NO_ERROR            = 4'ha, 
	ECALL_M_MODE        = 4'hb,
	INST_PAGE_FAULT     = 4'hc,
	LOAD_PAGE_FAULT     = 4'hd,
	HALTED_ON_WFI       = 4'he, 
	STORE_PAGE_FAULT    = 4'hf
} EXCEPTION_CODE;

`endif // __SYS_DEFS_VH__

