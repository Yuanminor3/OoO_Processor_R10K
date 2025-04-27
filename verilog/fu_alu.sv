`ifndef __ALU_V__
`define __ALU_V__

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"


// ====================================================================
// Module: alu
// Description: Purely combinational ALU module to execute arithmetic
//              and logic operations based on a command code.
// ====================================================================
module alu(
    input  [`XLEN-1:0] opa,        // Operand A
    input  [`XLEN-1:0] opb,        // Operand B
    ALU_SELECT          func,      // ALU operation selector
    output logic [`XLEN-1:0] result // Result of ALU operation
);
    // Sign-extended versions for signed comparisons
    wire signed [`XLEN-1:0] signed_opa = opa;
    wire signed [`XLEN-1:0] signed_opb = opb;

    // Combinational logic to compute result based on operation type
    always_comb begin
        case (func)
            ALU_ADD:   result = opa + opb;
            ALU_SUB:   result = opa - opb;
            ALU_AND:   result = opa & opb;
            ALU_SLT:   result = signed_opa < signed_opb;
            ALU_SLTU:  result = opa < opb;
            ALU_OR:    result = opa | opb;
            ALU_XOR:   result = opa ^ opb;
            ALU_SRL:   result = opa >> opb[4:0];         // Logical shift right
            ALU_SLL:   result = opa << opb[4:0];         // Logical shift left
            ALU_SRA:   result = signed_opa >>> opb[4:0]; // Arithmetic shift right
            default:   result = `XLEN'hfacebeec;         // Default case
        endcase
    end
endmodule


module fu_alu(
    input                       clock,              // Clock
    input                       reset,              // Reset
    input                       complete_stall,     // Structural stall from complete stage
    input ISSUE_FU_PACKET       fu_packet_in,       // Input packet from issue stage
    output logic                fu_ready,           // Always ready to accept
    output logic                want_to_complete,   // Output valid signal
    output FU_COMPLETE_PACKET   fu_packet_out,      // Output to complete stage
    output                      if_store,           // Is instruction a store?
    output SQ_ENTRY_PACKET      store_pckt,         // Store queue packet
    output [`LSQ-1:0]           sq_idx              // Store queue index
);

// Local signals
    logic [`XLEN-1:0] opa_mux_out, opb_mux_out;
    FU_COMPLETE_PACKET result;
    ALU_SELECT alu_sel;


// Determine if instruction is a store
    assign if_store = fu_packet_in.valid && fu_packet_in.op_sel.alu >= SB;
    // Select ALU op: ALU_ADD for store address calculation, otherwise from op_sel
    assign alu_sel = if_store ? ALU_ADD : fu_packet_in.op_sel.alu;
    assign sq_idx  = fu_packet_in.sq_tail;



// Pre-fill default values in result packet
    assign result.if_take_branch = `FALSE;
    assign result.valid          = fu_packet_in.valid;
    assign result.halt           = fu_packet_in.halt;
    assign result.target_pc      = 0;
    assign result.dest_pr        = fu_packet_in.dest_pr;
    assign result.rob_entry      = fu_packet_in.rob_entry;

// Operand A MUX
    always_comb begin
        opa_mux_out = `XLEN'hdeadfbac;
        case (fu_packet_in.opa_select)
            OPA_IS_RS1:  opa_mux_out = fu_packet_in.r1_value;
            OPA_IS_NPC:  opa_mux_out = fu_packet_in.NPC;
            OPA_IS_PC:   opa_mux_out = fu_packet_in.PC;
            OPA_IS_ZERO: opa_mux_out = 0;
        endcase
    end

// Operand B MUX
    always_comb begin
        opb_mux_out = `XLEN'hfacefeed;
        case (fu_packet_in.opb_select)
            OPB_IS_RS2:   opb_mux_out = fu_packet_in.r2_value;
            OPB_IS_I_IMM: opb_mux_out = `RV32_signext_Iimm(fu_packet_in.inst);
            OPB_IS_S_IMM: opb_mux_out = `RV32_signext_Simm(fu_packet_in.inst);
            OPB_IS_B_IMM: opb_mux_out = `RV32_signext_Bimm(fu_packet_in.inst);
            OPB_IS_U_IMM: opb_mux_out = `RV32_signext_Uimm(fu_packet_in.inst);
            OPB_IS_J_IMM: opb_mux_out = `RV32_signext_Jimm(fu_packet_in.inst);
        endcase 
    end


// ALU execution
    logic [`XLEN-1:0] alu_result;
    alu alu_0 (
        .opa(opa_mux_out),
        .opb(opb_mux_out),
        .func(alu_sel),
        .result(alu_result)
    );
    assign result.dest_value = alu_result;

// Store packet generation
logic [`XLEN-1:0] store_val;
assign store_val = fu_packet_in.r2_value; 
always_comb begin
        store_pckt.addr     = {alu_result[`XLEN-1:2], 2'b00};
        store_pckt.ready    = 1;
        store_pckt.data     = 0;
        store_pckt.usebytes = 0;

        case (fu_packet_in.op_sel.alu)
            SB: case (alu_result[1:0])
                2'b00: begin store_pckt.usebytes = 4'b0001; store_pckt.data[7:0]   = store_val[7:0];   end
                2'b01: begin store_pckt.usebytes = 4'b0010; store_pckt.data[15:8]  = store_val[7:0];   end
                2'b10: begin store_pckt.usebytes = 4'b0100; store_pckt.data[23:16] = store_val[7:0];   end
                2'b11: begin store_pckt.usebytes = 4'b1000; store_pckt.data[31:24] = store_val[7:0];   end
            endcase
            SH: case (alu_result[1:0])
                2'b00: begin store_pckt.usebytes = 4'b0011; store_pckt.data[15:0]  = store_val[15:0];  end
                2'b10: begin store_pckt.usebytes = 4'b1100; store_pckt.data[31:16] = store_val[15:0];  end
            endcase
            SW: begin
                store_pckt.usebytes = 4'b1111;
                store_pckt.data     = store_val;
            end
        endcase
    end


// Final assignments
    assign want_to_complete = fu_packet_in.valid;
    assign fu_ready         = 1;
    assign fu_packet_out    = result;

endmodule // module 
`endif // __ALU_V__
