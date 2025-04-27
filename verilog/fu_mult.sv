`define TEST_MODE
`ifndef __MULT_SV__
`define __MULT_SV__

`include "verilog/sys_defs.svh"

`timescale 1ns/100ps

module mult_stage (
    input clock, reset, start,
    input [(2*`XLEN)-1:0] mplier_in, mcand_in,
    input [(2*`XLEN)-1:0] product_in,

    output logic done,
    output logic [(2*`XLEN)-1:0] mplier_out, mcand_out,
    output logic [(2*`XLEN)-1:0] product_out
);

    parameter NUM_BITS = (2*`XLEN)/`MULT_STAGES;

    logic [(2*`XLEN)-1:0] prod_in_reg, partial_prod, next_partial_product;
    logic [(2*`XLEN)-1:0] next_mplier, next_mcand;

    assign product_out = prod_in_reg + partial_prod;

    assign next_partial_product = mplier_in[(NUM_BITS-1):0] * mcand_in;

    assign next_mplier = {{(NUM_BITS){1'b0}}, mplier_in[2*`XLEN-1:(NUM_BITS)]};
    assign next_mcand  = {mcand_in[(2*`XLEN-1-NUM_BITS):0], {(NUM_BITS){1'b0}}};

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        prod_in_reg  <= `SD product_in;
        partial_prod <= `SD next_partial_product;
        mplier_out   <= `SD next_mplier;
        mcand_out    <= `SD next_mcand;
    end

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            done <= `SD 1'b0;
        end else begin
            done <= `SD start;
        end
    end

endmodule

module mult (
    input clock, reset,
    input start,
    input [1:0] sign,
    input [`XLEN-1:0] mcand, mplier,

    output [(2*`XLEN)-1:0] product,
    output logic [`MULT_STAGES:0] dones
);

    logic [(2*`XLEN)-1:0] mcand_out, mplier_out, mcand_in, mplier_in;
    logic [`MULT_STAGES:0][(2*`XLEN)-1:0] internal_mcands, internal_mpliers;
    logic [`MULT_STAGES:0][(2*`XLEN)-1:0] internal_products;
    logic [`MULT_STAGES:0] internal_dones;

    assign mcand_in  = sign[0] ? {{`XLEN{mcand[`XLEN-1]}}, mcand}   : {{`XLEN{1'b0}}, mcand};
    assign mplier_in = sign[1] ? {{`XLEN{mplier[`XLEN-1]}}, mplier} : {{`XLEN{1'b0}}, mplier};

    assign internal_mcands[0]   = mcand_in;
    assign internal_mpliers[0]  = mplier_in;
    assign internal_products[0] = 'h0;
    assign internal_dones[0]    = start;

    assign dones    = internal_dones;
    assign product  = internal_products[`MULT_STAGES];

    genvar i;
    for (i = 0; i < `MULT_STAGES; ++i) begin : mstage
        mult_stage ms (
            .clock(clock),
            .reset(reset),
            .product_in(internal_products[i]),
            .mplier_in(internal_mpliers[i]),
            .mcand_in(internal_mcands[i]),
            .start(internal_dones[i]),
            .product_out(internal_products[i+1]),
            .mplier_out(internal_mpliers[i+1]),
            .mcand_out(internal_mcands[i+1]),
            .done(internal_dones[i+1])
        );
    end
endmodule

module fu_mult (
	input                       clock,               // System clock
	input                       reset,               // Synchronous reset
	input                       complete_stall,      // Stall from complete stage
	input ISSUE_FU_PACKET       fu_packet_in,        // Incoming instruction packet
	output logic                fu_ready,            // Ready to accept new instruction
	output logic                want_to_complete,    // Signal when result is valid
	output FU_COMPLETE_PACKET   fu_packet_out        // Output to complete stage
);
// Internal control and operand registers
	logic start;                             // Start signal to multiplier
	logic [1:0] sign;                        // Sign control bits
	logic [`XLEN-1:0] rs1, rs2;              // Source operands from packet
	logic [2*`XLEN-1:0] product;             // Full product from multiplier
	logic [`XLEN-1:0] result;                // Selected result (low/high)
	FU_COMPLETE_PACKET result_pckt;         // Final output packet
	logic [`MULT_STAGES:0] dones;            // Done flags per stage
	ISSUE_FU_PACKET fu_in_reg;              // Latched packet for timing alignment

// Start when instruction is valid and pipeline is not stalled
	assign start = fu_packet_in.valid & ~complete_stall;
	assign rs1 = fu_packet_in.r1_value;
	assign rs2 = fu_packet_in.r2_value;

	// Select sign control based on instruction type
	always_comb begin
		priority case (fu_packet_in.op_sel.mult)
			MULT, MULH: begin
				sign[0] = rs1[`XLEN-1];      // Signed operand A
				sign[1] = rs2[`XLEN-1];      // Signed operand B
			end
			MULHSU: begin
				sign[0] = rs1[`XLEN-1];      // Signed operand A
				sign[1] = 1'b0;              // Unsigned operand B
			end
			MULHU: sign = 2'b00;              // Both operands unsigned
			default: sign = 2'b00;
		endcase
	end

// Instantiate the pipelined multiplier
	mult mult_0 (
		.clock(clock), 
		.reset(reset),
		.start(start),
		.sign(sign),
		.mcand(rs1),
		.mplier(rs2),
		.product(product),
		.dones(dones)
	);

	// Select low 32 bits for MULT, high bits otherwise (MULH/MULHU/etc.)
	always_comb begin
		if (fu_in_reg.op_sel.mult == MULT)
			result = product[`XLEN-1:0];
		else
			result = product[2*`XLEN-1:`XLEN];
	end


// Latch input packet for timing alignment with output
	always_ff @(posedge clock) begin
		if (reset)
			fu_in_reg <= `SD 0;
		else if (fu_packet_in.valid)
			fu_in_reg <= `SD fu_packet_in;
		else
			fu_in_reg <= `SD fu_in_reg;
	end

	// Construct output packet with result
	always_comb begin
		result_pckt = 0;
		result_pckt.halt = fu_in_reg.halt;                 // Propagate halt flag
		result_pckt.valid = dones[`MULT_STAGES];           // Mark as valid at last stage
		result_pckt.dest_pr = fu_in_reg.dest_pr;           // Destination physical register
		result_pckt.dest_value = result;                   // Computed result
		result_pckt.rob_entry = fu_in_reg.rob_entry;       // Reorder buffer entry
	end

// Output control logic
	assign want_to_complete = dones[`MULT_STAGES];          // Signal when output ready
	assign fu_ready = ~(|dones[`MULT_STAGES-1:0]);          // Ready if no pipeline is busy
	assign fu_packet_out = result_pckt;                     // Send output downstream

endmodule // fu_mult
`endif //__MULT_SV__
