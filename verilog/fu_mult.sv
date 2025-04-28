`define TEST_MODE
`ifndef __MULT_SV__
`define __MULT_SV__

`include "verilog/sys_defs.svh"

`timescale 1ns/100ps

// =================================================================
// Modulename : mult_stage
// Description : One stage for multi-cycle multiplication
// =================================================================
module mult_stage #(
    parameter XLEN = 32, 
    parameter NUM_STAGE = 4
)(
    input  logic                   clock,
    input  logic                   reset,
    input  logic                   start,
    input  logic [(2*XLEN)-1:0]     mplier_in,
    input  logic [(2*XLEN)-1:0]     mcand_in,
    input  logic [(2*XLEN)-1:0]     product_in,

    output logic                   done,
    output logic [(2*XLEN)-1:0]     mplier_out,
    output logic [(2*XLEN)-1:0]     mcand_out,
    output logic [(2*XLEN)-1:0]     product_out
);

    // Divide multiplication bits equally across stages
    parameter NUM_BITS = (2*XLEN)/NUM_STAGE;

    // Internal wires for partial computation
    logic [(2*XLEN)-1:0] partial_product;
    logic [(2*XLEN)-1:0] next_mplier, next_mcand;

    // Generate partial product from lower bits
    assign partial_product = mplier_in[(NUM_BITS-1):0] * mcand_in;
    assign next_mplier     = {{(NUM_BITS){1'b0}}, mplier_in[2*XLEN-1:NUM_BITS]};
    assign next_mcand      = {mcand_in[2*XLEN-1-NUM_BITS:0], {(NUM_BITS){1'b0}}};

    // Sequential update on clock
    always_ff @(posedge clock) begin
        if (reset) begin
            mplier_out   <= 0;
            mcand_out    <= 0;
            product_out  <= 0;
            done         <= 1'b0;
        end else begin
            mplier_out   <= `SD next_mplier;
            mcand_out    <= `SD next_mcand;
            product_out  <= `SD product_in + partial_product;
            done         <= `SD start;
        end
    end

endmodule

// =================================================================                                                          //
// Modulename : mult
// Description : Top-level pipelined multiplier module
// =================================================================
module mult #(
    parameter XLEN = 32,
    parameter NUM_STAGE = 4
)(
    input  logic                   clock,
    input  logic                   reset,
    input  logic                   start,
    input  logic [1:0]              sign,
    input  logic [XLEN-1:0]         mcand,
    input  logic [XLEN-1:0]         mplier,

    output logic [(2*XLEN)-1:0]     product,
    output logic [NUM_STAGE:0]      dones
);

    // Internal data arrays to pass between pipeline stages
    logic [(2*XLEN)-1:0] mcand_in, mplier_in;
    logic [NUM_STAGE:0][2*XLEN-1:0] internal_mcands, internal_mpliers;
    logic [NUM_STAGE:0][2*XLEN-1:0] internal_products;
    logic [NUM_STAGE:0]             internal_dones;

    // Sign extension for multiplicand and multiplier
    assign mcand_in  = sign[0] ? {{XLEN{mcand[XLEN-1]}}, mcand}   : {{XLEN{1'b0}}, mcand};
    assign mplier_in = sign[1] ? {{XLEN{mplier[XLEN-1]}}, mplier} : {{XLEN{1'b0}}, mplier};

    // Initialize pipeline
    assign internal_mcands[0]   = mcand_in;
    assign internal_mpliers[0]  = mplier_in;
    assign internal_products[0] = 'h0;
    assign internal_dones[0]    = start;

    // Instantiate mult_stage units
    genvar i;
    for (i = 0; i < NUM_STAGE; ++i) begin : mstage
        mult_stage #(
            .XLEN(XLEN),
            .NUM_STAGE(NUM_STAGE)
        ) u_mult_stage (
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

    // Final outputs after all stages
    assign product = internal_products[NUM_STAGE];
    assign dones   = internal_dones;

endmodule

// =================================================================
// Module: fu_mult
// Description : Functional unit for executing multiply instructions
// =================================================================
module fu_mult(
    input  logic                 clock,
    input  logic                 reset,
    input  logic                 complete_stall,
    input  ISSUE_FU_PACKET       fu_packet_in,

    output logic                 fu_ready,
    output logic                 want_to_complete,
    output FU_COMPLETE_PACKET    fu_packet_out
);

    // Internal signals
    logic                        start;
    logic [1:0]                  sign;
    logic [`XLEN-1:0]             rs1, rs2;
    logic [2*`XLEN-1:0]           product;
    logic [`XLEN-1:0]             result;
    logic [`MUL_STAGE:0]          dones;
    ISSUE_FU_PACKET               fu_in_reg;
    FU_COMPLETE_PACKET            result_pckt;

    // Control signal generation
    assign start = fu_packet_in.valid && ~complete_stall;
    assign rs1   = fu_packet_in.r1_value;
    assign rs2   = fu_packet_in.r2_value;

    // Determine operand signedness based on instruction type
    always_comb begin
        unique case (fu_packet_in.op_sel.mult)
            MULT, MULH: begin
                sign[0] = rs1[`XLEN-1];
                sign[1] = rs2[`XLEN-1];
            end
            MULHSU: begin
                sign[0] = rs1[`XLEN-1];
                sign[1] = 1'b0;
            end
            MULHU: sign = 2'b00;
            default: sign = 2'b00;
        endcase
    end

    // Multiplier instantiation
    mult #(
        .XLEN(`XLEN),
        .NUM_STAGE(`MUL_STAGE)
    ) mult_inst (
        .clock(clock),
        .reset(reset),
        .start(start),
        .sign(sign),
        .mcand(rs1),
        .mplier(rs2),
        .product(product),
        .dones(dones)
    );

    // Register instruction metadata
    always_ff @(posedge clock) begin
        if (reset)
            fu_in_reg <= `SD 0;
        else if (fu_packet_in.valid)
            fu_in_reg <= `SD fu_packet_in;
    end

    // Select output portion depending on operation
    always_comb begin
        if (fu_in_reg.op_sel.mult == MULT)
            result = product[`XLEN-1:0];
        else
            result = product[2*`XLEN-1:`XLEN];
    end

    // Package the output
    always_comb begin
        result_pckt            = '0;
        result_pckt.halt       = fu_in_reg.halt;
        result_pckt.valid      = dones[`MUL_STAGE];
        result_pckt.dest_pr    = fu_in_reg.dest_pr;
        result_pckt.dest_value = result;
        result_pckt.rob_entry  = fu_in_reg.rob_entry;
    end

    // Output control
    assign want_to_complete = dones[`MUL_STAGE];
    assign fu_ready          = ~(|dones[`MUL_STAGE-1:0]);
    assign fu_packet_out     = result_pckt;

endmodule

`endif // __MULT_SV__
