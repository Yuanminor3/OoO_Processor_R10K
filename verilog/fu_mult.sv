
`ifndef __MULT_SV__
`define __MULT_SV__

`include "verilog/sys_defs.svh"

`timescale 1ns/100ps

// =================================================================
// Modulename : mult_stage
// Description : One stage for multi-cycle multiplication
// =================================================================
module mult_stage #(
    parameter SYS_XLEN = 32, 
    parameter NUM_STAGE = 4
)(
    input  logic                   clk,
    input  logic                   rst,
    input  logic                   start,
    input  logic [(2*SYS_XLEN)-1:0]     mls_in_multiplier,
    input  logic [(2*SYS_XLEN)-1:0]     mcand_in,
    input  logic [(2*SYS_XLEN)-1:0]     mls_in_partial_product,

    output logic                   done,
    output logic [(2*SYS_XLEN)-1:0]     mls_out_multiplier,
    output logic [(2*SYS_XLEN)-1:0]     mls_out_multiplicand,
    output logic [(2*SYS_XLEN)-1:0]     mls_out_partial_product
);

    // Divide multiplication bits equally across stages
    parameter NUM_BITS = (2*SYS_XLEN)/NUM_STAGE;

    // Internal wires for partial computation
    logic [(2*SYS_XLEN)-1:0] mls_calc_partial_prodc;
    logic [(2*SYS_XLEN)-1:0] mls_shifted_multiplier_nxt, mls_shifted_multiplicand_nxt;

    // Generate partial product from lower bits
    assign mls_calc_partial_prodc = mls_in_multiplier[(NUM_BITS-1):0] * mcand_in;
    assign mls_shifted_multiplier_nxt     = {{(NUM_BITS){1'b0}}, mls_in_multiplier[2*SYS_XLEN-1:NUM_BITS]};
    assign mls_shifted_multiplicand_nxt      = {mcand_in[2*SYS_XLEN-1-NUM_BITS:0], {(NUM_BITS){1'b0}}};

    // Sequential update on clk
    always_ff @(posedge clk) begin
        if (rst) begin
            mls_out_multiplier   <= 0;
            mls_out_multiplicand    <= 0;
            mls_out_partial_product  <= 0;
            done         <= 1'b0;
        end else begin
            mls_out_multiplier   <= `SYS_SMALL_DELAY mls_shifted_multiplier_nxt;
            mls_out_multiplicand    <= `SYS_SMALL_DELAY mls_shifted_multiplicand_nxt;
            mls_out_partial_product  <= `SYS_SMALL_DELAY mls_in_partial_product + mls_calc_partial_prodc;
            done         <= `SYS_SMALL_DELAY start;
        end
    end

endmodule

// =================================================================                                                          //
// Modulename : mult
// Description : Top-level pipelined multiplier module
// =================================================================
module mult #(
    parameter SYS_XLEN = 32,
    parameter NUM_STAGE = 4
)(
    input  logic                   clk,
    input  logic                   rst,
    input  logic                   start,
    input  logic [1:0]              sign,
    input  logic [SYS_XLEN-1:0]         mlp_initial_multiplicand,
    input  logic [SYS_XLEN-1:0]         mlp_initial_multiplier,

    output logic [(2*SYS_XLEN)-1:0]     product,
    output logic [NUM_STAGE:0]      mlp_stage_done_flags
);

    // Internal data arrays to pass between pipeline stages
    logic [(2*SYS_XLEN)-1:0] mcand_in, mls_in_multiplier;
    logic [NUM_STAGE:0][2*SYS_XLEN-1:0] mlp_stage_multiplicands, mlp_stage_multipliers;
    logic [NUM_STAGE:0][2*SYS_XLEN-1:0] mlp_stage_partial_products;
    logic [NUM_STAGE:0]             mlp_stage_done_pipeline_flags;

    // Sign extension for multiplicand and multiplier
    assign mcand_in  = sign[0] ? {{SYS_XLEN{mlp_initial_multiplicand[SYS_XLEN-1]}}, mlp_initial_multiplicand}   : {{SYS_XLEN{1'b0}}, mlp_initial_multiplicand};
    assign mls_in_multiplier = sign[1] ? {{SYS_XLEN{mlp_initial_multiplier[SYS_XLEN-1]}}, mlp_initial_multiplier} : {{SYS_XLEN{1'b0}}, mlp_initial_multiplier};

    // Initialize pipeline
    assign mlp_stage_multiplicands[0]   = mcand_in;
    assign mlp_stage_multipliers[0]  = mls_in_multiplier;
    assign mlp_stage_partial_products[0] = 'h0;
    assign mlp_stage_done_pipeline_flags[0]    = start;

    // Instantiate mult_stage units
    genvar i;
    for (i = 0; i < NUM_STAGE; ++i) begin : mstage
        mult_stage #(
            .SYS_XLEN(SYS_XLEN),
            .NUM_STAGE(NUM_STAGE)
        ) u_mult_stage (
            .clk(clk),
            .rst(rst),
            .mls_in_partial_product(mlp_stage_partial_products[i]),
            .mls_in_multiplier(mlp_stage_multipliers[i]),
            .mcand_in(mlp_stage_multiplicands[i]),
            .start(mlp_stage_done_pipeline_flags[i]),
            .mls_out_partial_product(mlp_stage_partial_products[i+1]),
            .mls_out_multiplier(mlp_stage_multipliers[i+1]),
            .mls_out_multiplicand(mlp_stage_multiplicands[i+1]),
            .done(mlp_stage_done_pipeline_flags[i+1])
        );
    end

    // Final outputs after all stages
    assign product = mlp_stage_partial_products[NUM_STAGE];
    assign mlp_stage_done_flags   = mlp_stage_done_pipeline_flags;

endmodule

// =================================================================
// Module: fu_mult
// Description : Functional unit for executing multiply instructions
// =================================================================
module fu_mult(
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 bs_hazard,
    input  ISSUE_FU_PACKET       bs_in_pkt,

    output logic                 rsb_fu_ready,
    output logic                 fum_complete_req,
    output FU_COMPLETE_PACKET    bs_out_pkt
);

    // Internal signals
    logic                        start;
    logic [1:0]                  sign;
    logic [`SYS_XLEN-1:0]             cmp_opA, cmp_opB;
    logic [2*`SYS_XLEN-1:0]           product;
    logic [`SYS_XLEN-1:0]             result;
    logic [`SYS_MUL_STAGES:0]          mlp_stage_done_flags;
    ISSUE_FU_PACKET               fum_saved_issue_pkt;
    FU_COMPLETE_PACKET            fum_temp_completion_pkt;

    // Control signal generation
    assign start = bs_in_pkt.valid && ~bs_hazard;
    assign cmp_opA   = bs_in_pkt.r1_value;
    assign cmp_opB   = bs_in_pkt.r2_value;

    // Determine operand signedness based on instruction type
    always_comb begin
        unique case (bs_in_pkt.dec_fu_opcode.mult)
            MULT, MULH: begin
                sign[0] = cmp_opA[`SYS_XLEN-1];
                sign[1] = cmp_opB[`SYS_XLEN-1];
            end
            MULHSU: begin
                sign[0] = cmp_opA[`SYS_XLEN-1];
                sign[1] = 1'b0;
            end
            MULHU: sign = 2'b00;
            default: sign = 2'b00;
        endcase
    end

    // Multiplier instantiation
    mult #(
        .SYS_XLEN(`SYS_XLEN),
        .NUM_STAGE(`SYS_MUL_STAGES)
    ) mult_inst (
        .clk(clk),
        .rst(rst),
        .start(start),
        .sign(sign),
        .mlp_initial_multiplicand(cmp_opA),
        .mlp_initial_multiplier(cmp_opB),
        .product(product),
        .mlp_stage_done_flags(mlp_stage_done_flags)
    );

    // Register instruction metadata
    always_ff @(posedge clk) begin
        if (rst)
            fum_saved_issue_pkt <= `SYS_SMALL_DELAY 0;
        else if (bs_in_pkt.valid)
            fum_saved_issue_pkt <= `SYS_SMALL_DELAY bs_in_pkt;
    end

    // Select output portion depending on operation
    always_comb begin
        if (fum_saved_issue_pkt.dec_fu_opcode.mult == MULT)
            result = product[`SYS_XLEN-1:0];
        else
            result = product[2*`SYS_XLEN-1:`SYS_XLEN];
    end

    // Package the output
    always_comb begin
        fum_temp_completion_pkt            = '0;
        fum_temp_completion_pkt.halt       = fum_saved_issue_pkt.halt;
        fum_temp_completion_pkt.valid      = mlp_stage_done_flags[`SYS_MUL_STAGES];
        fum_temp_completion_pkt.dispatch_allocated_prs    = fum_saved_issue_pkt.dispatch_allocated_prs;
        fum_temp_completion_pkt.dest_value = result;
        fum_temp_completion_pkt.rob_entry  = fum_saved_issue_pkt.rob_entry;
    end

    // Output control
    assign fum_complete_req = mlp_stage_done_flags[`SYS_MUL_STAGES];
    assign rsb_fu_ready          = ~(|mlp_stage_done_flags[`SYS_MUL_STAGES-1:0]);
    assign bs_out_pkt     = fum_temp_completion_pkt;

endmodule

`endif // __MULT_SV__
