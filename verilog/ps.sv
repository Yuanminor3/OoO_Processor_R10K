
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module ps2 (
    input  logic [1:0] req,
    input  logic       en,
    output logic [1:0] gnt,
    output logic       req_up
);
    always_comb begin
        gnt = 2'b00;
        if (en) begin
            if (req[1])
                gnt = 2'b10;
            else if (req[0])
                gnt = 2'b01;
        end
    end

    always_comb begin
        req_up = |req;  // req[1] | req[0]
    end
endmodule

module ps4 (
    input  logic [3:0] req,
    input  logic       en,
    output logic [3:0] gnt,
    output logic       req_up
);
    logic [1:0] state, sel;

    ps2 psl   (.req(req[3:2]), .en(sel[1]), .gnt(gnt[3:2]), .req_up(state[1]));
    ps2 psr   (.req(req[1:0]), .en(sel[0]), .gnt(gnt[1:0]), .req_up(state[0]));
    ps2 p2top (.req(state),      .en(en),     .gnt(sel),       .req_up(req_up));
endmodule

module ps8 (
    input  logic [7:0] req,
    input  logic       en,
    output logic [7:0] gnt,
    output logic       req_up
);
    logic [1:0] state, sel;

    ps4 psl   (.req(req[7:4]), .en(sel[1]), .gnt(gnt[7:4]), .req_up(state[1]));
    ps4 psr   (.req(req[3:0]), .en(sel[0]), .gnt(gnt[3:0]), .req_up(state[0]));
    ps2 pstop (.req(state),      .en(en),     .gnt(sel),       .req_up(req_up));
endmodule

module ps16 (
    input  logic [15:0] req,
    input  logic        en,
    output logic [15:0] gnt,
    output logic        req_up
);
    logic [3:0] state, sel;

    ps4 ps3   (.req(req[15:12]), .en(sel[3]), .gnt(gnt[15:12]), .req_up(state[3]));
    ps4 ps2   (.req(req[11:8]),  .en(sel[2]), .gnt(gnt[11:8]),  .req_up(state[2]));
    ps4 ps1   (.req(req[7:4]),   .en(sel[1]), .gnt(gnt[7:4]),   .req_up(state[1]));
    ps4 ps0   (.req(req[3:0]),   .en(sel[0]), .gnt(gnt[3:0]),   .req_up(state[0]));
    ps4 pstop (.req(state),        .en(en),     .gnt(sel),         .req_up(req_up));
endmodule

