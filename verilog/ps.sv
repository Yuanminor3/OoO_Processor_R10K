
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module ps2 (
    input  logic [1:0] rcmp_eq_flag,
    input  logic       en,
    output logic [1:0] gnt,
    output logic       req_up
);
    always_comb begin
        gnt = 2'b00;
        if (en) begin
            if (rcmp_eq_flag[1])
                gnt = 2'b10;
            else if (rcmp_eq_flag[0])
                gnt = 2'b01;
        end
    end

    always_comb begin
        req_up = |rcmp_eq_flag;  // rcmp_eq_flag[1] | rcmp_eq_flag[0]
    end
endmodule

module ps4 (
    input  logic [3:0] rcmp_eq_flag,
    input  logic       en,
    output logic [3:0] gnt,
    output logic       req_up
);
    logic [1:0] state, sel;

    ps2 psl   (.rcmp_eq_flag(rcmp_eq_flag[3:2]), .en(sel[1]), .gnt(gnt[3:2]), .req_up(state[1]));
    ps2 psr   (.rcmp_eq_flag(rcmp_eq_flag[1:0]), .en(sel[0]), .gnt(gnt[1:0]), .req_up(state[0]));
    ps2 p2top (.rcmp_eq_flag(state),      .en(en),     .gnt(sel),       .req_up(req_up));
endmodule

module ps8 (
    input  logic [7:0] rcmp_eq_flag,
    input  logic       en,
    output logic [7:0] gnt,
    output logic       req_up
);
    logic [1:0] state, sel;

    ps4 psl   (.rcmp_eq_flag(rcmp_eq_flag[7:4]), .en(sel[1]), .gnt(gnt[7:4]), .req_up(state[1]));
    ps4 psr   (.rcmp_eq_flag(rcmp_eq_flag[3:0]), .en(sel[0]), .gnt(gnt[3:0]), .req_up(state[0]));
    ps2 pstop (.rcmp_eq_flag(state),      .en(en),     .gnt(sel),       .req_up(req_up));
endmodule

module ps16 (
    input  logic [15:0] rcmp_eq_flag,
    input  logic        en,
    output logic [15:0] gnt,
    output logic        req_up
);
    logic [3:0] state, sel;

    ps4 ps3   (.rcmp_eq_flag(rcmp_eq_flag[15:12]), .en(sel[3]), .gnt(gnt[15:12]), .req_up(state[3]));
    ps4 ps2   (.rcmp_eq_flag(rcmp_eq_flag[11:8]),  .en(sel[2]), .gnt(gnt[11:8]),  .req_up(state[2]));
    ps4 ps1   (.rcmp_eq_flag(rcmp_eq_flag[7:4]),   .en(sel[1]), .gnt(gnt[7:4]),   .req_up(state[1]));
    ps4 ps0   (.rcmp_eq_flag(rcmp_eq_flag[3:0]),   .en(sel[0]), .gnt(gnt[3:0]),   .req_up(state[0]));
    ps4 pstop (.rcmp_eq_flag(state),        .en(en),     .gnt(sel),         .req_up(req_up));
endmodule

