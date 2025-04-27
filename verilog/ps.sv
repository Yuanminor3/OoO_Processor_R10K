
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

// =====================
// Module: ps2
// Description: 2-input fixed-priority selector (req[1] > req[0]).
// Outputs one-hot grant and indicates if any request is active.
// =====================
module ps2 (
    input        [1:0] req,     // Request input vector
    input              en,      // Enable signal
    output logic [1:0] gnt,     // One-hot grant output
    output logic       req_up   // Whether any request is active
);
    always_comb begin
        if (~en)
            gnt = 2'b00;
        else if (req[1])
            gnt = 2'b10;
        else if (req[0])
            gnt = 2'b01;
        else
            gnt = 2'b00;
    end

    always_comb begin
        req_up = (req[1] | req[0]);
    end
endmodule


// =====================
// Module: ps4
// Description: 4-input fixed-priority selector.
// Built from 3 ps2 modules: 2 leaf, 1 top.
// =====================
module ps4 (
    input         [3:0] req,     // Request vector
    input               en,      // Enable signal
    output logic  [3:0] gnt,     // One-hot grant
    output logic        req_up   // Any request active
);
    wire [1:0] tmp;              // req_up signals from child ps2s
    wire [1:0] sel;              // enable signals for child ps2s

    ps2 psl (
        .req(req[3:2]), .en(sel[1]),
        .gnt(gnt[3:2]), .req_up(tmp[1])
    );

    ps2 psr (
        .req(req[1:0]), .en(sel[0]),
        .gnt(gnt[1:0]), .req_up(tmp[0])
    );

    ps2 p2top (
        .req(tmp), .en(en),
        .gnt(sel), .req_up(req_up)
    );
endmodule


// =====================
// Module: ps8
// Description: 8-input fixed-priority selector.
// Constructed with 2 ps4s + 1 ps2 as top arbiter.
// =====================
module ps8 (
    input        [7:0] req,      // Request vector
    input              en,       // Enable
    output logic [7:0] gnt,      // One-hot grant
    output logic       req_up    // Any request active
);
    wire [1:0] tmp;
    wire [1:0] sel;

    ps4 psl (
        .req(req[7:4]), .en(sel[1]),
        .gnt(gnt[7:4]), .req_up(tmp[1])
    );

    ps4 psr (
        .req(req[3:0]), .en(sel[0]),
        .gnt(gnt[3:0]), .req_up(tmp[0])
    );

    ps2 pstop (
        .req(tmp), .en(en),
        .gnt(sel), .req_up(req_up)
    );
endmodule


// =====================
// Module: ps16
// Description: 16-input fixed-priority selector.
// Constructed from 4 ps4s and 1 top ps4 for hierarchical selection.
// =====================
module ps16 (
    input         [15:0] req,     // Request vector
    input                en,      // Enable
    output logic  [15:0] gnt,     // One-hot grant
    output logic         req_up   // Any request active
);
    wire [3:0] tmp;
    wire [3:0] sel;

    ps4 ps_3 (
        .req(req[15:12]), .en(sel[3]),
        .gnt(gnt[15:12]), .req_up(tmp[3])
    );

    ps4 ps_2 (
        .req(req[11:8]), .en(sel[2]),
        .gnt(gnt[11:8]), .req_up(tmp[2])
    );

    ps4 ps_1 (
        .req(req[7:4]), .en(sel[1]),
        .gnt(gnt[7:4]), .req_up(tmp[1])
    );

    ps4 ps_0 (
        .req(req[3:0]), .en(sel[0]),
        .gnt(gnt[3:0]), .req_up(tmp[0])
    );

    ps4 pstop (
        .req(tmp), .en(en),
        .gnt(sel), .req_up(req_up)
    );
endmodule



// =====================
// Module: pc_sel2
// Description: 2-way PC-based priority selector.
// Chooses the request with the smaller PC value.
// =====================
module pc_sel2 (
    input  [1:0][`XLEN-1:0] pc,     // Two PC values
    input  [1:0]            req,    // Two request bits
    input                  en,      // Enable signal
    output logic [1:0]      gnt,    // One-hot grant output
    output logic            req_up, // Whether any request is up
    output logic [`XLEN-1:0] pc_up  // Selected PC value
);

    logic [1:0] pri;
    wire        smaller;

    assign req_up = req[1] || req[0];             // Any request active?
    assign gnt    = en ? pri : 2'b00;             // Only grant when enabled
    assign smaller = (pc[1] < pc[0]);             // PC comparison

    always_comb begin
        case (req)
            2'b11: begin                          // Both requests active
                pri   = smaller ? 2'b10 : 2'b01;  // Pick smaller PC
                pc_up = smaller ? pc[1] : pc[0];
            end
            2'b10: begin
                pri   = 2'b10;
                pc_up = pc[1];
            end
            2'b01: begin
                pri   = 2'b01;
                pc_up = pc[0];
            end
            default: begin
                pri   = 2'b00;
                pc_up = `XLEN'hfffffff;           // Default max value
            end
        endcase
    end

endmodule

// =====================
// Module: pc_sel4
// Description: 4-way PC-based priority selector.
// Hierarchically composed of 3 pc_sel2 modules.
// =====================
module pc_sel4 (
    input  [3:0][`XLEN-1:0] pc,
    input  [3:0]            req,
    input                   en,
    output logic [3:0]      gnt,
    output logic            req_up,
    output logic [`XLEN-1:0] pc_up
);

    wire [1:0]            req_children;
    wire [1:0][`XLEN-1:0] pc_children;
    wire [1:0]            en_children;

    // Left and right 2-way selectors
    pc_sel2 sell (
        .pc(pc[3:2]), .req(req[3:2]), .en(en_children[1]),
        .gnt(gnt[3:2]), .req_up(req_children[1]), .pc_up(pc_children[1])
    );

    pc_sel2 selr (
        .pc(pc[1:0]), .req(req[1:0]), .en(en_children[0]),
        .gnt(gnt[1:0]), .req_up(req_children[0]), .pc_up(pc_children[0])
    );

    // Top-level selector
    pc_sel2 seltop (
        .pc(pc_children), .req(req_children), .en(en),
        .gnt(en_children), .req_up(req_up), .pc_up(pc_up)
    );

endmodule

`timescale 1ns/100ps

// =====================
// Module: pc_sel16
// Description: 16-way PC-based priority selector.
// Built using 5 pc_sel4 modules in a tree structure.
// =====================
module pc_sel16 (
    input  [15:0][`XLEN-1:0] pc,
    input  [15:0]            req,
    input                    en,
    output logic [15:0]      gnt,
    output logic             req_up,
    output logic [`XLEN-1:0] pc_up
);

    wire [3:0]             req_children;
    wire [3:0][`XLEN-1:0]  pc_children;
    wire [3:0]             en_children;

    // Level 1: 4 subgroups of 4
    pc_sel4 sel3 (
        .pc(pc[15:12]), .req(req[15:12]), .en(en_children[3]),
        .gnt(gnt[15:12]), .req_up(req_children[3]), .pc_up(pc_children[3])
    );

    pc_sel4 sel2 (
        .pc(pc[11:8]), .req(req[11:8]), .en(en_children[2]),
        .gnt(gnt[11:8]), .req_up(req_children[2]), .pc_up(pc_children[2])
    );

    pc_sel4 sel1 (
        .pc(pc[7:4]), .req(req[7:4]), .en(en_children[1]),
        .gnt(gnt[7:4]), .req_up(req_children[1]), .pc_up(pc_children[1])
    );

    pc_sel4 sel0 (
        .pc(pc[3:0]), .req(req[3:0]), .en(en_children[0]),
        .gnt(gnt[3:0]), .req_up(req_children[0]), .pc_up(pc_children[0])
    );

    // Level 2: Final selector among the four 4-way winners
    pc_sel4 seltop (
        .pc(pc_children), .req(req_children), .en(en),
        .gnt(en_children), .req_up(req_up), .pc_up(pc_up)
    );

endmodule