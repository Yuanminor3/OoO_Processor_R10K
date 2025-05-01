
`ifndef __ISSUE_V__
`define __ISSUE_V__
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module issue_stage(
    input                               clock,
    input                               reset,
    input       RS_S_PACKET [2:0]       is_packet_in,
    input       ISSUE_FU_PACKET [2:0]   issue_fu_packet,
    input       FU_STATE_PACKET 	fu_ready_is,
    output      FU_FIFO_PACKET		fu_fifo_stall,
    output      ISSUE_FU_PACKET [2**`FU-1:0] is_fu_packet
);

// assign packet to fu fifo
ISSUE_FU_PACKET [2:0]   alu_fifo_in;
ISSUE_FU_PACKET [2:0]   mult_fifo_in;
ISSUE_FU_PACKET [2:0]   ls_fifo_in;
ISSUE_FU_PACKET [2:0]   br_fifo_in;
always_comb begin
    alu_fifo_in = 0;
    mult_fifo_in = 0;
    ls_fifo_in = 0;
    br_fifo_in = 0;
    for(int i=0; i<3; i++) begin
        alu_fifo_in[i] = is_packet_in[i].fu_sel == ALU_1 ? issue_fu_packet[i]:0;
        mult_fifo_in[i] = is_packet_in[i].fu_sel == MULT_1 ? issue_fu_packet[i]:0;
        ls_fifo_in[i] = is_packet_in[i].fu_sel == LS_1 ? issue_fu_packet[i]:0;
        br_fifo_in[i] = is_packet_in[i].fu_sel == BRANCH ? issue_fu_packet[i]:0;
    end
end
ISSUE_FU_PACKET [3:0] issue_waste;

fu_FIFO_3 alu_fifo(
    .clock(clock),
    .reset(reset),
    .fu_pckt_in(alu_fifo_in),
    .rd_EN({fu_ready_is.alu_1, fu_ready_is.alu_2, fu_ready_is.alu_3}),
    //.full(fifo_full[0]),
    .almost_full(fu_fifo_stall.alu),
    .fu_pckt_out({is_fu_packet[ALU_1], is_fu_packet[ALU_2], is_fu_packet[ALU_3]})
   
);

fu_FIFO_3 ls_fifo(
    .clock(clock),
    .reset(reset ),
    .fu_pckt_in(ls_fifo_in),
    .rd_EN({fu_ready_is.loadstore_1, fu_ready_is.loadstore_2, 1'b0}),
    //.full(fifo_full[1]),
    .almost_full(fu_fifo_stall.ls),
    .fu_pckt_out({is_fu_packet[LS_1], is_fu_packet[LS_2], issue_waste[0]})

);

fu_FIFO_3 mult_fifo(
    .clock(clock),
    .reset(reset ),
    .fu_pckt_in(mult_fifo_in),
    .rd_EN({fu_ready_is.mult_1, fu_ready_is.mult_2, 1'b0}),
    //.full(fifo_full[2]),
    .almost_full(fu_fifo_stall.mult),
    .fu_pckt_out({is_fu_packet[MULT_1], is_fu_packet[MULT_2], issue_waste[1]})

);

fu_FIFO_3 br_fifo(
    .clock(clock),
    .reset(reset ),
    .fu_pckt_in(br_fifo_in),
    .rd_EN({fu_ready_is.branch, 2'b0}),
    //.full(fifo_full[3]),
    .almost_full(fu_fifo_stall.branch),
    .fu_pckt_out({is_fu_packet[BRANCH], issue_waste[3:2]})

);

endmodule

module fu_FIFO_3 #(parameter FIFO_DEPTH=`IS_FIFO_DEPTH)(
    input                       clock,
    input                       reset,
    input ISSUE_FU_PACKET[2:0]  fu_pckt_in,
    input [2:0]                 rd_EN,
    //output logic                full,
    output logic                almost_full,
    output ISSUE_FU_PACKET[2:0] fu_pckt_out

);
/* sync registers */
ISSUE_FU_PACKET [FIFO_DEPTH-1:0] fifo_entries;
logic [$clog2(FIFO_DEPTH)-1:0] tail;

/* write count and EN */
logic [2:0] wr_EN;
logic [1:0]     wr_count;
always_comb begin
    for(int i=0; i<3; i++) begin
        wr_EN[i] = fu_pckt_in[i].valid;
    end
end
assign wr_count = wr_EN[0] + wr_EN[1] + wr_EN[2];

    // --------------------------
    // Reorder input
    // --------------------------
    ISSUE_FU_PACKET [2:0] fifo_push;

    always_comb begin
        fifo_push = '{default: 0};
        unique case (wr_EN)
            3'b001: fifo_push[2]       = fu_pckt_in[0];
            3'b010: fifo_push[2]       = fu_pckt_in[1];
            3'b011: fifo_push[2:1]     = fu_pckt_in[1:0];
            3'b100: fifo_push[2]       = fu_pckt_in[2];
            3'b101: begin
                        fifo_push[2]   = fu_pckt_in[2];
                        fifo_push[1]   = fu_pckt_in[0];
                    end
            3'b110: fifo_push[2:1]     = fu_pckt_in[2:1];
            3'b111: fifo_push          = fu_pckt_in;
            default: fifo_push         = '{default: 0};
        endcase
    end

    // --------------------------
    // Tail Calculation
    // --------------------------
    logic [$clog2(FIFO_DEPTH):0] tail_next;

    assign tail_next    = tail + wr_count;
    //assign full         = (tail_next >= FIFO_DEPTH);
    assign almost_full  = (tail_next + 3 >= FIFO_DEPTH);

    // --------------------------
    // Write to FIFO (tentative)
    // --------------------------
ISSUE_FU_PACKET [FIFO_DEPTH-1+3:0] fifo_entries_next;
always_comb begin
    fifo_entries_next = 0;
    fifo_entries_next[FIFO_DEPTH-1:0] = fifo_entries;
    for(int i=0; i<FIFO_DEPTH; i++) begin
        if (i == tail) begin
            fifo_entries_next[i] = fifo_push[2];
            fifo_entries_next[i+1] = fifo_push[1];
            fifo_entries_next[i+2] = fifo_push[0];
        end
    end
    
end

    // --------------------------
    // Read FIFO
    // --------------------------
    logic [1:0] rd_count;
    logic [2:0] valid_out;

    always_comb begin
        fu_pckt_out = '{default: 0};
        unique case (rd_EN)
            3'b001: fu_pckt_out[0]     = fifo_entries_next[0];
            3'b010: fu_pckt_out[1]     = fifo_entries_next[0];
            3'b011: fu_pckt_out[1:0]   = fifo_entries_next[1:0];
            3'b100: fu_pckt_out[2]     = fifo_entries_next[0];
            3'b101: begin
                        fu_pckt_out[0] = fifo_entries_next[0];
                        fu_pckt_out[2] = fifo_entries_next[1];
                    end
            3'b110: fu_pckt_out[2:1]   = fifo_entries_next[1:0];
            3'b111: fu_pckt_out        = fifo_entries_next[2:0];
            default: fu_pckt_out       = '{default: 0};
        endcase
    end

    always_comb begin
        for (int i = 0; i < 3; i++) begin
            valid_out[i] = fu_pckt_out[i].valid;
        end
    end

assign rd_count = valid_out[0] + valid_out[1] + valid_out[2];

    // --------------------------
    // FIFO Shift
    // --------------------------
    ISSUE_FU_PACKET [FIFO_DEPTH-1:0] fifo_entries_shifted;
    logic [$clog2(FIFO_DEPTH)-1:0]   tail_shifted;

    assign tail_shifted = (tail_next > rd_count) ? (tail_next - rd_count) : 0;

    always_comb begin
        fifo_entries_shifted = '{default: 0};
        for (int i = 0; i < FIFO_DEPTH; i++) begin
            case (rd_count)
                2'd3: fifo_entries_shifted[i] = fifo_entries_next[i+3];
                2'd2: fifo_entries_shifted[i] = fifo_entries_next[i+2];
                2'd1: fifo_entries_shifted[i] = fifo_entries_next[i+1];
                default: fifo_entries_shifted[i] = fifo_entries_next[i];
            endcase
        end
    end

    // --------------------------
    // Sync Update
    // --------------------------
    always_ff @(posedge clock) begin
        if (reset) begin
            fifo_entries <= `SD '{default: 0};
            tail         <= `SD '0;
        end else begin
            fifo_entries <= `SD fifo_entries_shifted;
            tail         <= `SD tail_shifted;
        end
    end

endmodule

`endif

