
`ifndef __ISSUE_V__
`define __ISSUE_V__
`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module issue_stage(
    input                               clk,
    input                               rst,
    input       RS_S_PACKET [2:0]       iss_rs_in_pkts,
    input       ISSUE_FU_PACKET [2:0]   iss_issue_in_pkts,
    input       FU_STATE_PACKET 	fu_ready_is,
    output      FU_FIFO_PACKET		fu_fifo_stall,
    output      ISSUE_FU_PACKET [2**`SYS_FU_ADDR_WIDTH-1:0] iss_issued_fu_pkts
);

// assign packet to fu fifo
ISSUE_FU_PACKET [2:0]   iss_alu_fifo_push_pkts;
ISSUE_FU_PACKET [2:0]   iss_mult_fifo_push_pkts;
ISSUE_FU_PACKET [2:0]   iss_ls_fifo_push_pkts;
ISSUE_FU_PACKET [2:0]   iss_br_fifo_push_pkts;
always_comb begin
    iss_alu_fifo_push_pkts = 0;
    iss_mult_fifo_push_pkts = 0;
    iss_ls_fifo_push_pkts = 0;
    iss_br_fifo_push_pkts = 0;
    for(int i=0; i<3; i++) begin
        iss_alu_fifo_push_pkts[i] = iss_rs_in_pkts[i].dec_fu_unit_sel == ALU_1 ? iss_issue_in_pkts[i]:0;
        iss_mult_fifo_push_pkts[i] = iss_rs_in_pkts[i].dec_fu_unit_sel == MULT_1 ? iss_issue_in_pkts[i]:0;
        iss_ls_fifo_push_pkts[i] = iss_rs_in_pkts[i].dec_fu_unit_sel == LS_1 ? iss_issue_in_pkts[i]:0;
        iss_br_fifo_push_pkts[i] = iss_rs_in_pkts[i].dec_fu_unit_sel == BRANCH ? iss_issue_in_pkts[i]:0;
    end
end
ISSUE_FU_PACKET [3:0] iss_fifo_drop_pkts;

fu_FIFO_3 alu_fifo(
    .clk(clk),
    .rst(rst),
    .fu_pckt_in(iss_alu_fifo_push_pkts),
    .fifo3_read_en_mask({fu_ready_is.alu_1, fu_ready_is.alu_2, fu_ready_is.alu_3}),
    .fifo3_almost_full(fu_fifo_stall.alu),
    .fu_pckt_out({iss_issued_fu_pkts[ALU_1], iss_issued_fu_pkts[ALU_2], iss_issued_fu_pkts[ALU_3]})
   
);

fu_FIFO_3 ls_fifo(
    .clk(clk),
    .rst(rst ),
    .fu_pckt_in(iss_ls_fifo_push_pkts),
    .fifo3_read_en_mask({fu_ready_is.loadstore_1, fu_ready_is.loadstore_2, 1'b0}),
    .fifo3_almost_full(fu_fifo_stall.ls),
    .fu_pckt_out({iss_issued_fu_pkts[LS_1], iss_issued_fu_pkts[LS_2], iss_fifo_drop_pkts[0]})

);

fu_FIFO_3 mult_fifo(
    .clk(clk),
    .rst(rst ),
    .fu_pckt_in(iss_mult_fifo_push_pkts),
    .fifo3_read_en_mask({fu_ready_is.mult_1, fu_ready_is.mult_2, 1'b0}),
    .fifo3_almost_full(fu_fifo_stall.mult),
    .fu_pckt_out({iss_issued_fu_pkts[MULT_1], iss_issued_fu_pkts[MULT_2], iss_fifo_drop_pkts[1]})

);

fu_FIFO_3 br_fifo(
    .clk(clk),
    .rst(rst ),
    .fu_pckt_in(iss_br_fifo_push_pkts),
    .fifo3_read_en_mask({fu_ready_is.branch, 2'b0}),
    .fifo3_almost_full(fu_fifo_stall.branch),
    .fu_pckt_out({iss_issued_fu_pkts[BRANCH], iss_fifo_drop_pkts[3:2]})

);

endmodule

module fu_FIFO_3 #(parameter FIFO_DEPTH=`SYS_IS_FIFO_DEPTH)(
    input                       clk,
    input                       rst,
    input ISSUE_FU_PACKET[2:0]  fu_pckt_in,
    input [2:0]                 fifo3_read_en_mask,
    output logic                fifo3_almost_full,
    output ISSUE_FU_PACKET[2:0] fu_pckt_out

);
/* sync registers */
ISSUE_FU_PACKET [FIFO_DEPTH-1:0] fifo3_entry;
logic [$clog2(FIFO_DEPTH)-1:0] fl_tail_reg;

/* write count and EN */
logic [2:0] fifo3_write_en_mask;
logic [1:0]     fifo3_wr_cnt;
always_comb begin
    for(int i=0; i<3; i++) begin
        fifo3_write_en_mask[i] = fu_pckt_in[i].valid;
    end
end
assign fifo3_wr_cnt = fifo3_write_en_mask[0] + fifo3_write_en_mask[1] + fifo3_write_en_mask[2];

    // --------------------------
    // Reorder input
    // --------------------------
    ISSUE_FU_PACKET [2:0] fifo3_reordered;

    always_comb begin
        fifo3_reordered = '{default: 0};
        unique case (fifo3_write_en_mask)
            3'b001: fifo3_reordered[2]       = fu_pckt_in[0];
            3'b010: fifo3_reordered[2]       = fu_pckt_in[1];
            3'b011: fifo3_reordered[2:1]     = fu_pckt_in[1:0];
            3'b100: fifo3_reordered[2]       = fu_pckt_in[2];
            3'b101: begin
                        fifo3_reordered[2]   = fu_pckt_in[2];
                        fifo3_reordered[1]   = fu_pckt_in[0];
                    end
            3'b110: fifo3_reordered[2:1]     = fu_pckt_in[2:1];
            3'b111: fifo3_reordered          = fu_pckt_in;
            default: fifo3_reordered         = '{default: 0};
        endcase
    end

    // --------------------------
    // Tail Calculation
    // --------------------------
    logic [$clog2(FIFO_DEPTH):0] fl_tail_nxt;

    assign fl_tail_nxt    = fl_tail_reg + fifo3_wr_cnt;
    //assign fl_buffer_full         = (fl_tail_nxt >= FIFO_DEPTH);
    assign fifo3_almost_full  = (fl_tail_nxt + 3 >= FIFO_DEPTH);

    // --------------------------
    // Write to FIFO (tentative)
    // --------------------------
ISSUE_FU_PACKET [FIFO_DEPTH-1+3:0] fifo3_entry_nxt;
always_comb begin
    fifo3_entry_nxt = 0;
    fifo3_entry_nxt[FIFO_DEPTH-1:0] = fifo3_entry;
    for(int i=0; i<FIFO_DEPTH; i++) begin
        if (i == fl_tail_reg) begin
            fifo3_entry_nxt[i] = fifo3_reordered[2];
            fifo3_entry_nxt[i+1] = fifo3_reordered[1];
            fifo3_entry_nxt[i+2] = fifo3_reordered[0];
        end
    end
    
end

    // --------------------------
    // Read FIFO
    // --------------------------
    logic [1:0] fifo3_rd_cnt;
    logic [2:0] fifo3_valid_mask;

    always_comb begin
        fu_pckt_out = '{default: 0};
        unique case (fifo3_read_en_mask)
            3'b001: fu_pckt_out[0]     = fifo3_entry_nxt[0];
            3'b010: fu_pckt_out[1]     = fifo3_entry_nxt[0];
            3'b011: fu_pckt_out[1:0]   = fifo3_entry_nxt[1:0];
            3'b100: fu_pckt_out[2]     = fifo3_entry_nxt[0];
            3'b101: begin
                        fu_pckt_out[0] = fifo3_entry_nxt[0];
                        fu_pckt_out[2] = fifo3_entry_nxt[1];
                    end
            3'b110: fu_pckt_out[2:1]   = fifo3_entry_nxt[1:0];
            3'b111: fu_pckt_out        = fifo3_entry_nxt[2:0];
            default: fu_pckt_out       = '{default: 0};
        endcase
    end

    always_comb begin
        for (int i = 0; i < 3; i++) begin
            fifo3_valid_mask[i] = fu_pckt_out[i].valid;
        end
    end

assign fifo3_rd_cnt = fifo3_valid_mask[0] + fifo3_valid_mask[1] + fifo3_valid_mask[2];

    // --------------------------
    // FIFO Shift
    // --------------------------
    ISSUE_FU_PACKET [FIFO_DEPTH-1:0] fifo_entries_shifted;
    logic [$clog2(FIFO_DEPTH)-1:0]   fifo3_shift_tail;

    assign fifo3_shift_tail = (fl_tail_nxt > fifo3_rd_cnt) ? (fl_tail_nxt - fifo3_rd_cnt) : 0;

    always_comb begin
        fifo_entries_shifted = '{default: 0};
        for (int i = 0; i < FIFO_DEPTH; i++) begin
            case (fifo3_rd_cnt)
                2'd3: fifo_entries_shifted[i] = fifo3_entry_nxt[i+3];
                2'd2: fifo_entries_shifted[i] = fifo3_entry_nxt[i+2];
                2'd1: fifo_entries_shifted[i] = fifo3_entry_nxt[i+1];
                default: fifo_entries_shifted[i] = fifo3_entry_nxt[i];
            endcase
        end
    end

    // --------------------------
    // Sync Update
    // --------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            fifo3_entry <= `SYS_SMALL_DELAY '{default: 0};
            fl_tail_reg         <= `SYS_SMALL_DELAY '0;
        end else begin
            fifo3_entry <= `SYS_SMALL_DELAY fifo_entries_shifted;
            fl_tail_reg         <= `SYS_SMALL_DELAY fifo3_shift_tail;
        end
    end

endmodule

`endif

