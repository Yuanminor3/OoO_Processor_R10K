`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module issue_stage_tb;

  // Clock and rst
  logic clk;
  logic rst;

  // Inputs
  RS_S_PACKET [2:0] iss_rs_in_pkts;
  ISSUE_FU_PACKET [2:0] iss_issue_in_pkts;
  FU_STATE_PACKET fu_ready_is;

  // Outputs
  FU_FIFO_PACKET fu_fifo_stall;
  ISSUE_FU_PACKET [2**`SYS_FU_ADDR_WIDTH-1:0] iss_issued_fu_pkts;

  // Instantiate DUT (Device Under Test)
  issue_stage dut (
    .clk(clk),
    .rst(rst),
    .iss_rs_in_pkts(iss_rs_in_pkts),
    .iss_issue_in_pkts(iss_issue_in_pkts),
    .fu_ready_is(fu_ready_is),
    .fu_fifo_stall(fu_fifo_stall),
    .iss_issued_fu_pkts(iss_issued_fu_pkts)
  );

  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 100MHz clk
  end

  // Test process
  initial begin
    // Initialize
    rst = 1;
    iss_rs_in_pkts = '{default: 0};
    iss_issue_in_pkts = '{default: 0};
    fu_ready_is = '{default: 0};
    @(posedge clk);
    @(posedge clk);
    rst = 0;

    // Step 1: Input some packets
    iss_rs_in_pkts[0].dec_fu_unit_sel = ALU_1;
    iss_rs_in_pkts[1].dec_fu_unit_sel = MULT_1;
    iss_rs_in_pkts[2].dec_fu_unit_sel = LS_1;

    iss_issue_in_pkts[0].valid = 1;
    iss_issue_in_pkts[0].PC = 32'hAAAA_AAAA;

    iss_issue_in_pkts[1].valid = 1;
    iss_issue_in_pkts[1].PC = 32'hBBBB_BBBB;

    iss_issue_in_pkts[2].valid = 1;
    iss_issue_in_pkts[2].PC = 32'hCCCC_CCCC;

    // SYS_FU_ADDR_WIDTH is ready to accept
    fu_ready_is.alu_1 = 1;
    fu_ready_is.mult_1 = 1;
    fu_ready_is.loadstore_1 = 1;

    @(posedge clk);
    @(posedge clk);

    // Step 2: Check outputs
    if (iss_issued_fu_pkts[ALU_1].valid !== 1 || iss_issued_fu_pkts[ALU_1].PC !== 32'hAAAA_AAAA) begin
      $display("FAILED: ALU_1 output incorrect");
      $finish;
    end
    if (iss_issued_fu_pkts[MULT_1].valid !== 1 || iss_issued_fu_pkts[MULT_1].PC !== 32'hBBBB_BBBB) begin
      $display("FAILED: MULT_1 output incorrect");
      $finish;
    end
    if (iss_issued_fu_pkts[LS_1].valid !== 1 || iss_issued_fu_pkts[LS_1].PC !== 32'hCCCC_CCCC) begin
      $display("FAILED: LS_1 output incorrect");
      $finish;
    end

    // Step 3: If all passed
    $display("\nPASSED\n");
    $finish;
  end

endmodule

