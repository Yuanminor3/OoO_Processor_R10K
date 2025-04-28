`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module issue_stage_tb;

  // Clock and reset
  logic clock;
  logic reset;

  // Inputs
  RS_S_PACKET [2:0] is_packet_in;
  ISSUE_FU_PACKET [2:0] issue_fu_packet;
  FU_STATE_PACKET fu_ready_is;

  // Outputs
  FU_FIFO_PACKET fu_fifo_stall;
  ISSUE_FU_PACKET [2**`FU-1:0] is_fu_packet;

  // Instantiate DUT (Device Under Test)
  issue_stage dut (
    .clock(clock),
    .reset(reset),
    .is_packet_in(is_packet_in),
    .issue_fu_packet(issue_fu_packet),
    .fu_ready_is(fu_ready_is),
    .fu_fifo_stall(fu_fifo_stall),
    .is_fu_packet(is_fu_packet)
  );

  // Clock generation
  initial begin
    clock = 0;
    forever #5 clock = ~clock;  // 100MHz clock
  end

  // Test process
  initial begin
    // Initialize
    reset = 1;
    is_packet_in = '{default: 0};
    issue_fu_packet = '{default: 0};
    fu_ready_is = '{default: 0};
    @(posedge clock);
    @(posedge clock);
    reset = 0;

    // Step 1: Input some packets
    is_packet_in[0].fu_sel = ALU_1;
    is_packet_in[1].fu_sel = MULT_1;
    is_packet_in[2].fu_sel = LS_1;

    issue_fu_packet[0].valid = 1;
    issue_fu_packet[0].PC = 32'hAAAA_AAAA;

    issue_fu_packet[1].valid = 1;
    issue_fu_packet[1].PC = 32'hBBBB_BBBB;

    issue_fu_packet[2].valid = 1;
    issue_fu_packet[2].PC = 32'hCCCC_CCCC;

    // FU is ready to accept
    fu_ready_is.alu_1 = 1;
    fu_ready_is.mult_1 = 1;
    fu_ready_is.loadstore_1 = 1;

    @(posedge clock);
    @(posedge clock);

    // Step 2: Check outputs
    if (is_fu_packet[ALU_1].valid !== 1 || is_fu_packet[ALU_1].PC !== 32'hAAAA_AAAA) begin
      $display("FAILED: ALU_1 output incorrect");
      $finish;
    end
    if (is_fu_packet[MULT_1].valid !== 1 || is_fu_packet[MULT_1].PC !== 32'hBBBB_BBBB) begin
      $display("FAILED: MULT_1 output incorrect");
      $finish;
    end
    if (is_fu_packet[LS_1].valid !== 1 || is_fu_packet[LS_1].PC !== 32'hCCCC_CCCC) begin
      $display("FAILED: LS_1 output incorrect");
      $finish;
    end

    // Step 3: If all passed
    $display("\nPASSED\n");
    $finish;
  end

endmodule

