`timescale 1ns/100ps
`ifndef __TESTBENCH_FIN_SV__
`define __TESTBENCH_FIN_SV__

/* ====== Global Defines ====== */
`define MAX_CYCLE 5000000
`define TEST_MODE 

/* ====== Import DPI-C Functions ====== */

/* freelist simulator */
import "DPI-C" function void fl_init();
import "DPI-C" function int fl_new_pr_valid();
import "DPI-C" function int fl_new_pr2(int new_pr_en);
import "DPI-C" function int fl_new_pr1(int new_pr_en);
import "DPI-C" function int fl_new_pr0(int new_pr_en);
import "DPI-C" function int fl_pop(int new_pr_en);

/* map table simulator */ 
import "DPI-C" function void mt_init();
import "DPI-C" function int mt_look_up(int i);
import "DPI-C" function int mt_look_up_ready(int i);
import "DPI-C" function void mt_map(int ar, int pr);

/* print pipeline */
import "DPI-C" function void print_header(string str);
import "DPI-C" function void print_num(int num);
import "DPI-C" function void print_stage(string div, int inst, int npc, int valid_inst);

/* print reservation station */
import "DPI-C" function void print_select(int index, int valid, int inst, int npc, int fu_select, int op_select);

/* simulate cache & memory */
import "DPI-C" function void mem_init();
import "DPI-C" function void mem_write(int addr, int data, int byte3, byte2, byte1, byte0);
import "DPI-C" function int mem_read(int addr);
import "DPI-C" function void mem_print();

/* memory operations */
import "DPI-C" function void mem_copy(int addr_int, int data_int);
import "DPI-C" function void mem_final_print();

`include "verilog/sys_defs.svh"

/* ====== Testbench Top Module ====== */
module testbench;

/* ====== Clock, Reset, Control ====== */
logic clock, reset;
logic program_halt;
logic [2:0] inst_count;

/* ====== Debug and Display Signals ====== */
`ifdef TEST_MODE

// Data Cache
logic [1:0][15:0][63:0]       cache_data_disp;
logic [1:0][15:0][8:0]        cache_tags_disp;
logic [1:0][15:0]             valids_disp;
logic [1:0][15:0]             dirties;

MHSRS_ENTRY_PACKET [`MHSRS_W-1:0] MHSRS_disp;
logic [`MHSRS-1:0] head_pointer;
logic [`MHSRS-1:0] issue_pointer;
logic [`MHSRS-1:0] tail_pointer;

`endif


/* ====== Cache Simulation Signals ====== */
SQ_ENTRY_PACKET [2:0]       cache_wb_sim;
logic [1:0][`XLEN-1:0]      cache_read_addr_sim, cache_read_data_sim;
logic [1:0]                 cache_read_start_sim;

/* ====== Memory Interface Signals ====== */
logic  [3:0] Imem2proc_response;
logic [63:0] Imem2proc_data;
logic  [3:0] Imem2proc_tag;
logic [`XLEN-1:0] proc2Imem_addr;
logic [1:0] proc2Imem_command;
logic [63:0] proc2Imem_data;

/* ====== Other Control Signals ====== */
logic [63:0] debug_counter;
EXCEPTION_CODE pipeline_error_status;
string program_memory_file;
string writeback_output_file;

/* ===== Instantiate Memory ===== */
mem memory(
    .clk(clock),
    .proc2mem_addr(proc2Imem_addr),
    .proc2mem_data(proc2Imem_data),
    .proc2mem_command(proc2Imem_command),
    .mem2proc_response(Imem2proc_response),
    .mem2proc_data(Imem2proc_data),
    .mem2proc_tag(Imem2proc_tag)
);

/* ===== Instantiate DUT: Core Pipeline ===== */
pipeline core(
    .clock(clock),
    .reset(reset),
    .mem2proc_response(Imem2proc_response),
    .mem2proc_data(Imem2proc_data),
    .mem2proc_tag(Imem2proc_tag),
    .proc2mem_command(proc2Imem_command),
    .proc2mem_addr(proc2Imem_addr),
    .proc2mem_data(proc2Imem_data),
    .halt(program_halt),
    .inst_count(inst_count)

`ifdef TEST_MODE

    // Data Cache
    , .cache_data_disp(cache_data_disp)
    , .cache_tags_disp(cache_tags_disp)
    , .valids_disp(valids_disp)
    , .dirties_disp(dirties)
    , .MHSRS_disp(MHSRS_disp)
    , .head_pointer(head_pointer)
    , .issue_pointer(issue_pointer)
    , .tail_pointer(tail_pointer)
`endif 

);

/* ===== Clock Generation ===== */
always begin
    #(`VERILOG_CLOCK_PERIOD/2.0);
    clock = ~clock;
end

/* ===== Wait Until Pipeline Halts ===== */
task wait_until_halt;
    forever begin : wait_loop
        @(posedge program_halt);
        @(negedge clock);
        if (program_halt) begin 
            @(negedge clock);
            disable wait_until_halt;
        end
    end
endtask

/* ====== Cycle Counter ====== */
int cycle_count;
always @(posedge clock) begin
    if (reset) 
        cycle_count <= `SD 0;
    else 
        cycle_count <= `SD cycle_count + 1;
end

/* ====== Instruction Counter and Halt Detection ====== */
int inst_total;
logic halted;
always @(posedge clock) begin
    if (reset)
        halted <= `SD 0;
    else if (~halted)
        halted <= `SD program_halt;
    else
        halted <= `SD 1;
end

always @(negedge clock) begin
    if (reset)
        inst_total = 0;
    else if (~halted)
        inst_total = inst_total + inst_count[0] + inst_count[1] + inst_count[2];
end

//////////////////////////////////////////////////////////////
//////////////                  DISPLAY
//////////////////////////////////////////////////////////////

/* ====== Display CPI ====== */
task print_cpi;
    real cpi;
    begin
        cpi = (cycle_count) / (inst_total - 1.0);
        $display("@@  %0d cycles / %0d instrs = %f CPI\n@@",
                 cycle_count, inst_total - 1, cpi);
        $display("@@  %4.2f ns total time to execute\n@@\n",
                 cycle_count * `VERILOG_CLOCK_PERIOD);
    end
endtask

/* ====== Display Dcache Contents ====== */
task show_dcache;
    begin
        $display("=====   Cache ram   =====");
        $display("|  Entry(idx) |      Tag |             data |");
        for (int i = 0; i < 32; ++i) begin
            $display("| %d | %b | %h |", i, cache_tags_disp[i], cache_data_disp[i]);
        end
        $display("-------------------------------------------------");
    end
endtask

/* ====== Display MHSRS State ====== */
task show_MHSRS;
    begin
        $display("=====   MHSRS   =====");
        $display("head: %d, issue: %d, tail: %d", head_pointer, issue_pointer, tail_pointer);
        $display("|         No. |                              addr  |command|mem_tag|left_or_right|            data |issued| usedbytes | dirty |");
        for (int i = 0; i < 16; i++) begin
            $display("| %d |  %b  |     %d |    %d |           %b | %h | %b | %b |  %b |",
                     i, MHSRS_disp[i].addr, MHSRS_disp[i].command, MHSRS_disp[i].mem_tag, 
                     MHSRS_disp[i].left_or_right, MHSRS_disp[i].data, MHSRS_disp[i].issued, 
                     MHSRS_disp[i].usebytes, MHSRS_disp[i].dirty);
        end
        $display("----------------------------------------------------------------- ");
    end
endtask

/* ====== Show Memory with Decimal ====== */
task show_mem_with_decimal;
    input [31:0] start_addr;
    input [31:0] end_addr;
    int showing_data;
    logic [63:0] memory_final [`MEM_64BIT_LINES - 1:0];
    begin
        // Copy current memory and cache contents
        memory_final = memory.unified_memory;

        for (int i = 0; i < 16; i++) begin
            for (int w = 0; w < 2; w++) begin
                if (valids_disp[w][i]) begin
                    memory_final[{cache_tags_disp[w][i], i[3:0]}] = cache_data_disp[w][i];
                end
            end
        end

        // Write back all stores from MHSRS
        if (head_pointer <= tail_pointer) begin
            for (int i = head_pointer; i < tail_pointer; i = i + 1) begin
                if (MHSRS_disp[i].command == BUS_STORE) begin
                    if (MHSRS_disp[i].left_or_right)
                        memory_final[MHSRS_disp[i].addr >> 3][63:32] = MHSRS_disp[i].data[63:32];
                    else
                        memory_final[MHSRS_disp[i].addr >> 3][31:0] = MHSRS_disp[i].data[31:0];
                end else if (MHSRS_disp[i].command == BUS_LOAD && MHSRS_disp[i].dirty) begin
                    for (int j = 0; j < 8; j = j + 1) begin
                        if (MHSRS_disp[i].usebytes[j]) begin
                            memory_final[MHSRS_disp[i].addr >> 3][8*j+7 -: 8] = MHSRS_disp[i].data[8*j+7 -: 8];
                        end
                    end
                end
            end
        end else begin
            for (int i = head_pointer; i <= `MHSRS_W-1; i = i + 1) begin
                if (MHSRS_disp[i].command == BUS_STORE) begin
                    if (MHSRS_disp[i].left_or_right)
                        memory_final[MHSRS_disp[i].addr >> 3][63:32] = MHSRS_disp[i].data[63:32];
                    else
                        memory_final[MHSRS_disp[i].addr >> 3][31:0] = MHSRS_disp[i].data[31:0];
                end else if (MHSRS_disp[i].command == BUS_LOAD && MHSRS_disp[i].dirty) begin
                    for (int j = 0; j < 8; j = j + 1) begin
                        if (MHSRS_disp[i].usebytes[j]) begin
                            memory_final[MHSRS_disp[i].addr >> 3][8*j+7 -: 8] = MHSRS_disp[i].data[8*j+7 -: 8];
                        end
                    end
                end
            end
            for (int i = 0; i < tail_pointer; i = i + 1) begin
                if (MHSRS_disp[i].command == BUS_STORE) begin
                    if (MHSRS_disp[i].left_or_right)
                        memory_final[MHSRS_disp[i].addr >> 3][63:32] = MHSRS_disp[i].data[63:32];
                    else
                        memory_final[MHSRS_disp[i].addr >> 3][31:0] = MHSRS_disp[i].data[31:0];
                end else if (MHSRS_disp[i].command == BUS_LOAD && MHSRS_disp[i].dirty) begin
                    for (int j = 0; j < 8; j = j + 1) begin
                        if (MHSRS_disp[i].usebytes[j]) begin
                            memory_final[MHSRS_disp[i].addr >> 3][8*j+7 -: 8] = MHSRS_disp[i].data[8*j+7 -: 8];
                        end
                    end
                end
            end
        end

        // Print non-zero memory lines
        $display("@@@");
        showing_data = 0;
        for (int k = start_addr; k <= end_addr; k = k + 1) begin
            if (memory_final[k] != 0) begin
                $display("@@@ mem[%5d] = %x : %0d", k*8, memory_final[k], memory_final[k]);
                showing_data = 1;
            end else if (showing_data != 0) begin
                $display("@@@");
                showing_data = 0;
            end
        end
        $display("@@@");
    end
endtask

/* ===== Debug and Halt Check ===== */
always @(negedge clock) begin
    if (reset) begin
        $display("@@\n@@  %t : System STILL at reset, can't show anything\n@@", $realtime);
        debug_counter <= 0;
    end else begin
        `SD;
        `SD;

        // Handle halt conditions
        if (pipeline_error_status != NO_ERROR || debug_counter > `MAX_CYCLE) begin
            $display("@@@ Unified Memory contents hex on left, decimal on right: ");
            show_mem_with_decimal(0, `MEM_64BIT_LINES - 1); // 8 Bytes per line, 16kB total

            $display("@@  %t : System halted\n@@", $realtime);
            
            case (pipeline_error_status)
                LOAD_ACCESS_FAULT:  
                    $display("@@@ System halted on memory error");
                HALTED_ON_WFI:          
                    $display("@@@ System halted on WFI instruction");
                ILLEGAL_INST:
                    $display("@@@ System halted on illegal instruction");
                default: 
                    $display("@@@ System halted on unknown error code %x", pipeline_error_status);
            endcase

            $display("@@@\n@@");
            print_cpi;
            #100 $finish;
        end

        debug_counter <= debug_counter + 1;
    end
end

/* ====== Initial Block: Memory Load and Reset Control ====== */
initial begin
    // Memory file loading
    if ($value$plusargs("MEMORY=%s", program_memory_file)) begin
        $display("Loading memory file: %s", program_memory_file);
    end else begin
        $display("Loading default memory file: program.mem");
        program_memory_file = "program.mem";
    end

    // Initial values
    clock = 1'b0;
    reset = 1'b0;
    pipeline_error_status = NO_ERROR;
    debug_counter = 0;

    // Assert reset
    $display("@@\n@@\n@@  %t  Asserting System reset......", $realtime);
    reset = 1'b1;

    @(posedge clock);
    @(posedge clock);

    // Load memory contents
    $readmemh(program_memory_file, memory.unified_memory);

    @(posedge clock);
    @(posedge clock);
    #1;
    
    // Deassert reset
    reset = 1'b0;
    $display("@@  %t  Deasserting System reset......\n@@\n@@", $realtime);

    @(negedge clock);
    wait_until_halt;

    pipeline_error_status = HALTED_ON_WFI;
end

endmodule
`endif // __TESTBENCH_FIN_SV__
