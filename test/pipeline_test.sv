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
import "DPI-C" function int fl_new_pr2(int dispatch_pr_allocEN);
import "DPI-C" function int fl_new_pr1(int dispatch_pr_allocEN);
import "DPI-C" function int fl_new_pr0(int dispatch_pr_allocEN);
import "DPI-C" function int fl_pop(int dispatch_pr_allocEN);

/* map table simulator */ 
import "DPI-C" function void mt_init();
import "DPI-C" function int mt_look_up(int i);
import "DPI-C" function int mt_look_up_ready(int i);
import "DPI-C" function void mt_map(int ar, int pr);

/* print pipeline */
import "DPI-C" function void print_header(string str);
import "DPI-C" function void print_num(int num);
import "DPI-C" function void print_stage(string div, int inst, int npc, int dec_valid_flag);

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
logic clk, rst;
logic program_halt;
logic [2:0] retired_inst_cnt;

/* ====== Debug and Display Signals ====== */
`ifdef TEST_MODE

// Data Cache
logic [1:0][15:0][63:0]       dc_dbg_data;
logic [1:0][15:0][8:0]        dc_dbg_tag;
logic [1:0][15:0]             dc_dbg_valid;
logic [1:0][15:0]             dirties;

MHSRS_ENTRY_PACKET [`SYS_MHSRS_NUM-1:0] mhsrs_entry_vector;
logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_head_index;
logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_issue_index;
logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_tail_index;

`endif


/* ====== Cache Simulation Signals ====== */
SQ_ENTRY_PACKET [2:0]       cache_wb_sim;
logic [1:0][`SYS_XLEN-1:0]      cache_read_addr_sim, cache_read_data_sim;
logic [1:0]                 cache_read_start_sim;

/* ====== Memory Interface Signals ====== */
logic  [3:0] icache_mem_resp_code;
logic [63:0] icache_mem_resp_data;
logic  [3:0] icache_mem_resp_id;
logic [`SYS_XLEN-1:0] icache_mem_req_addr;
logic [1:0] icache_mem_req_cmd;
logic [63:0] proc2Imem_data;

/* ====== Other Control Signals ====== */
logic [63:0] debug_counter;
EXCEPTION_CODE pipeline_error_status;
string program_memory_file;
string writeback_output_file;

/* ===== Instantiate Memory ===== */
mem memory(
    .clk(clk),
    .mem_req_addr(icache_mem_req_addr),
    .mem_req_data(proc2Imem_data),
    .mem_req_cmd(icache_mem_req_cmd),
    .mem_resp_code(icache_mem_resp_code),
    .mem_resp_data(icache_mem_resp_data),
    .mem_resp_id(icache_mem_resp_id)
);

/* ===== Instantiate DUT: Core Pipeline ===== */
pipeline core(
    .clk(clk),
    .rst(rst),
    .mem_resp_code(icache_mem_resp_code),
    .mem_resp_data(icache_mem_resp_data),
    .mem_resp_id(icache_mem_resp_id),
    .mem_req_cmd(icache_mem_req_cmd),
    .mem_req_addr(icache_mem_req_addr),
    .mem_req_data(proc2Imem_data),
    .halt(program_halt),
    .retired_inst_cnt(retired_inst_cnt)

`ifdef TEST_MODE

    // Data Cache
    , .dc_dbg_data(dc_dbg_data)
    , .dc_dbg_tag(dc_dbg_tag)
    , .dc_dbg_valid(dc_dbg_valid)
    , .dc_dbg_dir(dirties)
    , .mhsrs_entry_vector(mhsrs_entry_vector)
    , .mhsrs_head_index(mhsrs_head_index)
    , .mhsrs_issue_index(mhsrs_issue_index)
    , .mhsrs_tail_index(mhsrs_tail_index)
`endif 

);

/* ===== Clock Generation ===== */
always begin
    #(`SYS_CLK/2.0);
    clk = ~clk;
end

/* ===== Wait Until Pipeline Halts ===== */
task wait_until_halt;
    forever begin : wait_loop
        @(posedge program_halt);
        @(negedge clk);
        if (program_halt) begin 
            @(negedge clk);
            disable wait_until_halt;
        end
    end
endtask

/* ====== Cycle Counter ====== */
int cycle_count;
always @(posedge clk) begin
    if (rst) 
        cycle_count <= `SYS_SMALL_DELAY 0;
    else 
        cycle_count <= `SYS_SMALL_DELAY cycle_count + 1;
end

/* ====== Instruction Counter and Halt Detection ====== */
int inst_total;
logic halted;
always @(posedge clk) begin
    if (rst)
        halted <= `SYS_SMALL_DELAY 0;
    else if (~halted)
        halted <= `SYS_SMALL_DELAY program_halt;
    else
        halted <= `SYS_SMALL_DELAY 1;
end

always @(negedge clk) begin
    if (rst)
        inst_total = 0;
    else if (~halted)
        inst_total = inst_total + retired_inst_cnt[0] + retired_inst_cnt[1] + retired_inst_cnt[2];
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
                 cycle_count * `SYS_CLK);
    end
endtask

/* ====== Display Dcache Contents ====== */
task show_dcache;
    begin
        $display("=====   Cache ram   =====");
        $display("|  Entry(idx) |      Tag |             data |");
        for (int i = 0; i < 32; ++i) begin
            $display("| %d | %b | %h |", i, dc_dbg_tag[i], dc_dbg_data[i]);
        end
        $display("-------------------------------------------------");
    end
endtask

/* ====== Display SYS_MHSRS_ADDR_WIDTH State ====== */
task show_MHSRS;
    begin
        $display("=====   SYS_MHSRS_ADDR_WIDTH   =====");
        $display("fl_head_reg: %d, issue: %d, fl_tail_reg: %d", mhsrs_head_index, mhsrs_issue_index, mhsrs_tail_index);
        $display("|         No. |                              addr  |command|mem_tag|left_or_right|            data |issued| usedbytes | dirty |");
        for (int i = 0; i < 16; i++) begin
            $display("| %d |  %b  |     %d |    %d |           %b | %h | %b | %b |  %b |",
                     i, mhsrs_entry_vector[i].addr, mhsrs_entry_vector[i].command, mhsrs_entry_vector[i].mem_tag, 
                     mhsrs_entry_vector[i].left_or_right, mhsrs_entry_vector[i].data, mhsrs_entry_vector[i].issued, 
                     mhsrs_entry_vector[i].usebytes, mhsrs_entry_vector[i].dirty);
        end
        $display("----------------------------------------------------------------- ");
    end
endtask

/* ====== Show Memory with Decimal ====== */
task show_mem_with_decimal;
    input [31:0] start_addr;
    input [31:0] end_addr;
    int showing_data;
    logic [63:0] memory_final [`SYS_MEM_64B - 1:0];
    begin
        // Copy current memory and cache contents
        memory_final = memory.unified_memory;

        for (int i = 0; i < 16; i++) begin
            for (int w = 0; w < 2; w++) begin
                if (dc_dbg_valid[w][i]) begin
                    memory_final[{dc_dbg_tag[w][i], i[3:0]}] = dc_dbg_data[w][i];
                end
            end
        end

        // Write back all stores from SYS_MHSRS_ADDR_WIDTH
        if (mhsrs_head_index <= mhsrs_tail_index) begin
            for (int i = mhsrs_head_index; i < mhsrs_tail_index; i = i + 1) begin
                if (mhsrs_entry_vector[i].command == BUS_STORE) begin
                    if (mhsrs_entry_vector[i].left_or_right)
                        memory_final[mhsrs_entry_vector[i].addr >> 3][63:32] = mhsrs_entry_vector[i].data[63:32];
                    else
                        memory_final[mhsrs_entry_vector[i].addr >> 3][31:0] = mhsrs_entry_vector[i].data[31:0];
                end else if (mhsrs_entry_vector[i].command == BUS_LOAD && mhsrs_entry_vector[i].dirty) begin
                    for (int j = 0; j < 8; j = j + 1) begin
                        if (mhsrs_entry_vector[i].usebytes[j]) begin
                            memory_final[mhsrs_entry_vector[i].addr >> 3][8*j+7 -: 8] = mhsrs_entry_vector[i].data[8*j+7 -: 8];
                        end
                    end
                end
            end
        end else begin
            for (int i = mhsrs_head_index; i <= `SYS_MHSRS_NUM-1; i = i + 1) begin
                if (mhsrs_entry_vector[i].command == BUS_STORE) begin
                    if (mhsrs_entry_vector[i].left_or_right)
                        memory_final[mhsrs_entry_vector[i].addr >> 3][63:32] = mhsrs_entry_vector[i].data[63:32];
                    else
                        memory_final[mhsrs_entry_vector[i].addr >> 3][31:0] = mhsrs_entry_vector[i].data[31:0];
                end else if (mhsrs_entry_vector[i].command == BUS_LOAD && mhsrs_entry_vector[i].dirty) begin
                    for (int j = 0; j < 8; j = j + 1) begin
                        if (mhsrs_entry_vector[i].usebytes[j]) begin
                            memory_final[mhsrs_entry_vector[i].addr >> 3][8*j+7 -: 8] = mhsrs_entry_vector[i].data[8*j+7 -: 8];
                        end
                    end
                end
            end
            for (int i = 0; i < mhsrs_tail_index; i = i + 1) begin
                if (mhsrs_entry_vector[i].command == BUS_STORE) begin
                    if (mhsrs_entry_vector[i].left_or_right)
                        memory_final[mhsrs_entry_vector[i].addr >> 3][63:32] = mhsrs_entry_vector[i].data[63:32];
                    else
                        memory_final[mhsrs_entry_vector[i].addr >> 3][31:0] = mhsrs_entry_vector[i].data[31:0];
                end else if (mhsrs_entry_vector[i].command == BUS_LOAD && mhsrs_entry_vector[i].dirty) begin
                    for (int j = 0; j < 8; j = j + 1) begin
                        if (mhsrs_entry_vector[i].usebytes[j]) begin
                            memory_final[mhsrs_entry_vector[i].addr >> 3][8*j+7 -: 8] = mhsrs_entry_vector[i].data[8*j+7 -: 8];
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
always @(negedge clk) begin
    if (rst) begin
        $display("@@\n@@  %t : System STILL at rst, can't show anything\n@@", $realtime);
        debug_counter <= 0;
    end else begin
        `SYS_SMALL_DELAY;
        `SYS_SMALL_DELAY;

        // Handle halt conditions
        if (pipeline_error_status != NO_ERROR || debug_counter > `MAX_CYCLE) begin
            $display("@@@ Unified Memory contents hex on left, decimal on right: ");
            show_mem_with_decimal(0, `SYS_MEM_64B - 1); // 8 Bytes per line, 16kB total

            $display("@@  %t : System halted\n@@", $realtime);
            
            case (pipeline_error_status)
                LOAD_ACCESS_FAULT:  
                    $display("@@@ System halted on memory error");
                HALTED_ON_WFI:          
                    $display("@@@ System halted on WFI instruction");
                ILLEGAL_INST:
                    $display("@@@ System halted on dec_illegal_flag instruction");
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
    clk = 1'b0;
    rst = 1'b0;
    pipeline_error_status = NO_ERROR;
    debug_counter = 0;

    // Assert rst
    $display("@@\n@@\n@@  %t  Asserting System rst......", $realtime);
    rst = 1'b1;

    @(posedge clk);
    @(posedge clk);

    // Load memory contents
    $readmemh(program_memory_file, memory.unified_memory);

    @(posedge clk);
    @(posedge clk);
    #1;
    
    // Deassert rst
    rst = 1'b0;
    $display("@@  %t  Deasserting System rst......\n@@\n@@", $realtime);

    @(negedge clk);
    wait_until_halt;

    pipeline_error_status = HALTED_ON_WFI;
end

endmodule
`endif // __TESTBENCH_FIN_SV__
