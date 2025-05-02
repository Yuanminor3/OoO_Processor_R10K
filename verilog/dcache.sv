//2-way 16-set Cache(256B) + Top Module: MSHRs

//----------------------2-Way Set Associative Cache Memory - 256B --------------------
//This module implements a 2-way set associative cache memory with 16 sets.
`define TEST_MODE
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
`timescale 1ns/100ps

module dcache_2way_16set(
    // Clock and rst
    input clk, 
    input rst,
    
    // Write Port 1 (from Store Queue hit)
    input  [2:0] dc_sq_wrEN,                   // Write enable for each port
    input  [2:0][3:0] dc_sq_wr_index,             // 4-bit index
    input  [2:0][8:0] dc_sq_wr_id,             // 9-bit tag
    input  [2:0][63:0] dc_sq_wr_data,           // Data to write
    input  [2:0][7:0] dc_sq_wr_be,          // Byte enable mask
    input  [2:0] dc_sq_wr_hit,                  // Indicates if write is a hit
    
    // Write Hit Check Port
    input  [2:0][3:0] dc_wrchk_index,             // 4-bit index
    input  [2:0][8:0] dc_wrchk_id,             // 9-bit tag
    output logic [2:0] dc_wrchk_hit,            // Hit result (1 if hit)
    
    // Read Ports (for Load instructions)
    input  [1:0][3:0] dc_Id_index,             // 4-bit index
    input  [1:0][8:0] dc_Id_id,             // 9-bit tag
    output logic [1:0][63:0] dc_Id_data,     // Read data output
    output logic [1:0] dc_Id_valid,          // Read valid (1 if hit)
    
    // Write-back Interface
    output logic dc_wb_valid,           // 1 if eviction requires write-back
    output logic [63:0] dc_wb_data,       // Data to write back to memory
    output logic [`SYS_XLEN-1:0] dc_wb_addr,  // Address to write back
    
    // Cache Refill Port (from Memory)
    input        dc_refillEN,                   // Refill enable
    input  [3:0] dc_refill_index,                  // 4-bit index
    input  [8:0] dc_refill_id,                  // 9-bit tag
    input  [63:0] dc_refill_data,                // Refill data
    input  [7:0] dc_refill_be,             // Refill byte mask
    input        dc_refill_dir,                // Refill dirty flag
    
    // Debug Ports (for testing only)
    `ifdef TEST_MODE
    output logic [1:0][15:0][63:0] dc_dbg_data,  // Cache data per way
    output logic [1:0][15:0][8:0]  dc_dbg_tag,  // 9-bit tags per way
    output logic [1:0][15:0]       dc_dbg_valid,      // Valid bits per way
    output logic [1:0][15:0]       dc_dbg_dir      // Dirty bits per way
    `endif 
);

    // =============================================
    // Parameters and Internal Storage
    // =============================================
    parameter NUM_WAYS = 2;  // 2-way set associative
    parameter NUM_SETS = 16; // 16 sets (index width 4 bits)
    
    // Cache storage arrays (2 ways)
    logic [NUM_WAYS-1:0][NUM_SETS-1:0][63:0] ld_cache_fetched_data;     // Data storage
    logic [NUM_WAYS-1:0][NUM_SETS-1:0][8:0]  cache_tags;     // 9-bit tag storage
    logic [NUM_WAYS-1:0][NUM_SETS-1:0]       valids;         // Valid bits
    logic [NUM_WAYS-1:0][NUM_SETS-1:0]       dirties;        // Dirty bits
    logic [NUM_SETS-1:0]                     lru_bits;       // LRU tracking (1 bit per set)
    
    // Next-state variables
    logic [NUM_WAYS-1:0][NUM_SETS-1:0][63:0] next_cache_data;
    logic [NUM_WAYS-1:0][NUM_SETS-1:0][8:0]  next_cache_tags;
    logic [NUM_WAYS-1:0][NUM_SETS-1:0]       next_valids;
    logic [NUM_WAYS-1:0][NUM_SETS-1:0]       next_dirties;
    logic [NUM_SETS-1:0]                     next_lru_bits;

    // Temporary variables
    int hit_way;        // Way that hit for current operation
    int invalid_way;    // First invalid way found
    int replacement_way; // Way selected for replacement

    // =============================================
    // Debug Assignments (TEST_MODE only)
    // =============================================
    `ifdef TEST_MODE
    assign dc_dbg_data = ld_cache_fetched_data;
    assign dc_dbg_tag = cache_tags;
    assign dc_dbg_valid = valids;
    `endif
    
    // =============================================
    // Read Port Logic
    // =============================================
    always_comb begin
        for (int port = 0; port < 2; port++) begin
            dc_Id_valid[port] = 0;
            dc_Id_data[port] = 0;
            for (int way = 0; way < NUM_WAYS; way++) begin
                if (valids[way][dc_Id_index[port]] && 
                    cache_tags[way][dc_Id_index[port]] == dc_Id_id[port]) begin
                    dc_Id_valid[port] = 1;
                    dc_Id_data[port] = ld_cache_fetched_data[way][dc_Id_index[port]];
                end
            end
        end
    end
    
    // =============================================
    // Write Hit Check Logic
    // =============================================
    always_comb begin
        for (int port = 0; port < 3; port++) begin
            dc_wrchk_hit[port] = 0;
            for (int way = 0; way < NUM_WAYS; way++) begin
                if (valids[way][dc_wrchk_index[port]] && 
                    cache_tags[way][dc_wrchk_index[port]] == dc_wrchk_id[port]) begin
                    dc_wrchk_hit[port] = 1;
                end
            end
        end
    end
    
    // =============================================
    // Cache Update Logic
    // =============================================
    always_comb begin
        // Default: keep current values
        next_cache_data = ld_cache_fetched_data;
        next_cache_tags = cache_tags;
        next_valids = valids;
        next_dirties = dirties;
        next_lru_bits = lru_bits;
        
        // Initialize write-back signals
        dc_wb_valid = 0;
        dc_wb_data = 0;
        dc_wb_addr = 0;
        
        // -----------------------------
        // Process Store Queue Writes (wr1)
        // -----------------------------
        for (int port = 0; port < 3; port++) begin
            if (dc_sq_wrEN[port] && dc_sq_wr_hit[port]) begin
                // Find which way hit
                for (int way = 0; way < NUM_WAYS; way++) begin
                    if (valids[way][dc_sq_wr_index[port]] && 
                        cache_tags[way][dc_sq_wr_index[port]] == dc_sq_wr_id[port]) begin
                        hit_way = way;
                    end
                end
                
                // Update the hit way
                next_valids[hit_way][dc_sq_wr_index[port]] = 1'b1;
                next_cache_tags[hit_way][dc_sq_wr_index[port]] = dc_sq_wr_id[port];
                next_dirties[hit_way][dc_sq_wr_index[port]] = 1'b1;

                // Update data based on byte mask
                for (int bt = 0; bt < 8; bt++) begin
                    if (dc_sq_wr_be[port][bt]) begin
                        next_cache_data[hit_way][dc_sq_wr_index[port]][8*bt +: 8] = 
                            dc_sq_wr_data[port][8*bt +: 8];
                    end
                end
                
                // Update LRU (mark other way as recently used)
                next_lru_bits[dc_sq_wr_index[port]] = ~hit_way;
            end
        end
        
        // -----------------------------
        // Process Cache Refill (wr2)
        // -----------------------------
        if (dc_refillEN) begin
            invalid_way = -1;
            hit_way = -1;
            
            // Check for existing entry (hit)
            for (int way = 0; way < NUM_WAYS; way++) begin
                if (valids[way][dc_refill_index] && cache_tags[way][dc_refill_index] == dc_refill_id) begin 
                    hit_way = way;
                end
            end
            
            // Find first invalid way if no hit
            if (hit_way == -1) begin
                for (int way = 0; way < NUM_WAYS; way++) begin
                    if (!valids[way][dc_refill_index]) begin
                        invalid_way = way;
                        break;
                    end
                end
            end
            
            // Determine which way to use
            if (hit_way != -1) begin
                // Case 1: Hit - update existing entry
                replacement_way = hit_way;
                if (dc_refill_dir) begin
                    next_dirties[replacement_way][dc_refill_index] = 1'b1;
                    for (int bt = 0; bt < 8; bt++) begin
                        if (dc_refill_be[bt]) begin
                            next_cache_data[replacement_way][dc_refill_index][8*bt +: 8] = 
                                dc_refill_data[8*bt +: 8];
                        end
                    end
                end
            end 
            else if (invalid_way != -1) begin
                // Case 2: Miss but found invalid way - use it
                replacement_way = invalid_way;
                next_valids[replacement_way][dc_refill_index] = 1'b1;
                next_dirties[replacement_way][dc_refill_index] = dc_refill_dir;
                next_cache_data[replacement_way][dc_refill_index] = dc_refill_data;
                next_cache_tags[replacement_way][dc_refill_index] = dc_refill_id;
            end 
            else begin
                // Case 3: Miss and all ways valid - replace LRU
                replacement_way = lru_bits[dc_refill_index];
                
                // Check if we need to write back evicted line
                if (dirties[replacement_way][dc_refill_index]) begin
                    dc_wb_valid = 1'b1;
                    dc_wb_data = ld_cache_fetched_data[replacement_way][dc_refill_index];
                    dc_wb_addr = {16'b0, cache_tags[replacement_way][dc_refill_index], dc_refill_index, 3'b0};

                end
                
                // Install new line
                next_dirties[replacement_way][dc_refill_index] = dc_refill_dir;
                next_cache_data[replacement_way][dc_refill_index] = dc_refill_data;
                next_cache_tags[replacement_way][dc_refill_index] = dc_refill_id;
            end
            
            // Update LRU
            next_lru_bits[dc_refill_index] = ~replacement_way;
        end
    end
    
    // =============================================
    // Sequential Update (Flip-Flops)
    // =============================================
    always_ff @(posedge clk) begin
        if (rst) begin
            // Clear all cache state on rst
            for (int way = 0; way < NUM_WAYS; way++) begin
                valids[way] <= `SYS_SMALL_DELAY '0;
                cache_tags[way] <= `SYS_SMALL_DELAY '0;
                ld_cache_fetched_data[way] <= `SYS_SMALL_DELAY '0;
                dirties[way] <= `SYS_SMALL_DELAY '0;
            end
            lru_bits <= `SYS_SMALL_DELAY '0;
        end else begin
            // Update with next state values
            ld_cache_fetched_data <= `SYS_SMALL_DELAY next_cache_data;
            cache_tags <= `SYS_SMALL_DELAY next_cache_tags;
            valids <= `SYS_SMALL_DELAY next_valids;
            dirties <= `SYS_SMALL_DELAY next_dirties;
            lru_bits <= `SYS_SMALL_DELAY next_lru_bits;
        end
    end
endmodule


// ------------------------------------ Top-level dcache module ------------------------------------
// This module implements the top-level dcache with MSHR table and control logic
`define TEST_MODE
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
`timescale 1ns/100ps
module dcache(
    input   clk,
    input   rst,

    // Interface with memory controller
    input   [3:0] Ctlr2proc_response,
    input  [63:0] Ctlr2proc_data,
    input   [3:0] Ctlr2proc_tag,

    output logic [1:0] mc_dc_cmd,      
    output logic [`SYS_XLEN-1:0] mc_dc_addr,  
    output logic [63:0] mc_dc_wr_data,

    // Store Queue (SQ) interface
    input SQ_ENTRY_PACKET [2:0] sq_in,
    input SQ_ENTRY_PACKET [2:0] lsq_head_entry,
    output [2:0] rb_sq_stall,

    // Load Queue (LQ) / Load-SYS_FU_ADDR_WIDTH interface
    input [1:0] [`SYS_XLEN-1:0] ld_addr_in,
    input [1:0] ld_start,
    output logic [1:0] exs_dcache_hit_flags,
    output logic [1:0] [`SYS_XLEN-1:0] exs_dcache_resp_data,
    output logic [1:0] exs_dcache_brdcast_mask,
    output logic [`SYS_XLEN-1:0] exs_dcache_brdcast_data

    `ifdef TEST_MODE
      , output logic [1:0] [15:0] [63:0] dc_dbg_data
      , output logic [1:0] [15:0] [8:0] dc_dbg_tag
      , output logic [1:0] [15:0] dc_dbg_valid
      , output logic [1:0] [15:0] dc_dbg_dir

      , output MHSRS_ENTRY_PACKET [`SYS_MHSRS_NUM-1:0] mhsrs_entry_vector
      , output logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_head_index
      , output logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_issue_index
      , output logic [`SYS_MHSRS_ADDR_WIDTH-1:0] mhsrs_tail_index
    `endif
);

// ------------------------------------
// Write path for SQ to dcache
// ------------------------------------
logic [2:0] wr_en;
logic [2:0][3:0] wr_idx;
logic [2:0][8:0] icache_wr_id;
logic [2:0][63:0] wr_data;
logic [2:0][7:0] dc_sq_wr_be;
logic [2:0] wr_hit;

// Internal signals for way-hit tracking
// logic [1:0] rd1_way_hit;
// logic [2:0] wrh_way_hit;

// Read path for LQ from dcache
logic [1:0][3:0] rd_idx;
logic [1:0][8:0] rd_tag;
logic [1:0][63:0] rd_data;
logic [1:0] rd_valid;

// Signals to write back to memory when evicting dirty block
logic dc_wb_valid;
logic [63:0] dc_wb_data;
logic [`SYS_XLEN-1:0] dc_wb_addr;

// New memory address for store misses
logic [2:0][`SYS_XLEN-1:0] ld_new_mem_addr;

// Write-back from memory after load miss
logic dc_refillEN;
logic [3:0] dc_refill_index;
logic [8:0] dc_refill_id;
logic [63:0] dc_refill_data;
logic [7:0] dc_refill_be;
logic dc_refill_dir;

// SQ fl_head_reg hit logic
logic [2:0] sq_head_stall;
logic [2:0] sq_head_hit;
logic [2:0][8:0] dc_wrchk_id;
logic [2:0][3:0] dc_wrchk_index;

// ------------------------------------
// SQ Stall logic based on write hits
// ------------------------------------
always_comb begin
  case (sq_head_hit)
    3'b000, 3'b001, 3'b010, 3'b011: sq_head_stall = 3'b011;
    3'b100, 3'b101: sq_head_stall = 3'b001;
    3'b110, 3'b111: sq_head_stall = 3'b000;
  endcase
end

always_comb begin
  for (int i = 0; i < 3; i++)
    wr_hit[i] = sq_in[i].ready & sq_head_hit[i];
end

// ------------------------------------
// Extract tag and index from SQ fl_head_reg
// ------------------------------------
always_comb begin
  for (int i = 0; i < 3; i++) begin
    dc_wrchk_id[i] = lsq_head_entry[i].addr[`SYS_XLEN-1:7];   //  [SYS_XLEN-1:7] ➔ 9-bit tag
    dc_wrchk_index[i] = lsq_head_entry[i].addr[6:3];         //  [6:3] ➔ 4-bit index
  end
end

// ------------------------------------
// SQ input write buffer formatting
// ------------------------------------
always_comb begin : SQ_input_processing
  for (int i = 2; i >= 0; i--) begin
    wr_en[i] = sq_in[i].ready;
    icache_wr_id[i] = sq_in[i].addr[`SYS_XLEN-1:7];    // 9-bit tag
    wr_idx[i] = sq_in[i].addr[6:3];          // 4-bit index
    ld_new_mem_addr[i] = {sq_in[i].addr[`SYS_XLEN-1:3], 3'b0};  // still original 64b aligned
    if (sq_in[i].addr[2] == 1'b1) begin
      wr_data[i] = {sq_in[i].data, `SYS_XLEN'b0};
      dc_sq_wr_be[i] = {sq_in[i].usebytes, 4'b0};
    end else begin
      wr_data[i] = {`SYS_XLEN'b0, sq_in[i].data};
      dc_sq_wr_be[i] = {4'b0, sq_in[i].usebytes};
    end
  end
end

// ------------------------------------
// LQ input read tag/index extraction
// ------------------------------------

always_comb begin : LQ_input_processing
  for (int i = 1; i >= 0; i--) begin
    rd_tag[i] = ld_addr_in[i][`SYS_XLEN-1:7];   // 9-bit tag
    rd_idx[i] = ld_addr_in[i][6:3];         // 4-bit index
  end
end

// ------------------------------------
// Cache memory module instantiation
// ------------------------------------
dcache_2way_16set ram (
  .clk(clk),
  .rst(rst),
  .dc_sq_wrEN(wr_en),
  .dc_sq_wr_index(wr_idx),
  .dc_sq_wr_id(icache_wr_id),
  .dc_sq_wr_data(wr_data),
  .dc_sq_wr_be(dc_sq_wr_be),
  .dc_sq_wr_hit(wr_hit),
  .dc_wrchk_index(dc_wrchk_index),
  .dc_wrchk_id(dc_wrchk_id),
  .dc_wrchk_hit(sq_head_hit),
  .dc_Id_index(rd_idx),
  .dc_Id_id(rd_tag),
  .dc_Id_data(rd_data),
  .dc_Id_valid(rd_valid),
  .dc_wb_valid(dc_wb_valid),
  .dc_wb_data(dc_wb_data),
  .dc_wb_addr(dc_wb_addr),
  .dc_refillEN(dc_refillEN),
  .dc_refill_index(dc_refill_index),
  .dc_refill_id(dc_refill_id),
  .dc_refill_data(dc_refill_data),
  .dc_refill_be(dc_refill_be),
  .dc_refill_dir(dc_refill_dir)

  // ways
  // .rd1_way_hit(rd1_way_hit),
  // .wrh_way_hit(wrh_way_hit)

  `ifdef TEST_MODE
    , .dc_dbg_data(dc_dbg_data)
    , .dc_dbg_tag(dc_dbg_tag)
    , .dc_dbg_valid(dc_dbg_valid)
    , .dc_dbg_dir(dc_dbg_dir)
  `endif
);

// ------------------------------------
// Send final hit data from rd_data bus
// ------------------------------------
always_comb begin : send_hit_data
  exs_dcache_resp_data = 0;
  for (int i = 1; i >= 0; i--) begin
    if (ld_addr_in[i][2])
      exs_dcache_resp_data[i] = rd_data[i][63:32];
    else
      exs_dcache_resp_data[i] = rd_data[i][31:0];
  end
end


// --- MSHR Table and Control Logic ---

// Miss Status Holding Registers
MHSRS_ENTRY_PACKET [`SYS_MHSRS_NUM-1:0] mshrs_table, mshrs_table_next;
MHSRS_ENTRY_PACKET [`SYS_MHSRS_NUM-1:0] mshrs_table_next_after_retire;
MHSRS_ENTRY_PACKET [`SYS_MHSRS_NUM-1:0] mshrs_table_next_after_issue;

// MSHR fl_head_reg, issue, fl_tail_reg pointers
logic [`SYS_MHSRS_ADDR_WIDTH-1:0] fl_head_reg, fl_head_nxt;
logic [`SYS_MHSRS_ADDR_WIDTH-1:0] issue, issue_next;
logic [`SYS_MHSRS_ADDR_WIDTH-1:0] fl_tail_reg, fl_tail_nxt;

`ifdef TEST_MODE
  assign mhsrs_entry_vector = mshrs_table;
  assign mhsrs_head_index = fl_head_reg;
  assign mhsrs_issue_index = issue;
  assign mhsrs_tail_index = fl_tail_reg;
`endif

// Register MSHR state at posedge clk
always_ff @(posedge clk) begin : MSHRS_reg
  if (rst) begin
    mshrs_table <= `SYS_SMALL_DELAY 0;
    fl_head_reg <= `SYS_SMALL_DELAY 0;
    issue <= `SYS_SMALL_DELAY 0;
    fl_tail_reg <= `SYS_SMALL_DELAY 0;
  end else begin
    mshrs_table <= `SYS_SMALL_DELAY mshrs_table_next;
    fl_head_reg <= `SYS_SMALL_DELAY fl_head_nxt;
    issue <= `SYS_SMALL_DELAY issue_next;
    fl_tail_reg <= `SYS_SMALL_DELAY fl_tail_nxt;
  end
end

assign exs_dcache_brdcast_mask = 0;
assign exs_dcache_brdcast_data = 0;

// ------------------------------------
// fl_head_ptr logic: handle retiring MSHR entries
// ------------------------------------
always_comb begin : head_logic
  fl_head_nxt = fl_head_reg;
  dc_refillEN = 0; dc_refill_index = 0; dc_refill_id = 0;
  dc_refill_data = 0; dc_refill_dir = 0; dc_refill_be = 0;
  mshrs_table_next_after_retire = mshrs_table;

  if ((fl_head_reg != fl_tail_reg) && mshrs_table[fl_head_reg].issued &&
      (mshrs_table[fl_head_reg].command == BUS_STORE ||
      (mshrs_table[fl_head_reg].command == BUS_LOAD && Ctlr2proc_tag == mshrs_table[fl_head_reg].mem_tag))) begin

    fl_head_nxt = fl_head_reg + 1;
    mshrs_table_next_after_retire[fl_head_reg] = 0;
    // if the command is a store, just lsq_retire_mask the entry
    // If the command is a load, we still need to write back the data to cache
    if (mshrs_table[fl_head_reg].command == BUS_LOAD) begin
      dc_refillEN = 1'b1;
      {dc_refill_id, dc_refill_index} = mshrs_table[fl_head_reg].addr[`SYS_XLEN-1:3]; // 9bits + 4bits
      dc_refill_data = Ctlr2proc_data;
      dc_refill_dir = mshrs_table[fl_head_reg].dirty;
      dc_refill_be = mshrs_table[fl_head_reg].usebytes;

      // [Store-miss]: we need to overwrite the data in the table with the data coming from memory
      if (mshrs_table[fl_head_reg].dirty == 1'b1) begin
        for (int j = 7; j >= 0; j--) begin
          if (mshrs_table[fl_head_reg].usebytes[j])
            dc_refill_data[8*j +: 8] = mshrs_table[fl_head_reg].data[8*j +: 8];
        end
      end
    end
  end
end

// ------------------------------------
// Issue logic: send memory request to controller
// ------------------------------------
always_comb begin : issue_logic
  issue_next = issue;
  mc_dc_cmd = BUS_NONE;
  mc_dc_addr = 0;
  mc_dc_wr_data = 0;
  mshrs_table_next_after_issue = mshrs_table_next_after_retire;

  if ((issue != fl_tail_reg) && !mshrs_table[issue].issued) begin
    mc_dc_cmd = mshrs_table[issue].command;
    mc_dc_addr = mshrs_table[issue].addr;
    mc_dc_wr_data = mshrs_table[issue].data;
  end

  if (mshrs_table[issue].command != BUS_NONE && Ctlr2proc_response != 4'b0) begin
    mshrs_table_next_after_issue[issue].mem_tag = Ctlr2proc_response;
    mshrs_table_next_after_issue[issue].issued = 1'b1;
    issue_next = issue + 1;
  end
end


// ------------------------------------
// Load Hazard Detection Logic
// ------------------------------------
logic [1:0] is_there_load_hazard;
logic [1:0][7:0] load_hazard, load_hazard_next;
logic [1:0] ld_request, ld_request_next;

// **--- Load Merge Check ---
logic [1:0] load_merge_hit;
logic [1:0][`SYS_MHSRS_ADDR_WIDTH-1:0] load_merge_idx;

always_comb begin
  for (int i = 0; i < 2; i++) begin
    load_merge_hit[i] = 0;
    load_merge_idx[i] = 0;
    if (ld_start[i] && !rd_valid[i]) begin
      for (int j = 0; j < `SYS_MHSRS_NUM; j++) begin
        if (mshrs_table[j].command == BUS_LOAD && 
            mshrs_table[j].addr[`SYS_XLEN-1:3] == ld_addr_in[i][`SYS_XLEN-1:3]) begin
          load_merge_hit[i] = 1;
          load_merge_idx[i] = j;
        end
      end
    end
  end
end

// **--- Load-Store Hazard Check ---
logic [1:0] load_store_hazard;

always_comb begin
  for (int i = 0; i < 2; i++) begin
    load_store_hazard[i] = 0;
    for (int j = 0; j < `SYS_MHSRS_NUM; j++) begin
      if (mshrs_table[j].command == BUS_STORE &&
          mshrs_table[j].addr[`SYS_XLEN-1:3] == ld_addr_in[i][`SYS_XLEN-1:3]) begin
        load_store_hazard[i] = 1;
      end
    end
  end
end

// Hazard detection: compare address [15:3] (ignore byte offset)
always_comb begin
  for (int i = 0; i < 8; i++) begin
    load_hazard_next[0][i] = mshrs_table[i].dirty && (mshrs_table[i].addr[15:3] == ld_addr_in[0][15:3]);
    load_hazard_next[1][i] = mshrs_table[i].dirty && (mshrs_table[i].addr[15:3] == ld_addr_in[1][15:3]);
  end
end

// Record load hazard information across cycles
always_ff @(posedge clk) begin
  if (rst) load_hazard <= `SYS_SMALL_DELAY 0;
  else begin
    if (ld_start[0]) load_hazard[0] <= `SYS_SMALL_DELAY load_hazard_next[0];
    else load_hazard[0] <= `SYS_SMALL_DELAY (load_hazard[0] & load_hazard_next[0]);

    if (ld_start[1]) load_hazard[1] <= `SYS_SMALL_DELAY load_hazard_next[1];
    else load_hazard[1] <= `SYS_SMALL_DELAY (load_hazard[1] & load_hazard_next[1]);
  end
end

// Load request bookkeeping
always_ff @(posedge clk) begin
  if (rst) ld_request <= `SYS_SMALL_DELAY 0;
  else ld_request <= `SYS_SMALL_DELAY ld_request_next;
end

// // Final hazard signal for each load port
assign is_there_load_hazard[0] = ld_start[0] ? (|load_hazard_next[0] || load_store_hazard[0]) : (|load_hazard[0] || load_store_hazard[0]);
assign is_there_load_hazard[1] = ld_start[1] ? (|load_hazard_next[1] || load_store_hazard[1]) : (|load_hazard[1] || load_store_hazard[1]);
// // If hazard detected, force load to miss
assign exs_dcache_hit_flags[0] = (is_there_load_hazard[0] || load_merge_hit[0]) ? 1'b0 : rd_valid[0];
assign exs_dcache_hit_flags[1] = (is_there_load_hazard[1] || load_merge_hit[1]) ? 1'b0 : rd_valid[1];


// ------------------------------------
// Tail logic: allocate MSHRs for new requests
// ------------------------------------
logic [2:0][`SYS_MHSRS_ADDR_WIDTH-1:0] tail_after_ld;
logic [3:0][`SYS_MHSRS_ADDR_WIDTH-1:0] tail_after_wr;
logic [2:0] full_after_ld;
logic [3:0] full_after_wr;
logic [`SYS_MHSRS_ADDR_WIDTH-1:0] h_t_distance;

assign h_t_distance = fl_head_reg - fl_tail_reg;

assign rb_sq_stall[2] = sq_head_stall[2] | (h_t_distance == `SYS_MHSRS_ADDR_WIDTH'd4);
assign rb_sq_stall[1] = sq_head_stall[1] | (h_t_distance == `SYS_MHSRS_ADDR_WIDTH'd5);
assign rb_sq_stall[0] = sq_head_stall[0] | (h_t_distance == `SYS_MHSRS_ADDR_WIDTH'd6);

always_comb begin : tail_logic
  mshrs_table_next = mshrs_table_next_after_issue;
  ld_request_next = ld_request;
  tail_after_ld[2] = fl_tail_reg;
  full_after_ld[2] = (fl_tail_reg + 2 == fl_head_reg);

  for (int i = 1; i >= 0; i--) begin
  if (!full_after_ld[i+1] && ((!rd_valid[i] && ld_start[i]) || ld_request[i])) begin
    if (!load_merge_hit[i]) begin
      mshrs_table_next[tail_after_ld[i+1]].addr = {ld_addr_in[i][`SYS_XLEN-1:3], 3'b0};
      mshrs_table_next[tail_after_ld[i+1]].command = BUS_LOAD;
      mshrs_table_next[tail_after_ld[i+1]].mem_tag = 0;
      mshrs_table_next[tail_after_ld[i+1]].left_or_right = ld_addr_in[i][2];
      mshrs_table_next[tail_after_ld[i+1]].data = 0;
      mshrs_table_next[tail_after_ld[i+1]].issued = 0;
      mshrs_table_next[tail_after_ld[i+1]].exs_dcache_brdcast_mask = (i == 1) ? 2'b10 : 2'b01;
      mshrs_table_next[tail_after_ld[i+1]].usebytes = 8'b0;
      mshrs_table_next[tail_after_ld[i+1]].dirty = 0;
      tail_after_ld[i] = tail_after_ld[i+1] + 1;
    end else begin
      tail_after_ld[i] = tail_after_ld[i+1];
    end
      ld_request_next[i] = 0;
    end else if (full_after_ld[i+1] && !rd_valid[i] && ld_start[i]) begin
      tail_after_ld[i] = tail_after_ld[i+1];
      ld_request_next[i] = 1;
    end else begin
      tail_after_ld[i] = tail_after_ld[i+1];
    end
    full_after_ld[i] = (tail_after_ld[i] + 2 == fl_head_reg);
  end

  tail_after_wr[3] = tail_after_ld[0];
  for (int i = 2; i >= 0; i--) begin
    if (wr_en[i] && !wr_hit[i]) begin
      mshrs_table_next[tail_after_wr[i+1]].addr = ld_new_mem_addr[i];
      mshrs_table_next[tail_after_wr[i+1]].command = BUS_LOAD;
      mshrs_table_next[tail_after_wr[i+1]].mem_tag = 0;
      mshrs_table_next[tail_after_wr[i+1]].left_or_right = 0;
      mshrs_table_next[tail_after_wr[i+1]].data = wr_data[i];
      mshrs_table_next[tail_after_wr[i+1]].issued = 0;
      mshrs_table_next[tail_after_wr[i+1]].exs_dcache_brdcast_mask = 0;
      mshrs_table_next[tail_after_wr[i+1]].usebytes = dc_sq_wr_be[i];
      // mshrs_table_next[tail_after_wr[i+1]].way = wrh_way_hit[i];
      mshrs_table_next[tail_after_wr[i+1]].dirty = 1;
      tail_after_wr[i] = tail_after_wr[i+1] + 1;
    end else begin
      tail_after_wr[i] = tail_after_wr[i+1];
    end
  end

  fl_tail_nxt = tail_after_wr[0];

  if (dc_wb_valid) begin
    mshrs_table_next[tail_after_wr[0]].addr = dc_wb_addr;
    mshrs_table_next[tail_after_wr[0]].command = BUS_STORE;
    mshrs_table_next[tail_after_wr[0]].mem_tag = 0;
    mshrs_table_next[tail_after_wr[0]].left_or_right = 0;
    mshrs_table_next[tail_after_wr[0]].data = dc_wb_data;
    mshrs_table_next[tail_after_wr[0]].issued = 0;
    mshrs_table_next[tail_after_wr[0]].exs_dcache_brdcast_mask = 0;
    mshrs_table_next[tail_after_wr[0]].usebytes = 0;
    mshrs_table_next[tail_after_wr[0]].dirty = 0;
    fl_tail_nxt = tail_after_wr[0] + 1;
  end
end

endmodule
