//2-way 16-set Cache(256B) + Top Module: MSHRs

//----------------------2-Way Set Associative Cache Memory - 256B --------------------
//This module implements a 2-way set associative cache memory with 16 sets.
`define TEST_MODE
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
`timescale 1ns/100ps

module dcache_2way_16set(
    // Clock and reset
    input clock, 
    input reset,
    
    // Write Port 1 (from Store Queue hit)
    input  [2:0] wr1_en,                   // Write enable for each port
    input  [2:0][3:0] wr1_idx,             // 4-bit index
    input  [2:0][8:0] wr1_tag,             // 9-bit tag
    input  [2:0][63:0] wr1_data,           // Data to write
    input  [2:0][7:0] used_bytes,          // Byte enable mask
    input  [2:0] wr1_hit,                  // Indicates if write is a hit
    
    // Write Hit Check Port
    input  [2:0][3:0] wrh_idx,             // 4-bit index
    input  [2:0][8:0] wrh_tag,             // 9-bit tag
    output logic [2:0] wrh_hit,            // Hit result (1 if hit)
    
    // Read Ports (for Load instructions)
    input  [1:0][3:0] rd1_idx,             // 4-bit index
    input  [1:0][8:0] rd1_tag,             // 9-bit tag
    output logic [1:0][63:0] rd1_data,     // Read data output
    output logic [1:0] rd1_valid,          // Read valid (1 if hit)
    
    // Write-back Interface
    output logic need_write_mem,           // 1 if eviction requires write-back
    output logic [63:0] wb_mem_data,       // Data to write back to memory
    output logic [`XLEN-1:0] wb_mem_addr,  // Address to write back
    
    // Cache Refill Port (from Memory)
    input        wr2_en,                   // Refill enable
    input  [3:0] wr2_idx,                  // 4-bit index
    input  [8:0] wr2_tag,                  // 9-bit tag
    input  [63:0] wr2_data,                // Refill data
    input  [7:0] wr2_usebytes,             // Refill byte mask
    input        wr2_dirty,                // Refill dirty flag
    
    // Debug Ports (for testing only)
    `ifdef TEST_MODE
    output logic [1:0][15:0][63:0] cache_data_disp,  // Cache data per way
    output logic [1:0][15:0][8:0]  cache_tags_disp,  // 9-bit tags per way
    output logic [1:0][15:0]       valids_disp,      // Valid bits per way
    output logic [1:0][15:0]       dirties_disp      // Dirty bits per way
    `endif 
);

    // =============================================
    // Parameters and Internal Storage
    // =============================================
    parameter NUM_WAYS = 2;  // 2-way set associative
    parameter NUM_SETS = 16; // 16 sets (index width 4 bits)
    
    // Cache storage arrays (2 ways)
    logic [NUM_WAYS-1:0][NUM_SETS-1:0][63:0] cache_data;     // Data storage
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
    assign cache_data_disp = cache_data;
    assign cache_tags_disp = cache_tags;
    assign valids_disp = valids;
    `endif
    
    // =============================================
    // Read Port Logic
    // =============================================
    always_comb begin
        for (int port = 0; port < 2; port++) begin
            rd1_valid[port] = 0;
            rd1_data[port] = 0;
            for (int way = 0; way < NUM_WAYS; way++) begin
                if (valids[way][rd1_idx[port]] && 
                    cache_tags[way][rd1_idx[port]] == rd1_tag[port]) begin
                    rd1_valid[port] = 1;
                    rd1_data[port] = cache_data[way][rd1_idx[port]];
                end
            end
        end
    end
    
    // =============================================
    // Write Hit Check Logic
    // =============================================
    always_comb begin
        for (int port = 0; port < 3; port++) begin
            wrh_hit[port] = 0;
            for (int way = 0; way < NUM_WAYS; way++) begin
                if (valids[way][wrh_idx[port]] && 
                    cache_tags[way][wrh_idx[port]] == wrh_tag[port]) begin
                    wrh_hit[port] = 1;
                end
            end
        end
    end
    
    // =============================================
    // Cache Update Logic
    // =============================================
    always_comb begin
        // Default: keep current values
        next_cache_data = cache_data;
        next_cache_tags = cache_tags;
        next_valids = valids;
        next_dirties = dirties;
        next_lru_bits = lru_bits;
        
        // Initialize write-back signals
        need_write_mem = 0;
        wb_mem_data = 0;
        wb_mem_addr = 0;
        
        // -----------------------------
        // Process Store Queue Writes (wr1)
        // -----------------------------
        for (int port = 0; port < 3; port++) begin
            if (wr1_en[port] && wr1_hit[port]) begin
                // Find which way hit
                for (int way = 0; way < NUM_WAYS; way++) begin
                    if (valids[way][wr1_idx[port]] && 
                        cache_tags[way][wr1_idx[port]] == wr1_tag[port]) begin
                        hit_way = way;
                    end
                end
                
                // Update the hit way
                next_valids[hit_way][wr1_idx[port]] = 1'b1;
                next_cache_tags[hit_way][wr1_idx[port]] = wr1_tag[port];
                next_dirties[hit_way][wr1_idx[port]] = 1'b1;

                // Update data based on byte mask
                for (int bt = 0; bt < 8; bt++) begin
                    if (used_bytes[port][bt]) begin
                        next_cache_data[hit_way][wr1_idx[port]][8*bt +: 8] = 
                            wr1_data[port][8*bt +: 8];
                    end
                end
                
                // Update LRU (mark other way as recently used)
                next_lru_bits[wr1_idx[port]] = ~hit_way;
            end
        end
        
        // -----------------------------
        // Process Cache Refill (wr2)
        // -----------------------------
        if (wr2_en) begin
            invalid_way = -1;
            hit_way = -1;
            
            // Check for existing entry (hit)
            for (int way = 0; way < NUM_WAYS; way++) begin
                if (valids[way][wr2_idx] && cache_tags[way][wr2_idx] == wr2_tag) begin 
                    hit_way = way;
                end
            end
            
            // Find first invalid way if no hit
            if (hit_way == -1) begin
                for (int way = 0; way < NUM_WAYS; way++) begin
                    if (!valids[way][wr2_idx]) begin
                        invalid_way = way;
                        break;
                    end
                end
            end
            
            // Determine which way to use
            if (hit_way != -1) begin
                // Case 1: Hit - update existing entry
                replacement_way = hit_way;
                if (wr2_dirty) begin
                    next_dirties[replacement_way][wr2_idx] = 1'b1;
                    for (int bt = 0; bt < 8; bt++) begin
                        if (wr2_usebytes[bt]) begin
                            next_cache_data[replacement_way][wr2_idx][8*bt +: 8] = 
                                wr2_data[8*bt +: 8];
                        end
                    end
                end
            end 
            else if (invalid_way != -1) begin
                // Case 2: Miss but found invalid way - use it
                replacement_way = invalid_way;
                next_valids[replacement_way][wr2_idx] = 1'b1;
                next_dirties[replacement_way][wr2_idx] = wr2_dirty;
                next_cache_data[replacement_way][wr2_idx] = wr2_data;
                next_cache_tags[replacement_way][wr2_idx] = wr2_tag;
            end 
            else begin
                // Case 3: Miss and all ways valid - replace LRU
                replacement_way = lru_bits[wr2_idx];
                
                // Check if we need to write back evicted line
                if (dirties[replacement_way][wr2_idx]) begin
                    need_write_mem = 1'b1;
                    wb_mem_data = cache_data[replacement_way][wr2_idx];
                    wb_mem_addr = {16'b0, cache_tags[replacement_way][wr2_idx], wr2_idx, 3'b0};

                end
                
                // Install new line
                next_dirties[replacement_way][wr2_idx] = wr2_dirty;
                next_cache_data[replacement_way][wr2_idx] = wr2_data;
                next_cache_tags[replacement_way][wr2_idx] = wr2_tag;
            end
            
            // Update LRU
            next_lru_bits[wr2_idx] = ~replacement_way;
        end
    end
    
    // =============================================
    // Sequential Update (Flip-Flops)
    // =============================================
    always_ff @(posedge clock) begin
        if (reset) begin
            // Clear all cache state on reset
            for (int way = 0; way < NUM_WAYS; way++) begin
                valids[way] <= `SD '0;
                cache_tags[way] <= `SD '0;
                cache_data[way] <= `SD '0;
                dirties[way] <= `SD '0;
            end
            lru_bits <= `SD '0;
        end else begin
            // Update with next state values
            cache_data <= `SD next_cache_data;
            cache_tags <= `SD next_cache_tags;
            valids <= `SD next_valids;
            dirties <= `SD next_dirties;
            lru_bits <= `SD next_lru_bits;
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
    input   clock,
    input   reset,

    // Interface with memory controller
    input   [3:0] Ctlr2proc_response,
    input  [63:0] Ctlr2proc_data,
    input   [3:0] Ctlr2proc_tag,

    output logic [1:0] dcache2ctlr_command,      
    output logic [`XLEN-1:0] dcache2ctlr_addr,  
    output logic [63:0] dcache2ctlr_data,

    // Store Queue (SQ) interface
    input SQ_ENTRY_PACKET [2:0] sq_in,
    input SQ_ENTRY_PACKET [2:0] sq_head,
    output [2:0] sq_stall,

    // Load Queue (LQ) / Load-FU interface
    input [1:0] [`XLEN-1:0] ld_addr_in,
    input [1:0] ld_start,
    output logic [1:0] is_hit,
    output logic [1:0] [`XLEN-1:0] ld_data,
    output logic [1:0] broadcast_fu,
    output logic [`XLEN-1:0] broadcast_data

    `ifdef TEST_MODE
      , output logic [1:0] [15:0] [63:0] cache_data_disp
      , output logic [1:0] [15:0] [8:0] cache_tags_disp
      , output logic [1:0] [15:0] valids_disp
      , output logic [1:0] [15:0] dirties_disp

      , output MHSRS_ENTRY_PACKET [`MHSRS_W-1:0] MHSRS_disp
      , output logic [`MHSRS-1:0] head_pointer
      , output logic [`MHSRS-1:0] issue_pointer
      , output logic [`MHSRS-1:0] tail_pointer
    `endif
);

// ------------------------------------
// Write path for SQ to dcache
// ------------------------------------
logic [2:0] wr_en;
logic [2:0][3:0] wr_idx;
logic [2:0][8:0] wr_tag;
logic [2:0][63:0] wr_data;
logic [2:0][7:0] used_bytes;
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
logic need_write_mem;
logic [63:0] wb_mem_data;
logic [`XLEN-1:0] wb_mem_addr;

// New memory address for store misses
logic [2:0][`XLEN-1:0] ld_new_mem_addr;

// Write-back from memory after load miss
logic wr2_en;
logic [3:0] wr2_idx;
logic [8:0] wr2_tag;
logic [63:0] wr2_data;
logic [7:0] wr2_usebytes;
logic wr2_dirty;

// SQ head hit logic
logic [2:0] sq_head_stall;
logic [2:0] sq_head_hit;
logic [2:0][8:0] wrh_tag;
logic [2:0][3:0] wrh_idx;

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
// Extract tag and index from SQ head
// ------------------------------------
always_comb begin
  for (int i = 0; i < 3; i++) begin
    wrh_tag[i] = sq_head[i].addr[`XLEN-1:7];   //  [XLEN-1:7] ➔ 9-bit tag
    wrh_idx[i] = sq_head[i].addr[6:3];         //  [6:3] ➔ 4-bit index
  end
end

// ------------------------------------
// SQ input write buffer formatting
// ------------------------------------
always_comb begin : SQ_input_processing
  for (int i = 2; i >= 0; i--) begin
    wr_en[i] = sq_in[i].ready;
    wr_tag[i] = sq_in[i].addr[`XLEN-1:7];    // 9-bit tag
    wr_idx[i] = sq_in[i].addr[6:3];          // 4-bit index
    ld_new_mem_addr[i] = {sq_in[i].addr[`XLEN-1:3], 3'b0};  // still original 64b aligned
    if (sq_in[i].addr[2] == 1'b1) begin
      wr_data[i] = {sq_in[i].data, `XLEN'b0};
      used_bytes[i] = {sq_in[i].usebytes, 4'b0};
    end else begin
      wr_data[i] = {`XLEN'b0, sq_in[i].data};
      used_bytes[i] = {4'b0, sq_in[i].usebytes};
    end
  end
end

// ------------------------------------
// LQ input read tag/index extraction
// ------------------------------------

always_comb begin : LQ_input_processing
  for (int i = 1; i >= 0; i--) begin
    rd_tag[i] = ld_addr_in[i][`XLEN-1:7];   // 9-bit tag
    rd_idx[i] = ld_addr_in[i][6:3];         // 4-bit index
  end
end

// ------------------------------------
// Cache memory module instantiation
// ------------------------------------
dcache_2way_16set ram (
  .clock(clock),
  .reset(reset),
  .wr1_en(wr_en),
  .wr1_idx(wr_idx),
  .wr1_tag(wr_tag),
  .wr1_data(wr_data),
  .used_bytes(used_bytes),
  .wr1_hit(wr_hit),
  .wrh_idx(wrh_idx),
  .wrh_tag(wrh_tag),
  .wrh_hit(sq_head_hit),
  .rd1_idx(rd_idx),
  .rd1_tag(rd_tag),
  .rd1_data(rd_data),
  .rd1_valid(rd_valid),
  .need_write_mem(need_write_mem),
  .wb_mem_data(wb_mem_data),
  .wb_mem_addr(wb_mem_addr),
  .wr2_en(wr2_en),
  .wr2_idx(wr2_idx),
  .wr2_tag(wr2_tag),
  .wr2_data(wr2_data),
  .wr2_usebytes(wr2_usebytes),
  .wr2_dirty(wr2_dirty)

  // ways
  // .rd1_way_hit(rd1_way_hit),
  // .wrh_way_hit(wrh_way_hit)

  `ifdef TEST_MODE
    , .cache_data_disp(cache_data_disp)
    , .cache_tags_disp(cache_tags_disp)
    , .valids_disp(valids_disp)
    , .dirties_disp(dirties_disp)
  `endif
);

// ------------------------------------
// Send final hit data from rd_data bus
// ------------------------------------
always_comb begin : send_hit_data
  ld_data = 0;
  for (int i = 1; i >= 0; i--) begin
    if (ld_addr_in[i][2])
      ld_data[i] = rd_data[i][63:32];
    else
      ld_data[i] = rd_data[i][31:0];
  end
end


// --- MSHR Table and Control Logic ---

// Miss Status Holding Registers
MHSRS_ENTRY_PACKET [`MHSRS_W-1:0] mshrs_table, mshrs_table_next;
MHSRS_ENTRY_PACKET [`MHSRS_W-1:0] mshrs_table_next_after_retire;
MHSRS_ENTRY_PACKET [`MHSRS_W-1:0] mshrs_table_next_after_issue;

// MSHR head, issue, tail pointers
logic [`MHSRS-1:0] head, head_next;
logic [`MHSRS-1:0] issue, issue_next;
logic [`MHSRS-1:0] tail, tail_next;

`ifdef TEST_MODE
  assign MHSRS_disp = mshrs_table;
  assign head_pointer = head;
  assign issue_pointer = issue;
  assign tail_pointer = tail;
`endif

// Register MSHR state at posedge clock
always_ff @(posedge clock) begin : MSHRS_reg
  if (reset) begin
    mshrs_table <= `SD 0;
    head <= `SD 0;
    issue <= `SD 0;
    tail <= `SD 0;
  end else begin
    mshrs_table <= `SD mshrs_table_next;
    head <= `SD head_next;
    issue <= `SD issue_next;
    tail <= `SD tail_next;
  end
end

assign broadcast_fu = 0;
assign broadcast_data = 0;

// ------------------------------------
// Head logic: handle retiring MSHR entries
// ------------------------------------
always_comb begin : head_logic
  head_next = head;
  wr2_en = 0; wr2_idx = 0; wr2_tag = 0;
  wr2_data = 0; wr2_dirty = 0; wr2_usebytes = 0;
  mshrs_table_next_after_retire = mshrs_table;

  if ((head != tail) && mshrs_table[head].issued &&
      (mshrs_table[head].command == BUS_STORE ||
      (mshrs_table[head].command == BUS_LOAD && Ctlr2proc_tag == mshrs_table[head].mem_tag))) begin

    head_next = head + 1;
    mshrs_table_next_after_retire[head] = 0;
    // if the command is a store, just retire the entry
    // If the command is a load, we still need to write back the data to cache
    if (mshrs_table[head].command == BUS_LOAD) begin
      wr2_en = 1'b1;
      {wr2_tag, wr2_idx} = mshrs_table[head].addr[`XLEN-1:3]; // 9bits + 4bits
      wr2_data = Ctlr2proc_data;
      wr2_dirty = mshrs_table[head].dirty;
      wr2_usebytes = mshrs_table[head].usebytes;

      // [Store-miss]: we need to overwrite the data in the table with the data coming from memory
      if (mshrs_table[head].dirty == 1'b1) begin
        for (int j = 7; j >= 0; j--) begin
          if (mshrs_table[head].usebytes[j])
            wr2_data[8*j +: 8] = mshrs_table[head].data[8*j +: 8];
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
  dcache2ctlr_command = BUS_NONE;
  dcache2ctlr_addr = 0;
  dcache2ctlr_data = 0;
  mshrs_table_next_after_issue = mshrs_table_next_after_retire;

  if ((issue != tail) && !mshrs_table[issue].issued) begin
    dcache2ctlr_command = mshrs_table[issue].command;
    dcache2ctlr_addr = mshrs_table[issue].addr;
    dcache2ctlr_data = mshrs_table[issue].data;
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
logic [1:0][`MHSRS-1:0] load_merge_idx;

always_comb begin
  for (int i = 0; i < 2; i++) begin
    load_merge_hit[i] = 0;
    load_merge_idx[i] = 0;
    if (ld_start[i] && !rd_valid[i]) begin
      for (int j = 0; j < `MHSRS_W; j++) begin
        if (mshrs_table[j].command == BUS_LOAD && 
            mshrs_table[j].addr[`XLEN-1:3] == ld_addr_in[i][`XLEN-1:3]) begin
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
    for (int j = 0; j < `MHSRS_W; j++) begin
      if (mshrs_table[j].command == BUS_STORE &&
          mshrs_table[j].addr[`XLEN-1:3] == ld_addr_in[i][`XLEN-1:3]) begin
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
always_ff @(posedge clock) begin
  if (reset) load_hazard <= `SD 0;
  else begin
    if (ld_start[0]) load_hazard[0] <= `SD load_hazard_next[0];
    else load_hazard[0] <= `SD (load_hazard[0] & load_hazard_next[0]);

    if (ld_start[1]) load_hazard[1] <= `SD load_hazard_next[1];
    else load_hazard[1] <= `SD (load_hazard[1] & load_hazard_next[1]);
  end
end

// Load request bookkeeping
always_ff @(posedge clock) begin
  if (reset) ld_request <= `SD 0;
  else ld_request <= `SD ld_request_next;
end

// // Final hazard signal for each load port
assign is_there_load_hazard[0] = ld_start[0] ? (|load_hazard_next[0] || load_store_hazard[0]) : (|load_hazard[0] || load_store_hazard[0]);
assign is_there_load_hazard[1] = ld_start[1] ? (|load_hazard_next[1] || load_store_hazard[1]) : (|load_hazard[1] || load_store_hazard[1]);
// // If hazard detected, force load to miss
assign is_hit[0] = (is_there_load_hazard[0] || load_merge_hit[0]) ? 1'b0 : rd_valid[0];
assign is_hit[1] = (is_there_load_hazard[1] || load_merge_hit[1]) ? 1'b0 : rd_valid[1];


// ------------------------------------
// Tail logic: allocate MSHRs for new requests
// ------------------------------------
logic [2:0][`MHSRS-1:0] tail_after_ld;
logic [3:0][`MHSRS-1:0] tail_after_wr;
logic [2:0] full_after_ld;
logic [3:0] full_after_wr;
logic [`MHSRS-1:0] h_t_distance;

assign h_t_distance = head - tail;

assign sq_stall[2] = sq_head_stall[2] | (h_t_distance == `MHSRS'd4);
assign sq_stall[1] = sq_head_stall[1] | (h_t_distance == `MHSRS'd5);
assign sq_stall[0] = sq_head_stall[0] | (h_t_distance == `MHSRS'd6);

always_comb begin : tail_logic
  mshrs_table_next = mshrs_table_next_after_issue;
  ld_request_next = ld_request;
  tail_after_ld[2] = tail;
  full_after_ld[2] = (tail + 2 == head);

  for (int i = 1; i >= 0; i--) begin
  if (!full_after_ld[i+1] && ((!rd_valid[i] && ld_start[i]) || ld_request[i])) begin
    if (!load_merge_hit[i]) begin
      mshrs_table_next[tail_after_ld[i+1]].addr = {ld_addr_in[i][`XLEN-1:3], 3'b0};
      mshrs_table_next[tail_after_ld[i+1]].command = BUS_LOAD;
      mshrs_table_next[tail_after_ld[i+1]].mem_tag = 0;
      mshrs_table_next[tail_after_ld[i+1]].left_or_right = ld_addr_in[i][2];
      mshrs_table_next[tail_after_ld[i+1]].data = 0;
      mshrs_table_next[tail_after_ld[i+1]].issued = 0;
      mshrs_table_next[tail_after_ld[i+1]].broadcast_fu = (i == 1) ? 2'b10 : 2'b01;
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
    full_after_ld[i] = (tail_after_ld[i] + 2 == head);
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
      mshrs_table_next[tail_after_wr[i+1]].broadcast_fu = 0;
      mshrs_table_next[tail_after_wr[i+1]].usebytes = used_bytes[i];
      // mshrs_table_next[tail_after_wr[i+1]].way = wrh_way_hit[i];
      mshrs_table_next[tail_after_wr[i+1]].dirty = 1;
      tail_after_wr[i] = tail_after_wr[i+1] + 1;
    end else begin
      tail_after_wr[i] = tail_after_wr[i+1];
    end
  end

  tail_next = tail_after_wr[0];

  if (need_write_mem) begin
    mshrs_table_next[tail_after_wr[0]].addr = wb_mem_addr;
    mshrs_table_next[tail_after_wr[0]].command = BUS_STORE;
    mshrs_table_next[tail_after_wr[0]].mem_tag = 0;
    mshrs_table_next[tail_after_wr[0]].left_or_right = 0;
    mshrs_table_next[tail_after_wr[0]].data = wb_mem_data;
    mshrs_table_next[tail_after_wr[0]].issued = 0;
    mshrs_table_next[tail_after_wr[0]].broadcast_fu = 0;
    mshrs_table_next[tail_after_wr[0]].usebytes = 0;
    mshrs_table_next[tail_after_wr[0]].dirty = 0;
    tail_next = tail_after_wr[0] + 1;
  end
end

endmodule
