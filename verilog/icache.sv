module icache(
    input         clk,
    input         rst,

    input   [1:0] icache_shift,
    input  [2:0][`SYS_XLEN-1:0] icache_req_addr,
    input  [2:0][63:0]       cachemem_data,
    input  [2:0]             cachemem_valid,
    input         icache_branch,

    input   [3:0] icache_mem_resp_code,
    input  [63:0] icache_mem_resp_data,
    input   [3:0] icache_mem_resp_id,
    input         mc_ic_hold_flag,

    input         icache_pipeline_hold,

    output logic  [2:0][31:0] fetch_inst_words,
    output logic  [2:0]      fetch_inst_valids,

    output logic  [1:0] icache_mem_req_cmd,
    output logic  [`SYS_XLEN-1:0] icache_mem_req_addr,

    output logic  [2:0][4:0]  icache_index,
    output logic  [2:0][7:0]  icache_id,
    output logic  [4:0]       icache_wr_index,
    output logic  [7:0]       icache_wr_id,
    output logic             icache_wrEN
);

  // Prefetch signals
  logic [1:0]               icache_pref_cmd;
  logic [`SYS_XLEN-1:0]         icache_pref_addr;
  logic [4:0]               icache_pref_idx;
  logic [7:0]               icache_pref_id;
  logic                     icache_pref_wEN;
  logic                     pf_bus_priority;
  logic                     icache_pref_done;

  // Internal regs/wires
  logic [3:0] current_mem_tag;
  logic       miss_outstanding;
  logic [3:0] real_Imem2proc_response;
  logic [3:0] sync_Imem2proc_response;

  logic [4:0]               fetch_index, fetch_index_next;
  logic [7:0]               fetch_tag,   fetch_tag_next;
  logic [`SYS_XLEN-1:0]         fetch_addr,  last_fetch_addr;

  // Sync memory response, clear on mc_ic_hold_flag
  assign real_Imem2proc_response = icache_mem_resp_code & {4{~mc_ic_hold_flag}};
  assign {fetch_tag_next, fetch_index_next} = fetch_addr[`SYS_XLEN-1:3];

  // Extract tag and index via procedural loop
  always_comb begin
    for (int w = 2; w >=0; w--) begin
      {icache_id[w], icache_index[w]} = icache_req_addr[w][`SYS_XLEN-1:3];
    end
  end
  //assign {icache_id[2], icache_index[2]} = icache_req_addr[2][`SYS_XLEN-1:3];
  //assign {icache_id[1], icache_index[1]} = icache_req_addr[1][`SYS_XLEN-1:3];
  //assign {icache_id[0], icache_index[0]} = icache_req_addr[0][`SYS_XLEN-1:3];

  // Detect cache miss conditions
  //wire changed_addr = (icache_index[2] != fetch_index) || (icache_id[2] != fetch_tag);
  wire changed_addr = |({icache_id[2], icache_index[2]} ^ {fetch_tag, fetch_index});
  // A miss if any way is invalid
  wire cache_miss = |(~cachemem_valid);

  // Output data selection via reverse-index procedural loop
  always_comb begin
    for (int j = 2; j >= 0; j--) begin
      fetch_inst_words[j] = icache_req_addr[j][2]
                         ? cachemem_data[j][63:32]
                         : cachemem_data[j][31:0];
    end
  end
  //assign fetch_inst_words[2] = icache_req_addr[2][2] ? cachemem_data[2][63:32] : cachemem_data[2][31:0];
  //assign fetch_inst_words[1] = icache_req_addr[1][2] ? cachemem_data[1][63:32] : cachemem_data[1][31:0];
  //assign fetch_inst_words[0] = icache_req_addr[0][2] ? cachemem_data[0][63:32] : cachemem_data[0][31:0];

  // Valid signal output
  assign fetch_inst_valids = cachemem_valid;

  // Detect new fetch cycle via XOR reduction
  wire new_read        = |(fetch_addr ^ last_fetch_addr);
  // Outstanding miss persists until a non-zero response arrives
  wire last_miss       = miss_outstanding & ~(|sync_Imem2proc_response);
  // Enable write-back when tag matches and is non-zero
  wire fetch_wr_enable = (current_mem_tag == icache_mem_resp_id) & (|current_mem_tag);

  //wire unanswered_miss = last_miss | (cache_miss & (icache_branch | changed_addr | new_read));

  wire unanswered_miss = ((icache_branch | changed_addr | new_read) & cache_miss) | (~(icache_branch | changed_addr | new_read) & last_miss);

  // Eligible to fetch if a miss is pending and not in rst or lsq_stall_mask
  wire want_to_fetch    = unanswered_miss & ~(rst | icache_pipeline_hold);
  // Require a load when first fetching a new address
  wire require_load  = want_to_fetch & ~icache_pref_done;
  // Update memory tag either after write-back or on a new miss event
  wire update_mem_tag   = fetch_wr_enable | (require_load & (changed_addr | unanswered_miss));
  // Issue prefetch when load isn't required and bus allows prefetch
  wire prefetch_require = (icache_pref_cmd == BUS_LOAD) & ~require_load;

  // Memory command: issue BUS_LOAD when not in rst and either load or prefetch is active
  assign icache_mem_req_cmd = (~rst & (require_load | prefetch_require))? BUS_LOAD : BUS_NONE;
  assign pf_bus_priority          = require_load;

  // Choose between fetch and prefetch addresses using bitmasking
  assign icache_mem_req_addr = ({`SYS_XLEN{require_load}} & fetch_addr) | ({`SYS_XLEN{prefetch_require}} & icache_pref_addr);
  assign icache_wrEN = fetch_wr_enable | icache_pref_wEN;
  assign icache_wr_index          = fetch_wr_enable  ? fetch_index    : icache_pref_wEN ? icache_pref_idx : '0;
  assign icache_wr_id            = fetch_wr_enable  ? fetch_tag      : icache_pref_wEN ? icache_pref_id   : '0;

    // Prefetch instantiation
  `ifndef NO_PREFETCH
    prefetch pf (
      .clk              (clk),
      .rst              (rst),
      .Imem2pref_response (real_Imem2proc_response),
      .Imem2pref_tag      (icache_mem_resp_id),
      .pf_bus_priority           (pf_bus_priority),
      .branch             (icache_branch),
      .icache_req_addr   (icache_req_addr),
      .cachemem_valid     (cachemem_valid),
      .want_to_fetch      (want_to_fetch),
      .icache_pref_done    (icache_pref_done),
      .icache_pref_cmd   (icache_pref_cmd),
      .icache_pref_addr      (icache_pref_addr),
      .icache_pref_idx     (icache_pref_idx),
      .icache_pref_id       (icache_pref_id),
      .icache_pref_wEN (icache_pref_wEN)
    );
  `else
    assign icache_pref_done    = 1'b0;
    assign icache_pref_cmd   = 2'b00;
    assign icache_pref_addr      = '0;
    assign icache_pref_idx     = '0;
    assign icache_pref_id       = '0;
    assign icache_pref_wEN = 1'b0;
  `endif

  // Fetch address multiplexing
  always_comb begin
    case (icache_shift)
      2'd1: fetch_addr = {icache_req_addr[1][`SYS_XLEN-1:3],3'b0};
      2'd2: fetch_addr = {icache_req_addr[0][`SYS_XLEN-1:3],3'b0};
      default: fetch_addr = {icache_req_addr[2][`SYS_XLEN-1:3],3'b0};
    endcase
  end

  // Sequential state update
  always_ff @(posedge clk) begin
    if (rst) begin
      fetch_index             <= `SYS_SMALL_DELAY -1;
      fetch_tag               <= `SYS_SMALL_DELAY -1;
      current_mem_tag         <= `SYS_SMALL_DELAY 0;
      miss_outstanding        <= `SYS_SMALL_DELAY 0;
      sync_Imem2proc_response <= `SYS_SMALL_DELAY 0;
      last_fetch_addr         <= `SYS_SMALL_DELAY 0;
    end else begin
      fetch_index             <= `SYS_SMALL_DELAY fetch_index_next;
      fetch_tag               <= `SYS_SMALL_DELAY fetch_tag_next;
      miss_outstanding        <= `SYS_SMALL_DELAY unanswered_miss;
      sync_Imem2proc_response <= `SYS_SMALL_DELAY real_Imem2proc_response;
      last_fetch_addr         <= `SYS_SMALL_DELAY fetch_addr;

      if (fetch_wr_enable || icache_branch)
        current_mem_tag <= `SYS_SMALL_DELAY 0;
      else if (update_mem_tag)
        current_mem_tag <= `SYS_SMALL_DELAY real_Imem2proc_response;
    end
  end

endmodule

