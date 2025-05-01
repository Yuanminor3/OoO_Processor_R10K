module icache(
    input         clock,
    input         reset,

    input   [1:0] shift,
    input  [2:0][`XLEN-1:0] proc2Icache_addr,
    input  [2:0][63:0]       cachemem_data,
    input  [2:0]             cachemem_valid,
    input         take_branch,

    input   [3:0] Imem2proc_response,
    input  [63:0] Imem2proc_data,
    input   [3:0] Imem2proc_tag,
    input         d_request,

    input         hit_but_stall,

    output logic  [2:0][31:0] Icache_data_out,
    output logic  [2:0]      Icache_valid_out,

    output logic  [1:0] proc2Imem_command,
    output logic  [`XLEN-1:0] proc2Imem_addr,

    output logic  [2:0][4:0]  current_index,
    output logic  [2:0][7:0]  current_tag,
    output logic  [4:0]       wr_index,
    output logic  [7:0]       wr_tag,
    output logic             data_write_enable
);

  // Prefetch signals
  logic [1:0]               prefetch_command;
  logic [`XLEN-1:0]         prefetch_addr;
  logic [4:0]               prefetch_index;
  logic [7:0]               prefetch_tag;
  logic                     prefetch_wr_enable;
  logic                     give_way;
  logic                     already_fetched;

  // Internal regs/wires
  logic [3:0] current_mem_tag;
  logic       miss_outstanding;
  logic [3:0] real_Imem2proc_response;
  logic [3:0] sync_Imem2proc_response;

  logic [4:0]               fetch_index, fetch_index_next;
  logic [7:0]               fetch_tag,   fetch_tag_next;
  logic [`XLEN-1:0]         fetch_addr,  last_fetch_addr;

  // Sync memory response, clear on d_request
  assign real_Imem2proc_response = Imem2proc_response & {4{~d_request}};
  assign {fetch_tag_next, fetch_index_next} = fetch_addr[`XLEN-1:3];

  // Extract tag and index via procedural loop
  always_comb begin
    for (int w = 2; w >=0; w--) begin
      {current_tag[w], current_index[w]} = proc2Icache_addr[w][`XLEN-1:3];
    end
  end
  //assign {current_tag[2], current_index[2]} = proc2Icache_addr[2][`XLEN-1:3];
  //assign {current_tag[1], current_index[1]} = proc2Icache_addr[1][`XLEN-1:3];
  //assign {current_tag[0], current_index[0]} = proc2Icache_addr[0][`XLEN-1:3];

  // Detect cache miss conditions
  //wire changed_addr = (current_index[2] != fetch_index) || (current_tag[2] != fetch_tag);
  wire changed_addr = |({current_tag[2], current_index[2]} ^ {fetch_tag, fetch_index});
  // A miss if any way is invalid
  wire cache_miss = |(~cachemem_valid);

  // Output data selection via reverse-index procedural loop
  always_comb begin
    for (int j = 2; j >= 0; j--) begin
      Icache_data_out[j] = proc2Icache_addr[j][2]
                         ? cachemem_data[j][63:32]
                         : cachemem_data[j][31:0];
    end
  end
  //assign Icache_data_out[2] = proc2Icache_addr[2][2] ? cachemem_data[2][63:32] : cachemem_data[2][31:0];
  //assign Icache_data_out[1] = proc2Icache_addr[1][2] ? cachemem_data[1][63:32] : cachemem_data[1][31:0];
  //assign Icache_data_out[0] = proc2Icache_addr[0][2] ? cachemem_data[0][63:32] : cachemem_data[0][31:0];

  // Valid signal output
  assign Icache_valid_out = cachemem_valid;

  // Detect new fetch cycle via XOR reduction
  wire new_read        = |(fetch_addr ^ last_fetch_addr);
  // Outstanding miss persists until a non-zero response arrives
  wire last_miss       = miss_outstanding & ~(|sync_Imem2proc_response);
  // Enable write-back when tag matches and is non-zero
  wire fetch_wr_enable = (current_mem_tag == Imem2proc_tag) & (|current_mem_tag);

  //wire unanswered_miss = last_miss | (cache_miss & (take_branch | changed_addr | new_read));

  wire unanswered_miss = ((take_branch | changed_addr | new_read) & cache_miss) | (~(take_branch | changed_addr | new_read) & last_miss);

  // Eligible to fetch if a miss is pending and not in reset or stall
  wire want_to_fetch    = unanswered_miss & ~(reset | hit_but_stall);
  // Require a load when first fetching a new address
  wire require_load  = want_to_fetch & ~already_fetched;
  // Update memory tag either after write-back or on a new miss event
  wire update_mem_tag   = fetch_wr_enable | (require_load & (changed_addr | unanswered_miss));
  // Issue prefetch when load isn't required and bus allows prefetch
  wire prefetch_require = (prefetch_command == BUS_LOAD) & ~require_load;

  // Memory command: issue BUS_LOAD when not in reset and either load or prefetch is active
  assign proc2Imem_command = (~reset & (require_load | prefetch_require))? BUS_LOAD : BUS_NONE;
  assign give_way          = require_load;

  // Choose between fetch and prefetch addresses using bitmasking
  assign proc2Imem_addr = ({`XLEN{require_load}} & fetch_addr) | ({`XLEN{prefetch_require}} & prefetch_addr);
  assign data_write_enable = fetch_wr_enable | prefetch_wr_enable;
  assign wr_index          = fetch_wr_enable  ? fetch_index    : prefetch_wr_enable ? prefetch_index : '0;
  assign wr_tag            = fetch_wr_enable  ? fetch_tag      : prefetch_wr_enable ? prefetch_tag   : '0;

    // Prefetch instantiation
  `ifndef NO_PREFETCH
    prefetch pf (
      .clock              (clock),
      .reset              (reset),
      .Imem2pref_response (real_Imem2proc_response),
      .Imem2pref_tag      (Imem2proc_tag),
      .give_way           (give_way),
      .branch             (take_branch),
      .proc2Icache_addr   (proc2Icache_addr),
      .cachemem_valid     (cachemem_valid),
      .want_to_fetch      (want_to_fetch),
      .already_fetched    (already_fetched),
      .prefetch_command   (prefetch_command),
      .prefetch_addr      (prefetch_addr),
      .prefetch_index     (prefetch_index),
      .prefetch_tag       (prefetch_tag),
      .prefetch_wr_enable (prefetch_wr_enable)
    );
  `else
    assign already_fetched    = 1'b0;
    assign prefetch_command   = 2'b00;
    assign prefetch_addr      = '0;
    assign prefetch_index     = '0;
    assign prefetch_tag       = '0;
    assign prefetch_wr_enable = 1'b0;
  `endif

  // Fetch address multiplexing
  always_comb begin
    case (shift)
      2'd1: fetch_addr = {proc2Icache_addr[1][`XLEN-1:3],3'b0};
      2'd2: fetch_addr = {proc2Icache_addr[0][`XLEN-1:3],3'b0};
      default: fetch_addr = {proc2Icache_addr[2][`XLEN-1:3],3'b0};
    endcase
  end

  // Sequential state update
  always_ff @(posedge clock) begin
    if (reset) begin
      fetch_index             <= `SD -1;
      fetch_tag               <= `SD -1;
      current_mem_tag         <= `SD 0;
      miss_outstanding        <= `SD 0;
      sync_Imem2proc_response <= `SD 0;
      last_fetch_addr         <= `SD 0;
    end else begin
      fetch_index             <= `SD fetch_index_next;
      fetch_tag               <= `SD fetch_tag_next;
      miss_outstanding        <= `SD unanswered_miss;
      sync_Imem2proc_response <= `SD real_Imem2proc_response;
      last_fetch_addr         <= `SD fetch_addr;

      if (fetch_wr_enable || take_branch)
        current_mem_tag <= `SD 0;
      else if (update_mem_tag)
        current_mem_tag <= `SD real_Imem2proc_response;
    end
  end

endmodule

