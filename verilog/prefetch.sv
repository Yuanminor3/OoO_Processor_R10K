`timescale 1ns/100ps

module prefetch (
    input                       clk,
    input                       rst,
    input  [3:0]                Imem2pref_response,
    input  [3:0]                Imem2pref_tag,

    input                       pf_bus_priority,           // it's high if the current load request is rejected when the fetch stage is also fetching
    input                       branch,             // it's high when the PC is not sequential
    input  [2:0][`SYS_XLEN-1:0]     icache_req_addr,   // current address the fetch stage is fetching
    input  [2:0]                cachemem_valid,

    input                       want_to_fetch,      // icache tells prefetch if it wants to send a new request

    output logic                icache_pref_done,     // tell icache if the request has already been sent

    output logic [1:0]          icache_pref_cmd,
    output logic [`SYS_XLEN-1:0]    icache_pref_addr,
    output logic [4:0]          icache_pref_idx,
    output logic [7:0]          icache_pref_id,
    output logic                icache_pref_wEN

);

    // Truly private signals renamed
    logic [3:0]                 synced_ImRsp;

    logic [`SYS_PREFETCH_DISTANCE-1:0][3:0]      mem_tag;
    logic [`SYS_PREFETCH_DISTANCE-1:0][3:0]      next_mem_tag;
    logic [`SYS_PREFETCH_DISTANCE-1:0][4:0]      store_prefetch_index;
    logic [`SYS_PREFETCH_DISTANCE-1:0][4:0]      next_store_index;
    logic [`SYS_PREFETCH_DISTANCE-1:0][7:0]      store_prefetch_tag;
    logic [`SYS_PREFETCH_DISTANCE-1:0][7:0]      next_store_tag;
    
    logic [7:0]                 count_pf;
    logic [7:0]                 count_last;
    logic [7:0]                 back_dist;
    logic [7:0]                 fwd_dist;

    logic                       tag_insert;
    logic [`SYS_XLEN-1:0]           first_miss;
    logic [`SYS_XLEN-1:0]           async_addr;

    logic                       found_loc;

    wire enough_pf = count_last >= `SYS_PREFETCH_DISTANCE;
    wire enable_pf = ~enough_pf & ~branch;
    wire reject_mem = enable_pf && ~pf_bus_priority && (Imem2pref_response == 0);
    wire valid_resp = enable_pf & ~pf_bus_priority & ~reject_mem;

    assign icache_pref_cmd = enable_pf ? BUS_LOAD : BUS_NONE;

    assign first_miss = ~cachemem_valid[2] ? icache_req_addr[2] :
                        ~cachemem_valid[1] ? icache_req_addr[1] :
                        ~cachemem_valid[0] ? icache_req_addr[0] :
                        icache_req_addr[0] + 4;

    assign back_dist = ~cachemem_valid[2] ? 0 :
                       ~cachemem_valid[1] ? (icache_req_addr[1][3] == icache_req_addr[2][3]) :
                       ~cachemem_valid[0] && (icache_req_addr[1][3] == icache_req_addr[2][3]) ? 1 :
                       2;

    assign fwd_dist = enough_pf ? 0 :
                      pf_bus_priority    ? 0 :
                      reject_mem  ? 0 :
                      1;

    wire is_start_zero = first_miss[`SYS_XLEN-1:3] == 0;

    assign async_addr = is_start_zero ? ({first_miss[`SYS_XLEN-1:3],3'b000} + 8 * count_pf) :
                         ({first_miss[`SYS_XLEN-1:3],3'b000} + 8 * count_pf - 8);

    wire neg_count = (count_last == 0 && back_dist > fwd_dist) ||
                     (count_last == 1 && back_dist > fwd_dist + 1);

    assign count_pf = branch ? 7'd0 :
                      neg_count  ? 7'd0 :
                      count_last - back_dist + fwd_dist;

    always_comb begin
        icache_pref_done = 0;
        if (want_to_fetch) begin
            for (int i = 0; i < `SYS_PREFETCH_DISTANCE; i++) begin
                if (mem_tag[i] != 0 && first_miss[15:3] == {store_prefetch_tag[i], store_prefetch_index[i]})
                    icache_pref_done = 1;
            end
        end

        // store logic
        icache_pref_idx = 0;
        icache_pref_id   = 0;
        icache_pref_wEN = 1'b0;
        next_mem_tag   = mem_tag;
        for (int i = 0; i < `SYS_PREFETCH_DISTANCE; i++) begin
            if (mem_tag[i] != 0 && mem_tag[i] == Imem2pref_tag) begin
                icache_pref_idx     = store_prefetch_index[i];
                icache_pref_id       = store_prefetch_tag[i];
                icache_pref_wEN = 1'b1;
                next_mem_tag[i]    = 0;
            end
        end

        found_loc               = 0;
        next_store_tag         = store_prefetch_tag;
        next_store_index       = store_prefetch_index;
        if (valid_resp) begin
            for (int i = 0; i < `SYS_PREFETCH_DISTANCE; i++) begin
                if (!found_loc && mem_tag[i] == 4'd0) begin
                    {next_store_tag[i], next_store_index[i]} = icache_pref_addr[`SYS_XLEN-1:3];
                    next_mem_tag[i] = Imem2pref_response;
                    found_loc      = 1'b1;
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            mem_tag            <= `SYS_SMALL_DELAY 0;
            store_prefetch_tag <= `SYS_SMALL_DELAY 0;
            store_prefetch_index <= `SYS_SMALL_DELAY 0;
            synced_ImRsp       <= `SYS_SMALL_DELAY 0;
            count_last         <= `SYS_SMALL_DELAY 0;
            icache_pref_addr      <= `SYS_SMALL_DELAY 0;
        end else begin
            mem_tag            <= `SYS_SMALL_DELAY next_mem_tag;
            store_prefetch_tag <= `SYS_SMALL_DELAY next_store_tag;
            store_prefetch_index <= `SYS_SMALL_DELAY next_store_index;
            synced_ImRsp       <= `SYS_SMALL_DELAY Imem2pref_response;
            count_last         <= `SYS_SMALL_DELAY count_pf;
            icache_pref_addr      <= `SYS_SMALL_DELAY async_addr;
        end
    end
endmodule
