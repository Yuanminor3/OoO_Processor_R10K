`timescale 1ns/100ps

module prefetch (
    input                       clock,
    input                       reset,
    input  [3:0]                Imem2pref_response,
    input  [3:0]                Imem2pref_tag,

    input                       give_way,           // it's high if the current load request is rejected when the fetch stage is also fetching
    input                       branch,             // it's high when the PC is not sequential
    input  [2:0][`XLEN-1:0]     proc2Icache_addr,   // current address the fetch stage is fetching
    input  [2:0]                cachemem_valid,

    input                       want_to_fetch,      // icache tells prefetch if it wants to send a new request

    output logic                already_fetched,     // tell icache if the request has already been sent

    output logic [1:0]          prefetch_command,
    output logic [`XLEN-1:0]    prefetch_addr,
    output logic [4:0]          prefetch_index,
    output logic [7:0]          prefetch_tag,
    output logic                prefetch_wr_enable

    `ifdef  TEST_MODE
    , output logic [7:0]                 pref_count_display
    , output logic [`PREF-1:0][3:0]      mem_tag_display
    , output logic [`PREF-1:0][4:0]      store_prefetch_index_display
    , output logic [`PREF-1:0][7:0]      store_prefetch_tag_display
    `endif
);

    // Truly private signals renamed
    logic [3:0]                 synced_ImRsp;

    logic [`PREF-1:0][3:0]      mem_tag;
    logic [`PREF-1:0][3:0]      next_mem_tag;
    logic [`PREF-1:0][4:0]      store_prefetch_index;
    logic [`PREF-1:0][4:0]      next_store_index;
    logic [`PREF-1:0][7:0]      store_prefetch_tag;
    logic [`PREF-1:0][7:0]      next_store_tag;
    
    logic [7:0]                 count_pf;
    logic [7:0]                 count_last;
    logic [7:0]                 back_dist;
    logic [7:0]                 fwd_dist;

    logic                       tag_insert;
    logic [`XLEN-1:0]           first_miss;
    logic [`XLEN-1:0]           async_addr;

    logic                       found_loc;

    wire enough_pf = count_last >= `PREF;
    wire enable_pf = ~enough_pf & ~branch;
    wire reject_mem = enable_pf && ~give_way && (Imem2pref_response == 0);
    wire valid_resp = enable_pf & ~give_way & ~reject_mem;

    `ifdef TEST_MODE
    assign mem_tag_display = mem_tag;
    assign pref_count_display = count_pf;
    assign store_prefetch_index_display = store_prefetch_index;
    assign store_prefetch_tag_display = store_prefetch_tag;
    `endif

    assign prefetch_command = enable_pf ? BUS_LOAD : BUS_NONE;

    assign first_miss = ~cachemem_valid[2] ? proc2Icache_addr[2] :
                        ~cachemem_valid[1] ? proc2Icache_addr[1] :
                        ~cachemem_valid[0] ? proc2Icache_addr[0] :
                        proc2Icache_addr[0] + 4;

    assign back_dist = ~cachemem_valid[2] ? 0 :
                       ~cachemem_valid[1] ? (proc2Icache_addr[1][3] == proc2Icache_addr[2][3]) :
                       ~cachemem_valid[0] && (proc2Icache_addr[1][3] == proc2Icache_addr[2][3]) ? 1 :
                       2;

    assign fwd_dist = enough_pf ? 0 :
                      give_way    ? 0 :
                      reject_mem  ? 0 :
                      1;

    wire is_start_zero = first_miss[`XLEN-1:3] == 0;

    assign async_addr = is_start_zero ? ({first_miss[`XLEN-1:3],3'b000} + 8 * count_pf) :
                         ({first_miss[`XLEN-1:3],3'b000} + 8 * count_pf - 8);

    wire neg_count = (count_last == 0 && back_dist > fwd_dist) ||
                     (count_last == 1 && back_dist > fwd_dist + 1);

    assign count_pf = branch ? 7'd0 :
                      neg_count  ? 7'd0 :
                      count_last - back_dist + fwd_dist;

    always_comb begin
        already_fetched = 0;
        if (want_to_fetch) begin
            for (int i = 0; i < `PREF; i++) begin
                if (mem_tag[i] != 0 && first_miss[15:3] == {store_prefetch_tag[i], store_prefetch_index[i]})
                    already_fetched = 1;
            end
        end

        // store logic
        prefetch_index = 0;
        prefetch_tag   = 0;
        prefetch_wr_enable = 1'b0;
        next_mem_tag   = mem_tag;
        for (int i = 0; i < `PREF; i++) begin
            if (mem_tag[i] != 0 && mem_tag[i] == Imem2pref_tag) begin
                prefetch_index     = store_prefetch_index[i];
                prefetch_tag       = store_prefetch_tag[i];
                prefetch_wr_enable = 1'b1;
                next_mem_tag[i]    = 0;
            end
        end

        found_loc               = 0;
        next_store_tag         = store_prefetch_tag;
        next_store_index       = store_prefetch_index;
        if (valid_resp) begin
            for (int i = 0; i < `PREF; i++) begin
                if (!found_loc && mem_tag[i] == 4'd0) begin
                    {next_store_tag[i], next_store_index[i]} = prefetch_addr[`XLEN-1:3];
                    next_mem_tag[i] = Imem2pref_response;
                    found_loc      = 1'b1;
                end
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            mem_tag            <= `SD 0;
            store_prefetch_tag <= `SD 0;
            store_prefetch_index <= `SD 0;
            synced_ImRsp       <= `SD 0;
            count_last         <= `SD 0;
            prefetch_addr      <= `SD 0;
        end else begin
            mem_tag            <= `SD next_mem_tag;
            store_prefetch_tag <= `SD next_store_tag;
            store_prefetch_index <= `SD next_store_index;
            synced_ImRsp       <= `SD Imem2pref_response;
            count_last         <= `SD count_pf;
            prefetch_addr      <= `SD async_addr;
        end
    end
endmodule