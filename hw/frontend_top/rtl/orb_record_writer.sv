// ============================================================================
// orb_record_writer.sv
// Packs unified ORB frontend features into DDR4 Buffer-A frames matching
// docs/figures/fig2_extractor_record.svg and streams them over 64-bit AXI4-Stream
// (little-endian, TLAST on final word of frame):
//   * Header : 8 x 64-bit words (64 B): magic "ORBK", version 1, frame_id,
//              n_records, timestamp_ns, width, height, n_dropped, status,
//              per-level counts L0..L7.
//   * Record : 5 x 64-bit words (40 B) per keypoint:
//              w0..w3 = descriptor[255:0]
//              w4     = {4'd0, angle16[15:0], 4'd0, score[11:0], 1'b1, 1'b0,
//                        level[1:0], y[11:0], x[11:0]}
//
// Synchronous active-high reset (`rst`). BRAM-inferable record buffer.
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

module orb_record_writer #(
    parameter int          IMG_W      = vslam_cfg_pkg::IMG_W,
    parameter int          IMG_H      = vslam_cfg_pkg::IMG_H,
    parameter int          REC_AW     = vslam_cfg_pkg::REC_AW,     // max records = 1 << REC_AW
    parameter logic [31:0] MAGIC_ORBK = vslam_cfg_pkg::MAGIC_ORBK, // "ORBK"
    parameter logic [15:0] FMT_VER    = vslam_cfg_pkg::FMT_VER
)(
    input  wire         clk,
    input  wire         rst,

    // Frame control & metadata
    input  wire         frame_start,
    input  wire         frame_done,
    input  wire [31:0]  frame_id,
    input  wire [63:0]  frame_timestamp_ns,

    // Unified feature stream from orb_frontend_top
    input  wire         feat_valid,
    input  wire [1:0]   feat_level,
    input  wire [11:0]  feat_x,
    input  wire [11:0]  feat_y,
    input  wire [11:0]  feat_score,
    input  wire [1:0]   feat_quad,
    input  wire [3:0]   feat_sector,
    input  wire [255:0] feat_desc,

    // AXI4-Stream master (64-bit little-endian to AXI DMA S2MM)
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready,
    output wire [63:0]  m_axis_tdata,
    output wire         m_axis_tlast,

    output wire         busy,
    output logic [15:0] n_dropped
);

    localparam int MAX_RECORDS = 1 << REC_AW;

    typedef enum logic [1:0] {
        ST_COLLECT  = 2'd0,
        ST_EMIT_HDR = 2'd1,
        ST_EMIT_REC = 2'd2
    } state_t;

    state_t state;

    // 16-bit binary angle (65536 = 360 deg), aligned with trig_decoder quadrant map
    logic [15:0] angle16;
    always_comb begin
        case (feat_quad)
            2'd1:    angle16 = {2'b00,  feat_sector, 10'd512}; //   0.. 90 deg
            2'd0:    angle16 = {2'b01, ~feat_sector, 10'd512}; //  90..180 deg
            2'd3:    angle16 = {2'b10,  feat_sector, 10'd512}; // 180..270 deg
            default: angle16 = {2'b11, ~feat_sector, 10'd512}; // 270..360 deg
        endcase
    end

    logic [63:0] feat_meta_w4;
    assign feat_meta_w4 = {
        4'd0, angle16, 4'd0, feat_score,
        1'b1, 1'b0, feat_level, feat_y, feat_x
    };

    // Simple dual-port BRAM storage: 320 bits = {w4, w3, w2, w1, w0}
    logic [319:0] rec_mem [0:MAX_RECORDS-1];
    logic [319:0] rec_q;

    logic [31:0]       n_records;
    logic [15:0]       lvl_cnt0, lvl_cnt1, lvl_cnt2, lvl_cnt3;
    logic [31:0]       lat_frame_id;
    logic [63:0]       lat_timestamp_ns;

    logic [2:0]        hdr_idx;
    logic [REC_AW-1:0] rec_idx;
    logic [2:0]        word_idx;

    wire rec_full   = (n_records >= 32'(MAX_RECORDS));
    wire rec_wr_en  = (state == ST_COLLECT) && !frame_start && feat_valid && !rec_full;
    wire rec_drop   = (state == ST_COLLECT) && !frame_start && feat_valid &&  rec_full;

    // Single synchronous BRAM read port pre-fetching the next 320-bit record
    wire rd_en = ((state == ST_EMIT_HDR) && (hdr_idx == 3'd7) && m_axis_tready && (n_records != 32'd0)) ||
                 ((state == ST_EMIT_REC) && (word_idx == 3'd4) && m_axis_tready && (({{(32-REC_AW){1'b0}}, rec_idx} + 32'd1) < n_records));
    wire [REC_AW-1:0] rd_addr = (state == ST_EMIT_HDR) ? {REC_AW{1'b0}} : (rec_idx + {{(REC_AW-1){1'b0}}, 1'b1});

    always_ff @(posedge clk) begin
        if (rec_wr_en) begin
            rec_mem[n_records[REC_AW-1:0]] <= {feat_meta_w4, feat_desc};
        end
        if (rd_en) begin
            rec_q <= rec_mem[rd_addr];
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state            <= ST_COLLECT;
            n_records        <= 32'd0;
            n_dropped        <= 16'd0;
            lvl_cnt0         <= 16'd0;
            lvl_cnt1         <= 16'd0;
            lvl_cnt2         <= 16'd0;
            lvl_cnt3         <= 16'd0;
            lat_frame_id     <= 32'd0;
            lat_timestamp_ns <= 64'd0;
            hdr_idx          <= 3'd0;
            rec_idx          <= {REC_AW{1'b0}};
            word_idx         <= 3'd0;
        end else begin
            case (state)
                ST_COLLECT: begin
                    if (frame_start) begin
                        n_records        <= 32'd0;
                        n_dropped        <= 16'd0;
                        lvl_cnt0         <= 16'd0;
                        lvl_cnt1         <= 16'd0;
                        lvl_cnt2         <= 16'd0;
                        lvl_cnt3         <= 16'd0;
                        lat_frame_id     <= frame_id;
                        lat_timestamp_ns <= frame_timestamp_ns;
                    end else begin
                        if (rec_wr_en) begin
                            n_records <= n_records + 32'd1;
                            case (feat_level)
                                2'd0: lvl_cnt0 <= lvl_cnt0 + 16'd1;
                                2'd1: lvl_cnt1 <= lvl_cnt1 + 16'd1;
                                2'd2: lvl_cnt2 <= lvl_cnt2 + 16'd1;
                                2'd3: lvl_cnt3 <= lvl_cnt3 + 16'd1;
                            endcase
                        end else if (rec_drop && (n_dropped != 16'hFFFF)) begin
                            n_dropped <= n_dropped + 16'd1;
                        end

                        if (frame_done) begin
                            hdr_idx  <= 3'd0;
                            rec_idx  <= {REC_AW{1'b0}};
                            word_idx <= 3'd0;
                            state    <= ST_EMIT_HDR;
                        end
                    end
                end

                ST_EMIT_HDR: begin
                    if (m_axis_tready) begin
                        if (hdr_idx == 3'd7) begin
                            hdr_idx  <= 3'd0;
                            rec_idx  <= {REC_AW{1'b0}};
                            word_idx <= 3'd0;
                            state    <= (n_records == 32'd0) ? ST_COLLECT : ST_EMIT_REC;
                            if (n_records == 32'd0) begin
                                n_records <= 32'd0;
                                n_dropped <= 16'd0;
                                lvl_cnt0  <= 16'd0;
                                lvl_cnt1  <= 16'd0;
                                lvl_cnt2  <= 16'd0;
                                lvl_cnt3  <= 16'd0;
                            end
                        end else begin
                            hdr_idx <= hdr_idx + 3'd1;
                        end
                    end
                end

                ST_EMIT_REC: begin
                    if (m_axis_tready) begin
                        if (word_idx == 3'd4) begin
                            word_idx <= 3'd0;
                            if (({{(32-REC_AW){1'b0}}, rec_idx} + 32'd1) >= n_records) begin
                                state     <= ST_COLLECT;
                                n_records <= 32'd0;
                                n_dropped <= 16'd0;
                                lvl_cnt0  <= 16'd0;
                                lvl_cnt1  <= 16'd0;
                                lvl_cnt2  <= 16'd0;
                                lvl_cnt3  <= 16'd0;
                            end else begin
                                rec_idx <= rec_idx + {{(REC_AW-1){1'b0}}, 1'b1};
                            end
                        end else begin
                            word_idx <= word_idx + 3'd1;
                        end
                    end
                end

                default: state <= ST_COLLECT;
            endcase
        end
    end

    assign m_axis_tvalid = (state != ST_COLLECT);
    assign busy          = (state != ST_COLLECT);

    wire [15:0] status_word = {15'd0, (n_dropped != 16'd0)};

    wire [63:0] hdr_word = (hdr_idx == 3'd0) ? {16'd0, FMT_VER, MAGIC_ORBK} :
                           (hdr_idx == 3'd1) ? {n_records, lat_frame_id} :
                           (hdr_idx == 3'd2) ? lat_timestamp_ns :
                           (hdr_idx == 3'd3) ? {status_word, n_dropped, 16'(IMG_H), 16'(IMG_W)} :
                           (hdr_idx == 3'd4) ? {lvl_cnt3, lvl_cnt2, lvl_cnt1, lvl_cnt0} : 64'd0;

    wire [63:0] rec_word = (word_idx == 3'd0) ? rec_q[63:0] :
                           (word_idx == 3'd1) ? rec_q[127:64] :
                           (word_idx == 3'd2) ? rec_q[191:128] :
                           (word_idx == 3'd3) ? rec_q[255:192] :
                           (word_idx == 3'd4) ? rec_q[319:256] : 64'd0;

    wire last_rec = (({{(32-REC_AW){1'b0}}, rec_idx} + 32'd1) >= n_records);

    assign m_axis_tdata = (state == ST_EMIT_HDR) ? hdr_word :
                          (state == ST_EMIT_REC) ? rec_word : 64'd0;
    assign m_axis_tlast = (state == ST_EMIT_HDR) ? ((hdr_idx == 3'd7) && (n_records == 32'd0)) :
                          (state == ST_EMIT_REC) ? ((word_idx == 3'd4) && last_rec) : 1'b0;

endmodule

`default_nettype wire
