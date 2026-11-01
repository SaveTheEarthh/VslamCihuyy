// -----------------------------------------------------------------------------
// orient_core.sv  --  streaming orientation module (Section 4.5), all windows
//
//   pixels (raster order, 1 px/clock when pix_valid)
//     -> coordinate counters -> orient_linebuf (37 rows)
//     -> orient_colsum (S, m01 of the column) -> orient_moments (delay line + m00/m10/m01)
//     -> orient_quantizer (quadrant, sector)
//
// One result is produced for every valid 37x37 window, i.e. for every pixel whose
// window centre (res_x, res_y) is at least 18 px from all borders.
// Latency from the pixel that completes the window to res_valid: 2+7+2+4 = 15 clocks.
//
//   pix_sof  : high with the first pixel of a frame (resyncs the x/y counters)
//   res_m*   : raw moments of that window (hardware frame, m10 mirrored), debug only
// -----------------------------------------------------------------------------
module orient_core #(
    parameter int PIX_W  = 8,
    parameter int IMG_W  = 640,
    parameter int IMG_H  = 480,
    parameter int N_SECT = 64,
    parameter int CW     = $clog2(IMG_W),
    parameter int RW     = $clog2(IMG_H),
    parameter int SEC_W  = $clog2(N_SECT / 4),
    parameter int SW     = PIX_W + 6,
    parameter int MCW    = PIX_W + 11,
    parameter int M0W    = PIX_W + 11,
    parameter int MW     = PIX_W + 14
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  pix_valid,
    input  logic                  pix_sof,
    input  logic [PIX_W-1:0]      pix_data,

    output logic                  res_valid,
    output logic [CW-1:0]         res_x,
    output logic [RW-1:0]         res_y,
    output logic [1:0]            res_quad,
    output logic [SEC_W-1:0]      res_sector,
    output logic [M0W-1:0]        res_m00,
    output logic signed [MW-1:0]  res_m10,
    output logic signed [MW-1:0]  res_m01
);
    localparam int TAG_W  = 1 + RW + CW;                       // {win_valid, cy, cx}
    localparam int QTAG_W = TAG_W + M0W + 2*MW;                // + raw moments (debug)

    // ---- raster coordinates of the incoming pixel -----------------------------------
    logic [CW-1:0] col_cnt, cur_col;
    logic [RW-1:0] row_cnt, cur_row;
    assign cur_col = pix_sof ? '0 : col_cnt;
    assign cur_row = pix_sof ? '0 : row_cnt;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            col_cnt <= '0;
            row_cnt <= '0;
        end else if (pix_valid) begin
            if (cur_col == CW'(IMG_W - 1)) begin
                col_cnt <= '0;
                row_cnt <= (cur_row == RW'(IMG_H - 1)) ? '0 : cur_row + 1'b1;
            end else begin
                col_cnt <= cur_col + 1'b1;
                row_cnt <= cur_row;
            end
        end
    end

    // window is complete when 36 rows and 36 columns precede the newest pixel
    logic             win_valid;
    logic [CW-1:0]    cx;
    logic [RW-1:0]    cy;
    logic [TAG_W-1:0] tag0;
    assign win_valid = (cur_row >= 6'd36) && (cur_col >= 6'd36);
    assign cx        = cur_col - CW'(18);
    assign cy        = cur_row - RW'(18);
    assign tag0      = {win_valid, cy, cx};

    // ---- line buffer -> column sums -> moments -> quantiser ---------------------
    logic                  col_valid;
    logic [37*PIX_W-1:0]   col_data;
    logic [TAG_W-1:0]      tag1;
    orient_linebuf #(.PIX_W(PIX_W), .IMG_W(IMG_W), .CW(CW), .TAG_W(TAG_W)) u_lb (
        .clk(clk), .rst_n(rst_n),
        .pix_valid(pix_valid), .pix_data(pix_data), .pix_col(cur_col), .tag_in(tag0),
        .col_valid(col_valid), .col_data(col_data), .tag_out(tag1));

    logic                  cs_valid;
    logic [SW-1:0]         cs_s;
    logic signed [MCW-1:0] cs_m;
    logic [TAG_W-1:0]      tag2;
    orient_colsum #(.PIX_W(PIX_W), .TAG_W(TAG_W)) u_cs (
        .clk(clk), .rst_n(rst_n),
        .valid_in(col_valid), .col_data(col_data), .tag_in(tag1),
        .valid_out(cs_valid), .s_out(cs_s), .m01c_out(cs_m), .tag_out(tag2));

    logic                  mo_valid;
    logic [M0W-1:0]        mo_m00;
    logic signed [MW-1:0]  mo_m10, mo_m01;
    logic [TAG_W-1:0]      tag3;
    orient_moments #(.PIX_W(PIX_W), .TAG_W(TAG_W)) u_mo (
        .clk(clk), .rst_n(rst_n),
        .valid_in(cs_valid), .s_in(cs_s), .m01c_in(cs_m), .tag_in(tag2),
        .valid_out(mo_valid), .m00_out(mo_m00), .m10_out(mo_m10), .m01_out(mo_m01),
        .tag_out(tag3));

    logic                  q_valid;
    logic [QTAG_W-1:0]     q_tag;
    orient_quantizer #(.MW(MW), .N_SECT(N_SECT), .TAG_W(QTAG_W)) u_q (
        .clk(clk), .rst_n(rst_n),
        .valid_in(mo_valid), .m10(mo_m10), .m01(mo_m01),
        .tag_in({tag3, mo_m00, mo_m10, mo_m01}),
        .valid_out(q_valid), .quadrant(res_quad), .sector(res_sector), .tag_out(q_tag));

    // unpack the sideband
    assign res_m01 = q_tag[MW-1:0];
    assign res_m10 = q_tag[2*MW-1:MW];
    assign res_m00 = q_tag[2*MW+M0W-1:2*MW];
    logic [TAG_W-1:0] qt;
    assign qt        = q_tag[QTAG_W-1:2*MW+M0W];
    assign res_x     = qt[CW-1:0];
    assign res_y     = qt[CW+RW-1:CW];
    assign res_valid = q_valid & qt[TAG_W-1];             // only complete windows
endmodule
