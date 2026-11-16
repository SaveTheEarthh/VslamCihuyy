// -----------------------------------------------------------------------------
// kp_match.sv  --  compare every finished window with the head of the keypoint FIFO
// (Section 4.6: "coordinates from the orientation module are constantly compared
//  with the head of the keypoint FIFO; on a match BRIEF starts").
//
//   match : head == (x, y)            -> output the result, pop the FIFO
//   stale : head is earlier in raster order than (x, y) (duplicate / missed point)
//           -> pop it without output, so it can never block the FIFO
// One FIFO entry is popped per window clock at most.
// -----------------------------------------------------------------------------
module kp_match #(
    parameter int CW    = 7,
    parameter int RW    = 6,
    parameter int SEC_W = 4
)(
    input  logic              clk,
    input  logic              rst_n,
    // from orient_core
    input  logic              res_valid,
    input  logic [CW-1:0]     res_x,
    input  logic [RW-1:0]     res_y,
    input  logic [1:0]        res_quad,
    input  logic [SEC_W-1:0]  res_sector,
    // keypoint FIFO head
    input  logic              kp_valid,         // = !empty
    input  logic [CW-1:0]     kp_x,
    input  logic [RW-1:0]     kp_y,
    output logic              kp_pop,
    // to BRIEF
    output logic              out_valid,
    output logic [CW-1:0]     out_x,
    output logic [RW-1:0]     out_y,
    output logic [1:0]        out_quad,
    output logic [SEC_W-1:0]  out_sector
);
    logic match, stale;
    assign match  = kp_valid && (kp_x == res_x) && (kp_y == res_y);
    assign stale  = kp_valid && ((kp_y < res_y) || (kp_y == res_y && kp_x < res_x));
    assign kp_pop = res_valid && (match || stale);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
        end else begin
            out_valid <= res_valid && match;
        end
        if (res_valid && match) begin
            out_x      <= res_x;
            out_y      <= res_y;
            out_quad   <= res_quad;
            out_sector <= res_sector;
        end
    end
endmodule
