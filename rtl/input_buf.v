`timescale 1ns/1ps

/*
 * input_buf.v - AXI-Stream 입력 버퍼 및 YCbCr 분리
 *
 * Y_curr → histogram 누적 (현재 픽셀 통계)
 * Y_prev → cdf_store 조회 (1클럭 전 픽셀 출력 결정)
 * Chroma : CLAHE 비처리, clahe_top에서 6단 시프트로 Y와 정렬
 */

module input_buf (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] din,       // {Y0[31:24], Cb[23:16], Y1[15:8], Cr[7:0]}
    input  wire        de_in,     // Data Enable (블랭킹 구간 0)
    output wire [31:0] y_quad,    // {Y1_curr[31:24], Y0_curr[23:16], Y1_prev[15:8], Y0_prev[7:0]}
    output wire [15:0] chroma,    // {Cr[15:8], Cb[7:0]}
    output wire        de_out     // de_in 그대로 전달
);

    wire [7:0] y0 = din[31:24];
    wire [7:0] cb = din[23:16];
    wire [7:0] y1 = din[15:8];
    wire [7:0] cr = din[7:0];

    // de_in=0(블랭킹)에서 갱신 안 함 — 쓰레기 값 유입 방지
    reg [7:0] y0_delayed, y1_delayed;
    always @(posedge clk) begin
        if (!rst_n) begin
            y0_delayed <= 8'b0;
            y1_delayed <= 8'b0;
        end else if (de_in) begin
            y0_delayed <= y0;
            y1_delayed <= y1;
        end
    end

    assign y_quad  = {y1, y0, y1_delayed, y0_delayed};
    assign chroma  = {cr, cb};
    assign de_out  = de_in;

endmodule
