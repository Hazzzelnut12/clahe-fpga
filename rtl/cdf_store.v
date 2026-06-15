`timescale 1ns/1ps

/*
 * cdf_store.v - 64타일 CDF 저장소 (Ping-Pong BRAM)
 *
 * 쓰기: clipper CDF / 읽기: 보간용 4방향 이웃 타일 × Y0/Y1 = 8포트 동시
 * BRAM 8개 (방향4 × Y0/Y1): 8포트 위해 분리 — 3-Port 추론은 Vivado hang 유발
 * 주소: {page(1), tile(6), bin(8)} = 15비트 (32,768/BRAM)
 * Ping-Pong: frame_cdf_done마다 wr/rd page 교체 (현재 쓰기 ↔ 이전 읽기)
 */

module cdf_store (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        stall,           // 1이면 읽기 레지스터 갱신 중지

    input  wire        frame_cdf_done,  // 64타일 CDF 완성 → page 전환

    input  wire        wr_en,
    input  wire [5:0]  wr_tile,         // {row[2:0], col[2:0]}
    input  wire [7:0]  wr_bin,
    input  wire [7:0]  wr_data,         // 정규화된 CDF (0~255)

    input  wire [5:0]  tile_ul, tile_ur, tile_ll, tile_lr,

    input  wire [7:0]  pixel_y0,
    output reg  [7:0]  cdf_y0_ul, cdf_y0_ur, cdf_y0_ll, cdf_y0_lr,

    input  wire [7:0]  pixel_y1,
    output reg  [7:0]  cdf_y1_ul, cdf_y1_ur, cdf_y1_ll, cdf_y1_lr
);

    // block 강제: distributed 추론 시 3-Port 합성 → Elaboration hang
    // _y0/_y1 = Y0/Y1 픽셀 읽기 포트
    (* ram_style = "block" *) reg [7:0] mem_ul_y0 [0:32767];
    (* ram_style = "block" *) reg [7:0] mem_ul_y1 [0:32767];
    (* ram_style = "block" *) reg [7:0] mem_ur_y0 [0:32767];
    (* ram_style = "block" *) reg [7:0] mem_ur_y1 [0:32767];
    (* ram_style = "block" *) reg [7:0] mem_ll_y0 [0:32767];
    (* ram_style = "block" *) reg [7:0] mem_ll_y1 [0:32767];
    (* ram_style = "block" *) reg [7:0] mem_lr_y0 [0:32767];
    (* ram_style = "block" *) reg [7:0] mem_lr_y1 [0:32767];

    // 웜업 전 출력 = 검은 이미지 (시뮬 X값 전파 방지)
    integer i;
    initial begin
        for (i = 0; i < 32768; i = i + 1) begin
            mem_ul_y0[i] = 8'b0; mem_ul_y1[i] = 8'b0;
            mem_ur_y0[i] = 8'b0; mem_ur_y1[i] = 8'b0;
            mem_ll_y0[i] = 8'b0; mem_ll_y1[i] = 8'b0;
            mem_lr_y0[i] = 8'b0; mem_lr_y1[i] = 8'b0;
        end
    end

    reg frame_wr_page;
    always @(posedge clk) begin
        if (!rst_n)              frame_wr_page <= 1'b0;
        else if (frame_cdf_done) frame_wr_page <= ~frame_wr_page;
    end
    wire frame_rd_page = ~frame_wr_page;

    wire [14:0] wr_addr = {frame_wr_page, wr_tile, wr_bin};

    // 8개 BRAM 동일 데이터 동시 기록
    always @(posedge clk) begin
        if (wr_en) begin
            mem_ul_y0[wr_addr] <= wr_data; mem_ul_y1[wr_addr] <= wr_data;
            mem_ur_y0[wr_addr] <= wr_data; mem_ur_y1[wr_addr] <= wr_data;
            mem_ll_y0[wr_addr] <= wr_data; mem_ll_y1[wr_addr] <= wr_data;
            mem_lr_y0[wr_addr] <= wr_data; mem_lr_y1[wr_addr] <= wr_data;
        end
    end

    // BRAM 동기 읽기 (1클럭 지연), stall 시 출력 동결
    always @(posedge clk) begin
        if (!stall) begin
            cdf_y0_ul <= mem_ul_y0[{frame_rd_page, tile_ul, pixel_y0}];
            cdf_y1_ul <= mem_ul_y1[{frame_rd_page, tile_ul, pixel_y1}];

            cdf_y0_ur <= mem_ur_y0[{frame_rd_page, tile_ur, pixel_y0}];
            cdf_y1_ur <= mem_ur_y1[{frame_rd_page, tile_ur, pixel_y1}];

            cdf_y0_ll <= mem_ll_y0[{frame_rd_page, tile_ll, pixel_y0}];
            cdf_y1_ll <= mem_ll_y1[{frame_rd_page, tile_ll, pixel_y1}];

            cdf_y0_lr <= mem_lr_y0[{frame_rd_page, tile_lr, pixel_y0}];
            cdf_y1_lr <= mem_lr_y1[{frame_rd_page, tile_lr, pixel_y1}];
        end
    end

endmodule
