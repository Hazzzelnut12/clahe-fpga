`timescale 1ns/1ps

/*
 * ram_cntl.v - 양선형 보간 가중치 및 이웃 타일 ID 계산 (순수 조합 회로)
 *
 * 타일 중심 좌표:
 *   열 k: x = 120 + k×240  (k=0..7: 120, 360, ..., 1800)
 *   행 k: y =  67 + k×135  (k=0..7:  67, 202, ..., 1012)
 *
 * 나눗셈 없음: 비교기 체인 + 오프셋 LUT → x_intra/y_intra 출력
 * 가중치 곱-시프트 근사 (×273>>8, ×243>>7)는 clahe_top Stage-B에서 수행
 */

module ram_cntl (
    input  wire [10:0] x, y,

    output wire [5:0]  tile_ul,   // 좌상 타일 ID {row_top[2:0], col_left[2:0]}
    output wire [5:0]  tile_ur,
    output wire [5:0]  tile_ll,
    output wire [5:0]  tile_lr,
    output wire [10:0] x_intra,   // 타일 내부 X 오프셋 (에지=0)
    output wire [10:0] y_intra    // 타일 내부 Y 오프셋 (에지=0)
);

    // ========== X 방향 ==========

    wire at_left_edge  = (x < 11'd120);
    wire at_right_edge = (x >= 11'd1800);

    // 첫 타일 중심(x=120)을 원점으로 이동 → 비교기 체인 단순화
    wire [10:0] x_centered = at_left_edge ? 11'd0 : (x - 11'd120);

    wire [2:0] col_left = at_left_edge  ? 3'd0 :
                          at_right_edge ? 3'd7 :
                          (x_centered < 11'd240)  ? 3'd0 :
                          (x_centered < 11'd480)  ? 3'd1 :
                          (x_centered < 11'd720)  ? 3'd2 :
                          (x_centered < 11'd960)  ? 3'd3 :
                          (x_centered < 11'd1200) ? 3'd4 :
                          (x_centered < 11'd1440) ? 3'd5 :
                          (x_centered < 11'd1680) ? 3'd6 : 3'd7;

    // 에지에서는 col_left와 동일 → 단일 타일 사용
    wire [2:0] col_right = (at_left_edge || at_right_edge) ? col_left : (col_left + 3'd1);

    // col_left × 240 오프셋 LUT (곱셈 대체)
    wire [10:0] col_base = (col_left == 3'd1) ? 11'd240  :
                           (col_left == 3'd2) ? 11'd480  :
                           (col_left == 3'd3) ? 11'd720  :
                           (col_left == 3'd4) ? 11'd960  :
                           (col_left == 3'd5) ? 11'd1200 :
                           (col_left == 3'd6) ? 11'd1440 :
                           (col_left == 3'd7) ? 11'd1680 : 11'd0;

    assign x_intra = (at_left_edge || at_right_edge) ? 11'd0 : (x_centered - col_base);

    // ========== Y 방향 (X와 동일 구조) ==========

    wire at_top_edge = (y < 11'd67);
    wire at_bot_edge = (y >= 11'd1012);

    wire [10:0] y_centered = at_top_edge ? 11'd0 : (y - 11'd67);

    wire [2:0] row_top = at_top_edge ? 3'd0 :
                         at_bot_edge ? 3'd7 :
                         (y_centered < 11'd135) ? 3'd0 :
                         (y_centered < 11'd270) ? 3'd1 :
                         (y_centered < 11'd405) ? 3'd2 :
                         (y_centered < 11'd540) ? 3'd3 :
                         (y_centered < 11'd675) ? 3'd4 :
                         (y_centered < 11'd810) ? 3'd5 :
                         (y_centered < 11'd945) ? 3'd6 : 3'd7;

    wire [2:0] row_bot = (at_top_edge || at_bot_edge) ? row_top : (row_top + 3'd1);

    // row_top × 135 오프셋 LUT
    wire [10:0] row_base = (row_top == 3'd1) ? 11'd135 :
                           (row_top == 3'd2) ? 11'd270 :
                           (row_top == 3'd3) ? 11'd405 :
                           (row_top == 3'd4) ? 11'd540 :
                           (row_top == 3'd5) ? 11'd675 :
                           (row_top == 3'd6) ? 11'd810 :
                           (row_top == 3'd7) ? 11'd945 : 11'd0;

    assign y_intra = (at_top_edge || at_bot_edge) ? 11'd0 : (y_centered - row_base);

    // ========== 타일 ID 조합 = {행[2:0], 열[2:0]} ==========
    assign tile_ul = {row_top, col_left};
    assign tile_ur = {row_top, col_right};
    assign tile_ll = {row_bot, col_left};
    assign tile_lr = {row_bot, col_right};

endmodule
