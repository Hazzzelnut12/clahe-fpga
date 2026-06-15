`timescale 1ns/1ps

/*
 * histogram_8bank.v - 타일 열별 밝기 히스토그램
 *
 * LUTRAM 4개   : Y0/Y1 × page0/1 (동시 쓰기 충돌 방지)
 * 파이프라인   : 읽기(T) → 래치(T) → 쓰기(T+1)
 * 해저드 포워딩: 연속 동일 주소 시 next_count 사용
 * Ping-Pong    : wr/rd page 분리 → 누적·clipper 읽기 병행
 */

module histogram_8bank (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:16] din,             // {Y1_curr[31:24], Y0_curr[23:16]}
    input  wire        de,               // Data Enable
    input  wire [10:0] curr_x,           // 0~1918, 2씩 증가

    output reg         hist_row_done,    // 135줄 완료 펄스 (1클럭) → clipper 트리거

    input  wire [2:0]  rd_col,           // clipper 읽기 열 번호 (0~7)
    input  wire [7:0]  rd_bin,           // clipper 읽기 빈 인덱스 (0~255)
    output wire [15:0] hist_data,        // Y0+Y1 카운트 합산 (비동기)

    input  wire        hist_clear_start
);

    localparam NUM_COLS  = 8;
    localparam HIST_BINS = 256;

    // distributed: 비동기 읽기 / rw_addr_collision: READ_FIRST + SYNTH-6 억제
    (* ram_style = "distributed" *) (* rw_addr_collision = "yes" *) reg [15:0] hist_y0_page0 [0:2047];
    (* ram_style = "distributed" *) (* rw_addr_collision = "yes" *) reg [15:0] hist_y0_page1 [0:2047];
    (* ram_style = "distributed" *) (* rw_addr_collision = "yes" *) reg [15:0] hist_y1_page0 [0:2047];
    (* ram_style = "distributed" *) (* rw_addr_collision = "yes" *) reg [15:0] hist_y1_page1 [0:2047];

    // 시뮬 X값 전파 방지
    integer i;
    initial begin
        for (i = 0; i < 2048; i = i + 1) begin
            hist_y0_page0[i] = 16'b0; hist_y0_page1[i] = 16'b0;
            hist_y1_page0[i] = 16'b0; hist_y1_page1[i] = 16'b0;
        end
    end

    wire [7:0] y0_curr = din[23:16];
    wire [7:0] y1_curr = din[31:24];

    wire [2:0] tile_col = (curr_x < 11'd240)  ? 3'd0 :
                          (curr_x < 11'd480)  ? 3'd1 :
                          (curr_x < 11'd720)  ? 3'd2 :
                          (curr_x < 11'd960)  ? 3'd3 :
                          (curr_x < 11'd1200) ? 3'd4 :
                          (curr_x < 11'd1440) ? 3'd5 :
                          (curr_x < 11'd1680) ? 3'd6 : 3'd7;

    wire [10:0] addr_y0 = {tile_col, y0_curr};  // {열[2:0], 밝기[7:0]} = 11비트
    wire [10:0] addr_y1 = {tile_col, y1_curr};

    // 타일 행 카운터: 135행(0~134) 채우면 hist_row_done 펄스
    reg [7:0] y_tile_cnt;
    always @(posedge clk) begin
        if (!rst_n) begin
            hist_row_done <= 1'b0;
            y_tile_cnt    <= 8'b0;
        end else begin
            hist_row_done <= 1'b0;
            if (de && curr_x == 11'd1918) begin
                if (y_tile_cnt == 8'd134) begin
                    hist_row_done <= 1'b1;
                    y_tile_cnt    <= 8'b0;
                end else begin
                    y_tile_cnt <= y_tile_cnt + 1;
                end
            end
        end
    end

    // hist_row_done보다 1클럭 선행 → 새 행 첫 픽셀이 신규 page에 기록
    wire row_boundary = de && (curr_x == 11'd1918) && (y_tile_cnt == 8'd134);

    // 쓰기 page 선택
    reg wr_page;
    always @(posedge clk) begin
        if (!rst_n) wr_page <= 1'b0;
        else if (row_boundary) wr_page <= ~wr_page;
    end
    wire rd_page = ~wr_page;

    // page 전환 직후에도 Stage1 쓰기는 구 page로 가도록 1클럭 지연
    reg wr_page_d1;
    always @(posedge clk) begin
        if (!rst_n) wr_page_d1 <= 1'b0;
        else        wr_page_d1 <= wr_page;
    end

    // Stage0(T): 읽기 → 해저드 mux → +1 → 래치 / Stage1(T+1): LUTRAM 쓰기
    reg [10:0] addr_y0_d1, addr_y1_d1;
    reg [15:0] next_count_y0, next_count_y1;
    reg        de_d1;

    wire [15:0] rd_y0 = (wr_page == 1'b0) ? hist_y0_page0[addr_y0] : hist_y0_page1[addr_y0];
    wire [15:0] rd_y1 = (wr_page == 1'b0) ? hist_y1_page0[addr_y1] : hist_y1_page1[addr_y1];

    // 연속 동일 주소: 구값 대신 쓰기 예정값 포워딩
    wire hazard_y0 = de_d1 && (addr_y0 == addr_y0_d1);
    wire hazard_y1 = de_d1 && (addr_y1 == addr_y1_d1);

    wire [15:0] eff_y0 = hazard_y0 ? next_count_y0 : rd_y0;
    wire [15:0] eff_y1 = hazard_y1 ? next_count_y1 : rd_y1;

    always @(posedge clk) begin
        if (!rst_n) begin
            de_d1        <= 1'b0;
            addr_y0_d1   <= 11'b0; addr_y1_d1   <= 11'b0;
            next_count_y0 <= 16'b0; next_count_y1 <= 16'b0;
        end else if (de) begin
            de_d1        <= 1'b1;
            addr_y0_d1   <= addr_y0; addr_y1_d1   <= addr_y1;
            next_count_y0 <= eff_y0 + 1'b1;
            next_count_y1 <= eff_y1 + 1'b1;
        end else begin
            de_d1 <= 1'b0;  // 블랭킹: Stage1 쓰기 억제
        end
    end

    // rd_page 전체 초기화 (256bin × 8col = 2,048클럭)
    reg        clear_active;
    reg [7:0]  clear_bin;
    reg [2:0]  clear_col;

    always @(posedge clk) begin
        if (!rst_n) begin
            clear_active <= 1'b0;
            clear_bin <= 8'b0; clear_col <= 3'b0;
        end else if (hist_clear_start) begin
            clear_active <= 1'b1;
            clear_bin <= 8'b0; clear_col <= 3'b0;
        end else if (clear_active) begin
            if (clear_bin == HIST_BINS - 1) begin
                clear_bin <= 8'b0;
                if (clear_col == NUM_COLS - 1) clear_active <= 1'b0;
                else clear_col <= clear_col + 1;
            end else begin
                clear_bin <= clear_bin + 1;
            end
        end
    end

    // clear=rd_page, 누적=wr_page_d1 → 항상 다른 page, 충돌 없음
    always @(posedge clk) begin
        if (clear_active && rd_page == 1'b0) begin
            hist_y0_page0[{clear_col, clear_bin}] <= 16'b0;
            hist_y1_page0[{clear_col, clear_bin}] <= 16'b0;
        end else if (de_d1 && wr_page_d1 == 1'b0) begin
            hist_y0_page0[addr_y0_d1] <= next_count_y0;
            hist_y1_page0[addr_y1_d1] <= next_count_y1;
        end
    end

    always @(posedge clk) begin
        if (clear_active && rd_page == 1'b1) begin
            hist_y0_page1[{clear_col, clear_bin}] <= 16'b0;
            hist_y1_page1[{clear_col, clear_bin}] <= 16'b0;
        end else if (de_d1 && wr_page_d1 == 1'b1) begin
            hist_y0_page1[addr_y0_d1] <= next_count_y0;
            hist_y1_page1[addr_y1_d1] <= next_count_y1;
        end
    end

    wire [10:0] rd_addr = {rd_col, rd_bin};
    assign hist_data = (rd_page == 1'b0) ? (hist_y0_page0[rd_addr] + hist_y1_page0[rd_addr])
                                         : (hist_y0_page1[rd_addr] + hist_y1_page1[rd_addr]);

endmodule
