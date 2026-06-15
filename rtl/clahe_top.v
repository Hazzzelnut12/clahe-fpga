`timescale 1ns/1ps

/*
 * clahe_top.v - CLAHE 하드웨어 가속기 최상위 통합 모듈
 *
 * 타일: 8×8 = 64개 (240×135 픽셀/타일), 1클럭 = 2픽셀(Y0+Y1), 100MHz
 */

module clahe_top (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] s_axis_data,    // {Y0[31:24], Cb[23:16], Y1[15:8], Cr[7:0]}
    input  wire        s_axis_valid,
    output wire        s_axis_ready,

    output wire [31:0] m_axis_data,    // {Y0_enhanced, Cb, Y1_enhanced, Cr}
    output wire        m_axis_valid,
    input  wire        m_axis_ready,

    input  wire        v_sync_in,      // 미사용 (예약)
    input  wire        h_sync_in,      // 미사용 (예약)
    input  wire        de_in,          // Data Enable

    input  wire [15:0] reg_clip_limit
);

    wire stall       = ~m_axis_ready;  // m_axis_ready=0 → 전체 파이프라인 동결
    // s_axis_valid 미사용: 블랭킹 중 valid=1 시 histogram 오누적 발생
    wire de_in_gated = de_in && !stall;

    // ========== Input Buffer ==========
    wire [31:0] y_quad;   // {Y1_curr, Y0_curr, Y1_prev, Y0_prev}
    wire [15:0] chroma;   // {Cr, Cb}
    wire        de_buf;   // input_buf 통과 후 DE 신호

    input_buf u_input_buf (
        .clk    (clk),
        .rst_n  (rst_n),
        .din    (s_axis_data),
        .de_in  (de_in_gated),
        .y_quad (y_quad),
        .chroma (chroma),
        .de_out (de_buf)
    );

    // ========== 픽셀 좌표 추적 ==========
    // 1클럭 = 2픽셀(Y0+Y1) → curr_x +2씩 증가, 행 끝(1918)에서 줄바꿈
    reg [10:0] curr_x, curr_y;

    always @(posedge clk) begin
        if (!rst_n) begin
            curr_x <= 11'b0;
            curr_y <= 11'b0;
        end else if (de_buf) begin
            if (curr_x == 11'd1918) begin
                curr_x <= 11'b0;
                curr_y <= (curr_y == 11'd1079) ? 11'b0 : curr_y + 1;
            end else begin
                curr_x <= curr_x + 2;
            end
        end
    end

    wire hist_row_done;

    reg [2:0] tile_row_cnt;
    always @(posedge clk) begin
        if (!rst_n)             tile_row_cnt <= 3'b0;
        else if (hist_row_done) tile_row_cnt <= tile_row_cnt + 1;
    end

    // ========== 출력 좌표 (1클럭 지연) ==========
    // Y_prev(y_quad[15:0]) 기준 보간 → curr_x/y 1클럭 지연본으로 ram_cntl 구동
    reg [10:0] out_x, out_y;
    always @(posedge clk) begin
        if (!rst_n) begin
            out_x <= 11'b0;
            out_y <= 11'b0;
        end else if (!stall) begin
            out_x <= curr_x;
            out_y <= curr_y;
        end
    end

    // ========== RAM Control ==========
    wire [5:0]  tile_ul, tile_ur, tile_ll, tile_lr;
    wire [10:0] x_intra_raw, y_intra_raw;

    ram_cntl u_ram_cntl (
        .x        (out_x),
        .y        (out_y),
        .tile_ul  (tile_ul),
        .tile_ur  (tile_ur),
        .tile_ll  (tile_ll),
        .tile_lr  (tile_lr),
        .x_intra  (x_intra_raw),
        .y_intra  (y_intra_raw)
    );

    // [Pipe1] 오프셋 FF — 조합(비교기+감산)과 곱셈 사이 크리티컬 패스 분리
    reg [10:0] x_intra_r, y_intra_r;
    always @(posedge clk) begin
        if (!rst_n) begin
            x_intra_r <= 11'b0;
            y_intra_r <= 11'b0;
        end else if (!stall) begin
            x_intra_r <= x_intra_raw;
            y_intra_r <= y_intra_raw;
        end
    end

    // [Pipe2] 가중치 FF — × 273>>8 ≈ x/240×256, × 243>>7 ≈ y/135×256 → 0~255 정규화
    wire [15:0] wx_product = x_intra_r * 16'd273;
    wire [15:0] wy_product = y_intra_r * 16'd243;
    reg [8:0] weight_x_r, weight_y_r;
    always @(posedge clk) begin
        if (!rst_n) begin
            weight_x_r <= 9'b0;
            weight_y_r <= 9'b0;
        end else if (!stall) begin
            weight_x_r <= {1'b0, wx_product[15:8]};
            weight_y_r <= wy_product[15:7];
        end
    end

    // ========== Histogram ==========
    wire [2:0]  rd_col;
    wire [15:0] hist_data;
    wire [7:0]  rd_bin;
    wire        hist_clear_start;

    histogram_8bank u_histogram (
        .clk             (clk),
        .rst_n           (rst_n),
        .din             (y_quad[31:16]),  // Y_curr만 누적 (Y_prev 제외)
        .de              (de_buf),
        .curr_x          (curr_x),
        .hist_row_done   (hist_row_done),
        .rd_col          (rd_col),
        .rd_bin          (rd_bin),
        .hist_data       (hist_data),
        .hist_clear_start(hist_clear_start)
    );

    // ========== Clipper & CDF Generator ==========
    wire        cdf_wr_en;
    wire [5:0]  cdf_wr_tile;
    wire [7:0]  cdf_wr_bin;
    wire [7:0]  cdf_wr_data;
    wire        frame_cdf_done;

    clipper_8col u_clipper (
        .clk             (clk),
        .rst_n           (rst_n),
        .hist_data       (hist_data),
        .rd_bin          (rd_bin),
        .hist_row_done   (hist_row_done),
        .clip_limit      (reg_clip_limit),
        .tile_row_in     (tile_row_cnt),
        .rd_col          (rd_col),
        .cdf_wr_en       (cdf_wr_en),
        .cdf_wr_tile     (cdf_wr_tile),
        .cdf_wr_bin      (cdf_wr_bin),
        .cdf_wr_data     (cdf_wr_data),
        .hist_clear_start(hist_clear_start),
        .frame_cdf_done  (frame_cdf_done)
    );

    // ========== CDF Store ==========
    wire [7:0] cdf_y0_ul, cdf_y0_ur, cdf_y0_ll, cdf_y0_lr;
    wire [7:0] cdf_y1_ul, cdf_y1_ur, cdf_y1_ll, cdf_y1_lr;

    cdf_store u_cdf_store (
        .clk           (clk),
        .rst_n         (rst_n),
        .stall         (stall),
        .frame_cdf_done(frame_cdf_done),
        .wr_en         (cdf_wr_en),
        .wr_tile       (cdf_wr_tile),
        .wr_bin        (cdf_wr_bin),
        .wr_data       (cdf_wr_data),
        .tile_ul       (tile_ul),
        .tile_ur       (tile_ur),
        .tile_ll       (tile_ll),
        .tile_lr       (tile_lr),
        .pixel_y0      (y_quad[7:0]),    // Y0_prev
        .cdf_y0_ul     (cdf_y0_ul),
        .cdf_y0_ur     (cdf_y0_ur),
        .cdf_y0_ll     (cdf_y0_ll),
        .cdf_y0_lr     (cdf_y0_lr),
        .pixel_y1      (y_quad[15:8]),   // Y1_prev
        .cdf_y1_ul     (cdf_y1_ul),
        .cdf_y1_ur     (cdf_y1_ur),
        .cdf_y1_ll     (cdf_y1_ll),
        .cdf_y1_lr     (cdf_y1_lr)
    );

    // [CDF 정렬 1차] BRAM 출력(T+1) → weight_x_r 도달 시점(T+2)에 맞춰 1클럭 지연
    reg [7:0] cdf_y0_ul_r, cdf_y0_ur_r, cdf_y0_ll_r, cdf_y0_lr_r;
    reg [7:0] cdf_y1_ul_r, cdf_y1_ur_r, cdf_y1_ll_r, cdf_y1_lr_r;
    always @(posedge clk) begin
        if (!stall) begin
            cdf_y0_ul_r <= cdf_y0_ul;  cdf_y0_ur_r <= cdf_y0_ur;
            cdf_y0_ll_r <= cdf_y0_ll;  cdf_y0_lr_r <= cdf_y0_lr;
            cdf_y1_ul_r <= cdf_y1_ul;  cdf_y1_ur_r <= cdf_y1_ur;
            cdf_y1_ll_r <= cdf_y1_ll;  cdf_y1_lr_r <= cdf_y1_lr;
        end
    end

    // [CDF 정렬 2차] weight 경로 Pipe1+Pipe2 = 2클럭 → CDF도 2클럭 지연으로 T+3 정렬 완료
    reg [7:0] cdf_y0_ul_r2, cdf_y0_ur_r2, cdf_y0_ll_r2, cdf_y0_lr_r2;
    reg [7:0] cdf_y1_ul_r2, cdf_y1_ur_r2, cdf_y1_ll_r2, cdf_y1_lr_r2;
    always @(posedge clk) begin
        if (!stall) begin
            cdf_y0_ul_r2 <= cdf_y0_ul_r;  cdf_y0_ur_r2 <= cdf_y0_ur_r;
            cdf_y0_ll_r2 <= cdf_y0_ll_r;  cdf_y0_lr_r2 <= cdf_y0_lr_r;
            cdf_y1_ul_r2 <= cdf_y1_ul_r;  cdf_y1_ur_r2 <= cdf_y1_ur_r;
            cdf_y1_ll_r2 <= cdf_y1_ll_r;  cdf_y1_lr_r2 <= cdf_y1_lr_r;
        end
    end

    // ========== 5단 파이프라인 양선형 보간 ==========

    // [Pipe3] 4방향 가중치 FF — W_UL=(256-wx)(256-wy) 등 (18비트, 합=65536)
    reg [17:0] w_ul_r, w_ur_r, w_ll_r, w_lr_r;
    always @(posedge clk) begin
        if (!stall) begin
            w_ul_r <= (9'd256 - weight_x_r) * (9'd256 - weight_y_r);
            w_ur_r <= weight_x_r            * (9'd256 - weight_y_r);
            w_ll_r <= (9'd256 - weight_x_r) * weight_y_r;
            w_lr_r <= weight_x_r            * weight_y_r;
        end
    end

    // [Pipe4] 부분곱 FF — 가중치(18비트) × CDF(8비트) 8개, DSP 병렬 처리
    reg [23:0] pp_y0_ul_r, pp_y0_ur_r, pp_y0_ll_r, pp_y0_lr_r;
    reg [23:0] pp_y1_ul_r, pp_y1_ur_r, pp_y1_ll_r, pp_y1_lr_r;
    always @(posedge clk) begin
        if (!stall) begin
            pp_y0_ul_r <= w_ul_r * {2'b0, cdf_y0_ul_r2};
            pp_y0_ur_r <= w_ur_r * {2'b0, cdf_y0_ur_r2};
            pp_y0_ll_r <= w_ll_r * {2'b0, cdf_y0_ll_r2};
            pp_y0_lr_r <= w_lr_r * {2'b0, cdf_y0_lr_r2};
            pp_y1_ul_r <= w_ul_r * {2'b0, cdf_y1_ul_r2};
            pp_y1_ur_r <= w_ur_r * {2'b0, cdf_y1_ur_r2};
            pp_y1_ll_r <= w_ll_r * {2'b0, cdf_y1_ll_r2};
            pp_y1_lr_r <= w_lr_r * {2'b0, cdf_y1_lr_r2};
        end
    end

    // [Pipe4 조합] 4방향 합산 → [23:16] 추출 (가중치 합=65536 → ÷65536 = 상위 8비트)
    wire [23:0] y0_sum = pp_y0_ul_r + pp_y0_ur_r + pp_y0_ll_r + pp_y0_lr_r;
    wire [23:0] y1_sum = pp_y1_ul_r + pp_y1_ur_r + pp_y1_ll_r + pp_y1_lr_r;
    wire [7:0] y0_out = y0_sum[23:16];
    wire [7:0] y1_out = y1_sum[23:16];

    // [Pipe5] 출력 FF
    reg [7:0] y0_pipe_r, y1_pipe_r;
    always @(posedge clk) begin
        if (!stall) begin
            y0_pipe_r <= y0_out;
            y1_pipe_r <= y1_out;
        end
    end

    // ========== Chroma / Valid 6클럭 지연 (Y 경로 정렬) ==========
    // shreg_extract="no": SRL16 최적화 시 stall CE 게이팅 불가 → 일반 FF 강제
    (* shreg_extract = "no" *) reg [15:0] chroma_pipe [1:6];
    reg [5:0] valid_pipe;
    integer k;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 1; k <= 6; k = k + 1) chroma_pipe[k] <= 16'b0;
            valid_pipe <= 6'b0;
        end else if (!stall) begin
            chroma_pipe[1] <= chroma;
            for (k = 2; k <= 6; k = k + 1) chroma_pipe[k] <= chroma_pipe[k-1];
            valid_pipe <= {valid_pipe[4:0], de_in_gated};
        end
    end

    // ========== 출력 조립 ==========
    // chroma = {Cr, Cb} → m_axis_data = {Y0, Cb, Y1, Cr}
    assign m_axis_data  = {y0_pipe_r, chroma_pipe[6][7:0],
                           y1_pipe_r, chroma_pipe[6][15:8]};
    assign m_axis_valid = valid_pipe[5];
    assign s_axis_ready = ~stall;

endmodule
