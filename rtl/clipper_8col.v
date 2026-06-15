`timescale 1ns/1ps

/*
 * clipper_8col.v - 8열 순차 클리핑 + CDF 생성 FSM
 *
 * ST_CLIP     (256클럭): clip_limit 초과분 합산
 * ST_CDF_WRITE(256클럭): 재분배 → CDF 누적 → 정규화 → BRAM 기록
 * 8열 × 512클럭 ≈ 4,112클럭 (타일 행 주기 ~129,600클럭 내)
 *
 * bin_idx 1클럭 선행 요청, proc_idx/bin_data로 지연 처리 (LUTRAM 조합 지연 흡수)
 */

module clipper_8col (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [15:0] hist_data,      // 요청 빈 카운트 (비동기 LUTRAM)
    output wire [7:0]  rd_bin,         // 빈 인덱스 선행 요청 (1클럭 앞)
    input  wire        hist_row_done,  // 타일 행 완료 → 처리 시작

    input  wire [15:0] clip_limit,     // 클리핑 한계값 (권장 400~600)
    input  wire [2:0]  tile_row_in,    // 현재 타일 행 번호 (0~7)

    output reg  [2:0]  rd_col,         // histogram 현재 처리 열

    output reg         cdf_wr_en,
    output reg  [5:0]  cdf_wr_tile,    // {tile_row[2:0], col_idx[2:0]}
    output reg  [7:0]  cdf_wr_bin,
    output reg  [7:0]  cdf_wr_data,    // 정규화 CDF (0~255)

    output wire        hist_clear_start,
    output wire        frame_cdf_done  // 행 7 완료 → cdf_store page 전환
);

    localparam NUM_COLS = 8;

    localparam ST_IDLE      = 3'd0;
    localparam ST_CLIP      = 3'd1;
    localparam ST_CDF_WRITE = 3'd2;
    localparam ST_NEXT_COL  = 3'd3;
    localparam ST_DONE      = 3'd4;

    reg [2:0]  state;
    reg [8:0]  bin_idx;       // 선행 요청용 (0~256, 9비트 — 256 포함)
    reg [7:0]  proc_idx;      // bin_idx 1클럭 지연: 실제 연산 대상 빈
    reg [2:0]  col_idx;
    reg [2:0]  tile_row_reg;
    reg [19:0] excess_total; // 초과분 총합 (최대 32,400 → 15비트, 20비트 마진)
    reg [23:0] cdf_accum;       // CDF 누적기 (배열 없이 순차 처리 — 합성 병목 제거)
    reg [15:0] bin_data;       // hist_data 1클럭 지연 샘플

    // WNS -0.148ns 해소: norm_calc 경로를 1클럭 분리
    reg [23:0] cdf_next_ff;
    reg [7:0]  proc_idx_r;
    reg        wr_active;
    reg        wr_active_r;

    wire [15:0] redist_step = excess_total[19:8]; // excess / 256
    wire [7:0]  redist_rem  = excess_total[7:0];  // excess % 256

    wire [15:0] clipped_val    = (bin_data > clip_limit) ? clip_limit[15:0] : bin_data;
    wire [15:0] redist_val     = clipped_val + redist_step
                               + ((proc_idx < redist_rem) ? 1'b1 : 1'b0);
    wire [23:0] next_cdf_combo = cdf_accum + redist_val;

    // 516/65536 ≈ 255/32400 (타일 픽셀 최대 32,400 기준 정규화)
    wire [31:0] norm_calc = ({8'b0, cdf_next_ff} * 32'd516) >> 16;

    assign rd_bin          = bin_idx[7:0]; // 256 도달 시 하위 8비트 = 0으로 순환
    assign hist_clear_start = (state == ST_DONE);
    assign frame_cdf_done   = (state == ST_DONE) && (tile_row_reg == 3'd7);

    always @(posedge clk) begin
        if (!rst_n) begin
            state            <= ST_IDLE;
            bin_idx          <= 9'b0;
            proc_idx         <= 8'b0;
            col_idx          <= 3'b0;
            tile_row_reg     <= 3'b0;
            excess_total    <= 20'b0;
            cdf_accum          <= 24'b0;
            bin_data          <= 16'b0;
            rd_col         <= 3'b0;
            cdf_wr_en        <= 1'b0;
            cdf_wr_tile      <= 6'b0;
            cdf_wr_bin       <= 8'b0;
            cdf_wr_data      <= 8'b0;
            cdf_next_ff       <= 24'b0;
            proc_idx_r       <= 8'b0;
            wr_active        <= 1'b0;
            wr_active_r      <= 1'b0;
        end else begin
            proc_idx    <= bin_idx[7:0];
            bin_data     <= hist_data;
            cdf_next_ff  <= next_cdf_combo;
            proc_idx_r  <= proc_idx;
            wr_active_r <= wr_active;
            wr_active   <= 1'b0;

            cdf_wr_en <= 1'b0;

            // bin 255 플러시: ST_CDF_WRITE→ST_NEXT_COL 전환 후 1클럭 더 유지
            if (wr_active_r) begin
                cdf_wr_en   <= 1'b1;
                cdf_wr_tile <= {tile_row_reg, col_idx};
                cdf_wr_bin  <= proc_idx_r;
                cdf_wr_data <= (norm_calc > 32'd255) ? 8'd255 : norm_calc[7:0];
            end

            case (state)
                ST_IDLE: begin
                    if (hist_row_done) begin
                        state         <= ST_CLIP;
                        bin_idx       <= 9'b0;
                        col_idx       <= 3'b0;
                        tile_row_reg  <= tile_row_in;
                        excess_total <= 20'b0;
                        rd_col      <= 3'b0;
                    end
                end

                ST_CLIP: begin
                    if (bin_idx < 9'd256)
                        bin_idx <= bin_idx + 1;

                    if (bin_idx > 0) begin  // 첫 클럭: 파이프라인 준비, 무시
                        if (bin_data > clip_limit)
                            excess_total <= excess_total + (bin_data - clip_limit);
                    end

                    if (bin_idx == 9'd256) begin
                        bin_idx <= 9'b0;
                        cdf_accum <= 24'b0;
                        state   <= ST_CDF_WRITE;
                    end
                end

                ST_CDF_WRITE: begin
                    if (bin_idx < 9'd256)
                        bin_idx <= bin_idx + 1;

                    if (bin_idx > 0) begin  // 첫 클럭: 파이프라인 준비, 무시
                        cdf_accum   <= next_cdf_combo;
                        wr_active <= 1'b1;
                    end

                    if (bin_idx == 9'd256)
                        state <= ST_NEXT_COL;
                end

                ST_NEXT_COL: begin
                    if (col_idx == NUM_COLS - 1) begin
                        col_idx <= 3'b0;
                        state   <= ST_DONE;
                    end else begin
                        col_idx       <= col_idx + 1;
                        rd_col      <= col_idx + 1;
                        excess_total <= 20'b0;
                        bin_idx       <= 9'b0;
                        cdf_accum       <= 24'b0;
                        state         <= ST_CLIP;
                    end
                end

                ST_DONE: begin
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
