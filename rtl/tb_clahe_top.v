`timescale 1ns / 1ps

/*
 * tb_clahe_top.v - CLAHE 최상위 영상 테스트벤치 (다중 프레임, YUYV422)
 *
 * 구조  : ping-pong CDF → 프레임 N은 프레임 N-1의 CDF로 변환 (CDF 1프레임 지연)
 * 입력  : input.yuv  (YUYV422 raw, 전 프레임 연속) — $fread로 1장씩 순차 로드
 * 출력  : out.yuv    (YUYV422 raw, 캡처 프레임 연속) — 프레임0(warmup) 제외
 * 색상  : din={Y0,Cb,Y1,Cr} = YUYV422 바이트순서 일치, CbCr 통과 (Y만 보정)
 *
 * 영상 준비:
 *   ffmpeg -i in.mp4 -vf scale=1920:1080 -pix_fmt yuyv422 -f rawvideo input.yuv
 * 결과 재조립:
 *   ffmpeg -f rawvideo -pix_fmt yuyv422 -s 1920x1080 -framerate 60 \
 *          -i out.yuv -c:v libx264 -pix_fmt yuv420p output.mp4
 *
 * 권장 실행 시간: 프레임당 ~11ms → NUM_FRAMES 따라 run 200ms 등
 */

module tb_clahe_top ();

    parameter H_SIZE      = 1920;
    parameter V_SIZE      = 1080;
    parameter NUM_FRAMES  = 120;    // 처리 프레임 수 (2초 @60fps, 입력 프레임 수 이하)
    parameter VBLANK_CLK  = 5000;   // 프레임 간 수직 블랭킹 (clipper 완료 보장, >=4096)

    localparam FRAME_BYTES = H_SIZE * V_SIZE * 2;   // YUYV422 = 2바이트/픽셀

    reg clk;
    reg rst_n;

    // AXI-Stream
    reg  [31:0] s_axis_data;
    reg         s_axis_valid;
    wire        s_axis_ready;
    wire [31:0] m_axis_data;
    wire        m_axis_valid;
    reg         m_axis_ready;   // 항상 1 — 백프레셔 없음

    reg de_in;                  // Data Enable
    reg [15:0] reg_clip_limit;
    reg capture_en;             // 1이면 출력 기록 (프레임1부터)

    integer fd_in, fd_out;      // fd_in = 입력 raw, fd_out = 출력 raw
    integer out_px_count;       // 캡처 픽셀 수 검증용

    reg [7:0] img_mem [0:FRAME_BYTES-1];   // 현재 프레임 1장 (YUYV422)

    clahe_top u_clahe_top (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_axis_data    (s_axis_data),
        .s_axis_valid   (s_axis_valid),
        .s_axis_ready   (s_axis_ready),
        .m_axis_data    (m_axis_data),
        .m_axis_valid   (m_axis_valid),
        .m_axis_ready   (m_axis_ready),
        .v_sync_in      (1'b0),
        .h_sync_in      (1'b0),
        .de_in          (de_in),
        .reg_clip_limit (reg_clip_limit)
    );

    // 100MHz
    initial clk = 1'b0;
    always  #5 clk = ~clk;

    // 출력 캡처 → out.yuv (YUYV422, 4바이트/워드 = {Y0,Cb,Y1,Cr})
    always @(posedge clk) begin
        if (capture_en && m_axis_valid && m_axis_ready) begin
            $fwrite(fd_out, "%c%c%c%c",
                    m_axis_data[31:24], m_axis_data[23:16],
                    m_axis_data[15:8],  m_axis_data[7:0]);
            out_px_count = out_px_count + 2;
        end
    end

    // 프레임 전송: img_mem(YUYV422)에서 1클럭 2픽셀, 행 끝 수평 블랭킹 20클럭
    task send_frame;
        integer fx, fy, off;
        begin
            $display("[%0t] Sending frame...", $time);
            for (fy = 0; fy < V_SIZE; fy = fy + 1) begin
                if (fy % 200 == 0) $display("[%0t]   row %0d / %0d", $time, fy, V_SIZE);

                for (fx = 0; fx < H_SIZE; fx = fx + 2) begin
                    off = fy * H_SIZE * 2 + fx * 2;   // 바이트 오프셋
                    // {Y0, Cb, Y1, Cr}
                    s_axis_data  = {img_mem[off], img_mem[off+1],
                                    img_mem[off+2], img_mem[off+3]};
                    s_axis_valid = 1'b1;
                    de_in        = 1'b1;

                    @(posedge clk);
                    while (!s_axis_ready) @(posedge clk);  // 스톨 시 대기
                end

                // 수평 블랭킹 20클럭
                s_axis_valid = 1'b0;
                de_in        = 1'b0;
                repeat(20) @(posedge clk);
            end

            s_axis_valid = 1'b0;
            de_in        = 1'b0;
            $display("[%0t] Frame sent.", $time);
        end
    endtask

    integer f, nbytes;
    initial begin
        fd_in  = $fopen("input.yuv", "rb");
        fd_out = $fopen("out.yuv",   "wb");
        if (fd_in == 0) begin
            $display("[ERROR] input.yuv 없음 — ffmpeg로 YUYV422 raw 생성 후 작업 디렉토리에 배치");
            $finish;
        end

        rst_n        = 1'b0;
        s_axis_valid = 1'b0;
        de_in        = 1'b0;
        m_axis_ready = 1'b1;
        capture_en   = 1'b0;
        out_px_count = 0;

        // clip_limit: 평균빈(~127)의 약 3배 — 자연스러운 대비 (낮으면 과평활, 높으면 약효)
        reg_clip_limit = 16'd400;

        #100 rst_n = 1'b1;
        $display("[%0t] Reset released", $time);
        #50;

        for (f = 0; f < NUM_FRAMES; f = f + 1) begin : frame_loop
            nbytes = $fread(img_mem, fd_in);   // 다음 프레임 1장 로드
            if (nbytes < FRAME_BYTES) begin
                $display("[TB] 입력 끝 (frame %0d, %0d / %0d bytes)", f, nbytes, FRAME_BYTES);
                f = NUM_FRAMES;                 // 루프 종료
            end else begin
                // 프레임0은 CDF 웜업 → 캡처 안 함 (출력은 프레임1부터)
                capture_en = (f >= 1);
                $display("[%0t] === Frame %0d (%s) ===", $time, f,
                         (f == 0) ? "warmup" : "capture");
                send_frame;

                // 수직 블랭킹: 다음 프레임 전 clipper/CDF 완료 보장
                repeat(VBLANK_CLK) @(posedge clk);
            end
        end

        $fclose(fd_in);
        $fclose(fd_out);

        $display("[%0t] === DONE ===", $time);
        // 정상이면 out_px_count == (NUM_FRAMES-1) * H_SIZE * V_SIZE
        $display("  captured pixels : %0d", out_px_count);
        $finish;
    end

    // VCD: 활성화 시 디스크 I/O로 시뮬 프리징 가능 — 필요 시만
    // initial begin
    //     $dumpfile("clahe_top.vcd");
    //     $dumpvars(0, tb_clahe_top);
    // end

endmodule
