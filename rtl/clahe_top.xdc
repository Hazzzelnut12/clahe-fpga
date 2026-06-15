###############################################################################
# clahe_top.xdc - CLAHE 가속기 타이밍 제약
#
# Target : Zynq-7000 (XC7Z020), 100MHz
# Scope  : STA 전용 (비트스트림 생성 없음)
###############################################################################

# 100MHz 클럭
create_clock -period 10.000 -name clk [get_ports clk]
set_clock_uncertainty -setup 0.200 [get_clocks clk]
set_clock_uncertainty -hold  0.050 [get_clocks clk]

# 비동기 리셋
set_false_path -from [get_ports rst_n]

# PS-PL AXI 포트: 타이밍 PS 관리 → STA 제외
set_false_path -from [get_ports {s_axis_data[*] s_axis_valid m_axis_ready}]
set_false_path -to   [get_ports {m_axis_data[*] m_axis_valid s_axis_ready}]
set_false_path -from [get_ports {de_in v_sync_in h_sync_in}]
set_false_path -from [get_ports {reg_clip_limit[*]}]
