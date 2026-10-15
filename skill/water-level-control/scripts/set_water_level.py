#!/usr/bin/env python3
import sys
import os
import subprocess

# 目标设备无 pip, water_control_cli.py 未安装为 water-control 命令。
# 直接用当前 python 解释器调用同级目录的 water_control_cli.py。
_CLI = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "water_control_cli.py")
_CMD = [sys.executable, _CLI]

def main():
    if len(sys.argv) != 2:
        print("Usage: set_water_level <target_level_mm>")
        print("Example: set_water_level 60")
        sys.exit(1)

    try:
        target_mm = float(sys.argv[1])
    except ValueError:
        print("Error: target level must be a number (mm)")
        sys.exit(1)

    # 支持的控制范围: 50-150mm。超出此区间拒绝, 不执行任何 PLC 写入。
    # PID 增益仅在此窗口内现场整定过; 区间外控制不稳定且可能无意义。
    SUPPORTED_MIN, SUPPORTED_MAX = 50.0, 150.0
    if not (SUPPORTED_MIN <= target_mm <= SUPPORTED_MAX):
        print(f"Error: target {target_mm:g} mm out of supported range "
              f"({SUPPORTED_MIN:g}-{SUPPORTED_MAX:g} mm). "
              f"water-level-control only supports 50-150 mm.")
        sys.exit(1)

    # 分段PID参数(现场实测整定): Kp=3.0, Kd=0 全程不变; Ki 随目标高度线性变化。
    # 实测点: 50mm -> Ki=0.49,  150mm -> Ki=0.4
    # 线性拟合: Ki = 0.535 - 0.0009 * mm  (目标越高 Ki 越小, 抑制高水位震荡)
    # 适用范围 50-150mm(已在上方校验), Ki 钳制到 [0.40, 0.49]。
    kp, kd = 3.0, 0.0
    ki = round(0.535 - 0.0009 * target_mm, 3)
    ki = max(0.40, min(0.49, ki))

    # 静默执行参数设置和水位设置，只捕获错误
    try:
        subprocess.run(_CMD + ["set-pid", str(kp), str(ki), str(kd)], check=True, capture_output=True)
        subprocess.run(_CMD + ["set-mm", str(target_mm)], check=True, capture_output=True)
        print(f"✅ 已成功将水位目标设置为 {target_mm} mm")
    except subprocess.CalledProcessError as e:
        print(f"❌ 设置失败，请检查PLC连接")
        sys.exit(1)

if __name__ == "__main__":
    main()
