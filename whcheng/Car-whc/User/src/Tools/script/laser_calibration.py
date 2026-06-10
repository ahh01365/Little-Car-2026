#!/usr/bin/env python3
"""
激光测距误差最小二乘法标定脚本
===================================
对采样数据分别做 2 阶和 3 阶多项式拟合，可视化拟合效果，
并输出 C++ 可直接使用的补偿系数。

输入文件格式：两列数据，第一列 = 实际距离(mm)，第二列 = 测量距离(mm)
分隔符支持：空格、逗号、制表符
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from numpy.polynomial import Polynomial

# 设置中文字体（防止中文乱码）
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def read_data(filepath: str):
    """读取数据文件，返回 (actual, measured) 两个 numpy 数组。"""
    try:
        data = np.loadtxt(filepath)
    except ValueError:
        # 尝试逗号分隔
        data = np.loadtxt(filepath, delimiter=',')

    if data.ndim != 2 or data.shape[1] < 2:
        raise ValueError(f"数据格式错误：需要两列数据，实际读取到 {data.shape[1] if data.ndim==2 else 1} 列。")

    actual   = data[:, 0]
    measured = data[:, 1]
    return actual, measured


def fit_and_evaluate(x, y, deg, x_label='x', y_label='y'):
    """
    多项式拟合并评估。
    返回: coeffs(降幂), r2, rmse, y_pred, residuals
    """
    # numpy.polyfit 返回降幂排列系数: [a_n, a_{n-1}, ..., a_0]
    coeffs = np.polyfit(x, y, deg)
    y_pred = np.polyval(coeffs, x)

    # R²
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0

    # RMSE
    rmse = np.sqrt(np.mean((y - y_pred) ** 2))

    # 最大残差
    residuals = y - y_pred
    max_res = np.max(np.abs(residuals))

    return coeffs, r2, rmse, y_pred, residuals, max_res


def format_poly_cpp(coeffs, var_name='d_meas'):
    """
    将多项式系数格式化为 C++ 代码字符串（升幂排列，适合 Horner 求值）。
    coeffs 降幂排列 [a_n, ..., a_0]，对应 a_n*x^n + ... + a_0
    """
    n = len(coeffs) - 1  # 最高阶数
    terms = []
    for i, a in enumerate(coeffs):
        power = n - i
        if power == 0:
            terms.append(f"({a:+.6f}f)")
        elif power == 1:
            terms.append(f"({a:+.6f}f * {var_name})")
        else:
            terms.append(f"({a:+.6f}f * {var_name} * {var_name})" if power == 2 else
                         f"({a:+.6f}f * pow({var_name}, {power}))")

    expr = "".join(terms)
    return f"error = {expr};"


def main():
    # ===== 1. 获取文件路径 =====
    print("=" * 60)
    print("  激光测距误差最小二乘法标定")
    print("=" * 60)
    print()
    print("数据格式要求：两列，第一列=实际距离(mm)，第二列=测量距离(mm)")
    print("支持分隔符：空格 / 逗号 / 制表符")
    print()

    if len(sys.argv) > 1:
        filepath = sys.argv[1]
        print(f"从命令行参数读取路径: {filepath}")
    else:
        filepath = input("请输入数据文件路径: ").strip().strip('"').strip("'")

    if not os.path.isfile(filepath):
        print(f"\n[错误] 文件不存在: {filepath}")
        input("\n按回车键退出...")
        sys.exit(1)

    # ===== 2. 读取数据 =====
    actual, measured = read_data(filepath)
    n = len(actual)
    print(f"\n成功读取 {n} 个数据点。")
    print(f"  实际距离范围: [{actual.min():.1f}, {actual.max():.1f}] mm")
    print(f"  测量距离范围: [{measured.min():.1f}, {measured.max():.1f}] mm")

    # ===== 3. 计算误差 =====
    error = actual - measured  # 正数=测量偏小, 负数=测量偏大

    print(f"  误差范围:     [{error.min():.3f}, {error.max():.3f}] mm")
    print(f"  误差均值:     {error.mean():.3f} mm")
    print(f"  误差标准差:   {error.std():.3f} mm")

    # ===== 4. 误差 = f(测量距离) 拟合 =====
    print("\n" + "-" * 60)
    print("  拟合: 误差 = f(测量距离)")
    print("-" * 60)

    # 2 阶
    c2_err, r2_2, rmse_2, yp2, res2, maxr2 = fit_and_evaluate(
        measured, error, 2, '测量距离', '误差')

    print(f"\n[2阶] R² = {r2_2:.6f}  RMSE = {rmse_2:.4f} mm  最大残差 = {maxr2:.4f} mm")
    print(f"  误差(d) = {c2_err[0]:.8f} * d² + {c2_err[1]:.8f} * d + {c2_err[2]:.8f}")

    # 3 阶
    c3_err, r2_3, rmse_3, yp3, res3, maxr3 = fit_and_evaluate(
        measured, error, 3, '测量距离', '误差')

    print(f"\n[3阶] R² = {r2_3:.6f}  RMSE = {rmse_3:.4f} mm  最大残差 = {maxr3:.4f} mm")
    print(f"  误差(d) = {c3_err[0]:.8f} * d³ + {c3_err[1]:.8f} * d² + {c3_err[2]:.8f} * d + {c3_err[3]:.8f}")

    # ===== 5. 实际值 = f(测量距离) 直接拟合 =====
    print("\n" + "-" * 60)
    print("  拟合: 实际距离 = f(测量距离)  [直接标定]")
    print("-" * 60)

    c2_dir, r2_d2, rmse_d2, ypd2, resd2, maxrd2 = fit_and_evaluate(
        measured, actual, 2, '测量距离', '实际距离')
    c3_dir, r2_d3, rmse_d3, ypd3, resd3, maxrd3 = fit_and_evaluate(
        measured, actual, 3, '测量距离', '实际距离')

    print(f"\n[2阶] R² = {r2_d2:.6f}  RMSE = {rmse_d2:.4f} mm")
    print(f"  actual = {c2_dir[0]:.8f} * d² + {c2_dir[1]:.8f} * d + {c2_dir[2]:.8f}")

    print(f"\n[3阶] R² = {r2_d3:.6f}  RMSE = {rmse_d3:.4f} mm")
    print(f"  actual = {c3_dir[0]:.8f} * d³ + {c3_dir[1]:.8f} * d² + {c3_dir[2]:.8f} * d + {c3_dir[3]:.8f}")

    # ===== 6. C++ 代码输出 =====
    print("\n" + "=" * 60)
    print("  C++ 补偿代码（推荐选 R² 更高、RMSE 更小的方案）")
    print("=" * 60)

    # 推荐方案：比较 4 个拟合的 RMSE
    results = [
        ("误差2阶", c2_err, rmse_2),
        ("误差3阶", c3_err, rmse_3),
        ("直接2阶", c2_dir, rmse_d2),
        ("直接3阶", c3_dir, rmse_d3),
    ]
    best_name, best_coeff, best_rmse = min(results, key=lambda x: x[2])

    print(f"\n推荐方案: {best_name} (RMSE = {best_rmse:.4f} mm)")

    if "误差" in best_name:
        print(f"\n// 误差补偿系数 (误差 = f(测量距离))")
        print(f"// 用法: corrected = measured + error_poly(measured);")
        if len(best_coeff) == 3:  # 2阶
            print(f"#define CAL_A {best_coeff[0]:.8f}f")
            print(f"#define CAL_B {best_coeff[1]:.8f}f")
            print(f"#define CAL_C {best_coeff[2]:.8f}f")
            print(f"")
            print(f"float corrected = measured + (CAL_A * measured * measured + CAL_B * measured + CAL_C);")
        else:  # 3阶
            print(f"#define CAL_A {best_coeff[0]:.8f}f")
            print(f"#define CAL_B {best_coeff[1]:.8f}f")
            print(f"#define CAL_C {best_coeff[2]:.8f}f")
            print(f"#define CAL_D {best_coeff[3]:.8f}f")
            print(f"")
            print(f"float corrected = measured + (CAL_A * measured * measured * measured + CAL_B * measured * measured + CAL_C * measured + CAL_D);")
    else:
        print(f"\n// 直接标定系数 (actual = f(测量距离))")
        print(f"// 用法: corrected = poly(measured);")
        if len(best_coeff) == 3:  # 2阶
            print(f"#define CAL_A {best_coeff[0]:.8f}f")
            print(f"#define CAL_B {best_coeff[1]:.8f}f")
            print(f"#define CAL_C {best_coeff[2]:.8f}f")
            print(f"")
            print(f"float corrected = CAL_A * measured * measured + CAL_B * measured + CAL_C;")
        else:  # 3阶
            print(f"#define CAL_A {best_coeff[0]:.8f}f")
            print(f"#define CAL_B {best_coeff[1]:.8f}f")
            print(f"#define CAL_C {best_coeff[2]:.8f}f")
            print(f"#define CAL_D {best_coeff[3]:.8f}f")
            print(f"")
            print(f"float corrected = CAL_A * measured * measured * measured + CAL_B * measured * measured + CAL_C * measured + CAL_D;")

    # 也输出所有方案的系数供参考
    print(f"\n所有方案系数速查：")
    print(f"  误差2阶: a={c2_err[0]:.8f} b={c2_err[1]:.8f} c={c2_err[2]:.8f}  R²={r2_2:.6f}")
    print(f"  误差3阶: a={c3_err[0]:.8f} b={c3_err[1]:.8f} c={c3_err[2]:.8f} d={c3_err[3]:.8f}  R²={r2_3:.6f}")
    print(f"  直接2阶: a={c2_dir[0]:.8f} b={c2_dir[1]:.8f} c={c2_dir[2]:.8f}  R²={r2_d2:.6f}")
    print(f"  直接3阶: a={c3_dir[0]:.8f} b={c3_dir[1]:.8f} c={c3_dir[2]:.8f} d={c3_dir[3]:.8f}  R²={r2_d3:.6f}")

    # ===== 7. 绘图 =====
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle('激光测距误差最小二乘法标定', fontsize=15, fontweight='bold')

    # ---- 左上：误差 = f(测量距离) 拟合曲线 ----
    ax1 = axes[0, 0]
    x_smooth = np.linspace(measured.min(), measured.max(), 300)

    ax1.scatter(measured, error, s=20, c='black', alpha=0.6, label='采样点', zorder=5)
    ax1.plot(x_smooth, np.polyval(c2_err, x_smooth), 'b-', linewidth=2,
             label=f'2阶拟合 (R²={r2_2:.5f}, RMSE={rmse_2:.3f})')
    ax1.plot(x_smooth, np.polyval(c3_err, x_smooth), 'r--', linewidth=2,
             label=f'3阶拟合 (R²={r2_3:.5f}, RMSE={rmse_3:.3f})')
    ax1.axhline(y=0, color='gray', linestyle=':', linewidth=0.8)
    ax1.set_xlabel('测量距离 (mm)')
    ax1.set_ylabel('误差 = 实际 - 测量 (mm)')
    ax1.set_title('误差拟合曲线: error = f(measured)')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ---- 右上：补偿后残差对比 ----
    ax2 = axes[0, 1]
    ax2.scatter(measured, res2, s=15, c='blue', alpha=0.6, marker='o', label=f'2阶补偿残差 (±{maxr2:.3f}mm)')
    ax2.scatter(measured, res3, s=15, c='red', alpha=0.6, marker='s', label=f'3阶补偿残差 (±{maxr3:.3f}mm)')
    ax2.axhline(y=0, color='gray', linestyle=':', linewidth=0.8)
    ax2.set_xlabel('测量距离 (mm)')
    ax2.set_ylabel('残差 (mm)')
    ax2.set_title('补偿后残差: error模型')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ---- 左下：实际值 vs 测量值 + 直接标定曲线 ----
    ax3 = axes[1, 0]
    ax3.scatter(measured, actual, s=20, c='black', alpha=0.6, label='原始数据', zorder=5)
    ax3.plot(x_smooth, np.polyval(c2_dir, x_smooth), 'b-', linewidth=2,
             label=f'2阶直接标定 (R²={r2_d2:.5f})')
    ax3.plot(x_smooth, np.polyval(c3_dir, x_smooth), 'r--', linewidth=2,
             label=f'3阶直接标定 (R²={r2_d3:.5f})')
    # 理想线 y = x
    ideal = np.linspace(measured.min(), measured.max(), 100)
    ax3.plot(ideal, ideal, 'g:', linewidth=1, alpha=0.5, label='理想 (y=x)')
    ax3.set_xlabel('测量距离 (mm)')
    ax3.set_ylabel('实际距离 (mm)')
    ax3.set_title('直接标定拟合: actual = f(measured)')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ---- 右下：直接标定残差 ----
    ax4 = axes[1, 1]
    ax4.scatter(measured, resd2, s=15, c='blue', alpha=0.6, marker='o', label=f'2阶标定残差 (±{maxrd2:.3f}mm)')
    ax4.scatter(measured, resd3, s=15, c='red', alpha=0.6, marker='s', label=f'3阶标定残差 (±{maxrd3:.3f}mm)')
    ax4.axhline(y=0, color='gray', linestyle=':', linewidth=0.8)
    ax4.set_xlabel('测量距离 (mm)')
    ax4.set_ylabel('残差 (mm)')
    ax4.set_title('补偿后残差: 直接标定模型')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    print("\n[提示] 关闭图形窗口即可退出程序。")
    plt.show()

    input("\n按回车键退出...")


if __name__ == '__main__':
    main()
