# 第一阶段 CUSUM 关键问题修复报告

本报告取代 `STAGE1_CUSUM_IMPLEMENTATION_REPORT.md` 中修复前的数值结果。修复前结果不得与本报告结果混合使用。

## 1. MATLAB 实际调用文件

已运行：

```matlab
which factor_graph_centralization -all
which residual_cal -all
```

实际调用文件为工程根目录中的：

- `factor_graph_centralization.m`
- `residual_cal.m`

未发现带编号的 `factor_graph_centralization` 副本参与调用。

## 2. 修复前问题与修复位置

| 问题 | 修复文件与位置 | 修复内容 |
| --- | --- | --- |
| CUSUM 使用 GPS 图先验 | `run_stage1_cusum_comparison.m`，图优化前的 CUSUM 输入构造处 | 僚机改用 KF 时间更新后的 `posiN_w_all`，经 `posical_xyz` 转为 ENU 米制；长机继续使用高精度 GPS ENU 先验。 |
| 僚机协方差固定为 GPS 方差 | 同上，`local_kf_position_covariance_enu` | 从 `PK_all{vehicle}(7:9,7:9)` 提取并转为 ENU 米制协方差。 |
| 三轴陀螺修正错误 | `run_stage1_cusum_comparison.m` | 从标量 `Xc(10)+Xc(13)` 改为三轴向量 `Xc(10:12)+Xc(13:15)`。 |
| GN 负步长误收敛 | `factor_graph_centralization.m` | 改为 `norm(delta_X,inf)<thresh`，增加 `iteration_count`、`final_step_norm`、`converged`。 |
| 故障/健康边统计窗口不一致 | `run_stage1_cusum_comparison.m` 的 `local_diagnostics` | 故障边、其他健康边和比值均改为同一故障窗口；额外保存其他健康边全时段中位权重。 |
| quick 故障仍在暖机 | `stage1_cusum_default_config.m` | 改为 `t_stop=20`、`warmup=2`、`fault=5--12 s`。 |
| Equal 仅做重复性测试 | `run_stage1_cusum_comparison.m`、`test_stage1_cusum_smoke.m` | 增加最小原始 Equal 分支，跳过 CUSUM 调用并使用同一缓存输入；比较导航、RMSE、图位置和图协方差。 |

## 3. 18 状态与坐标/协方差转换依据

`kalm_factor_init.m` 的初始状态注释和 `Xerr` 输出表明：

- 状态 `7:9`：纬度误差（rad）、经度误差（rad）、高度误差（m）；
- 状态 `10:12`：三轴陀螺常值误差；
- 状态 `13:15`：三轴陀螺相关/随机漂移误差；
- 状态 `16:18`：三轴加速度计误差。

`posical_xyz` 输出顺序为 ENU 米制 `[East, North, Up]`。因此对 `PK(7:9,7:9)` 使用：

```matlab
J = [0, (Rn+h)*cos(latitude), 0;
     (Rm+h), 0, 0;
     0, 0, 1];
P_enu = J * PK(7:9,7:9) * J';
```

随后显式对称化。在线 CUSUM 函数还会验证位置/协方差有限、实数、维度正确、协方差对称且半正定。在线接口不包含真值、故障边、故障幅值或故障时间。

## 4. 当前历元数据流

1. KF 时间更新，获得僚机 `posiN_w_all` 与 `PK_all`；
2. 僚机 SINS/KF 预测与长机高精度 GPS 先验组成 CUSUM 创新输入；
3. 当前测距计算带符号标准化创新、双边 CUSUM 和当前权重；
4. 原始因子图仍使用 GPS 图先验构造测距因子；仅测距残差/Jacobian 乘一次 `sqrt(weight)`；
5. GN 重线性化从原始白化测距残差重新计算后再乘一次 `sqrt(weight)`；
6. 图优化输出经过原有 KF/SINS 反馈。

没有使用后验图状态反算当前 CUSUM；没有重复加权。

## 5. 测试结果

- `test_stage1_cusum_components`: PASS。
- `test_stage1_cusum_smoke`: PASS。
- quick 配置确认 `2 < 5 < 12 < 20`，故障处于暖机后；故障 CUSUM 累积、故障边权重下降且检测延迟有限。
- 确定性重复性测试：PASS。
- 原始 Equal 回归测试：PASS。

原始 Equal 与新框架 `cusum_apply=false` 的最大差异：

| 对比项 | 最大绝对差异 |
| --- | ---: |
| 最终导航轨迹 | 0 |
| 逐僚机全时段/故障窗口 RMSE | 0 |
| 因子图僚机位置 | 0 |
| 因子图僚机协方差 | 0 |

## 6. 完整 600 s 重新实验

配置：故障边 `(2,3)`，`+5 m`，100--150 s；CUSUM 参数保持原值：`lambda=0.90`、`kappa=0.25`、`h=4.0`、`w_min=0.05`。

### 单种子 23

| 场景 | 方法 | 僚机 1 全时段 / 窗口 RMSE (m) | 僚机 2 全时段 / 窗口 RMSE (m) |
| --- | --- | ---: | ---: |
| healthy | Equal | 1.437911 / 1.497488 | 3.262818 / 3.582973 |
| healthy | CUSUM | 2.140867 / 2.358650 | 5.672532 / 6.413867 |
| fault | Equal | 7.439554 / 24.608395 | 9.940269 / 31.528325 |
| fault | CUSUM | 7.076266 / 22.531551 | 10.803857 / 29.236865 |

故障窗口权重：目标边 `0.075640`，其他健康边 `0.183680`，比值 `0.411804`。GN 所有历元均收敛，最终步长范数约 `1e-5`。

### 五种子 `[1,17,23,41,73]` 平均结果

| 场景 | 方法 | 僚机 1 全时段 / 窗口 RMSE (m) | 僚机 2 全时段 / 窗口 RMSE (m) |
| --- | --- | ---: | ---: |
| healthy | Equal | 1.686645 / 1.624929 | 1.780828 / 2.091854 |
| healthy | CUSUM | 2.591812 / 2.487292 | 2.633018 / 3.197571 |
| fault | Equal | 7.293414 / 23.822572 | 8.397292 / 27.047911 |
| fault | CUSUM | 6.711707 / 20.399595 | 6.643265 / 18.869792 |

故障窗口相对 Equal 的平均变化：僚机 1 `-14.37%`，僚机 2 `-30.24%`。

CUSUM-FGO 最差全时段种子：healthy 僚机 1 为种子 41（3.590355 m）、healthy 僚机 2 为种子 23（5.672532 m）；fault 僚机 1 为种子 41（7.317756 m）、fault 僚机 2 为种子 23（10.803857 m）。标准差、逐种子诊断和 GN 统计由命令行表格输出。

## 7. 验收状态与限制

- 代码正确性修复、快速检测覆盖、GN 收敛、Equal 回归：通过。
- 故障五种子平均故障窗口 RMSE：两架僚机均改善。
- 健康场景：**未通过**。修复后的正确 KF/SINS 创新在当前默认 CUSUM 参数下仍出现较多健康边降权，健康 RMSE 恶化约 48--54%。
- 部分故障种子的阈值检测显示 `not detected`，原因是故障前健康权重已低于相应阈值，未发生新的阈值跨越；这不是以 0 填充。

按本修复任务要求，未修改任何 CUSUM 参数以掩盖该现象。后续如需优化，必须另起调参阶段，并只在公开 CUSUM 参数上进行。

## 8. 范围确认

未加入 FI/KLD、滑动窗口、IMU 预积分、测距偏差状态、Oracle 权重、新鲁棒核、硬删边或分布式优化。未改变轨迹、UAV 数量、通信范围、正常测距噪声、故障模型或随机种子设置。测距仅加权一次。
