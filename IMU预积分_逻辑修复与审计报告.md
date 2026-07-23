# IMU 预积分逻辑修复与审计报告

日期：2026-07-23
分支：`codex/imu-preintegration-audit`

## 结论

本次审计确认，初版预积分分支能够运行，但存在会影响统计解释的结构性问题：
每一关键帧同时使用 SINS 推导的完整速度、姿态、零偏先验和同一段 IMU
构成的预积分因子；窗口最优的速度、姿态和残余零偏没有延续；以及协方差
传播的姿态项、离散噪声尺度与重预积分缓存不一致。

修复后，默认预积分路径使用**窗口首帧完整状态先验、其余帧 GPS 位置先验
和相邻帧 IMU 因子**。优化出的完整 15 状态会保留在重叠帧上并用于下一窗口
的初始化。修复提高了模型逻辑一致性；本次单种子 quick 对比没有显示出统一的
RMSE 改进，因此不能将本次修改宣称为精度提升。

## 根因与处理

| 审计项 | 初版问题 | 修复 |
| --- | --- | --- |
| IMU 信息重复 | 每帧完整 SINS 状态先验与预积分因子同时进入图 | `first_frame_full` 仅锚定每个活动窗口的首帧；后续帧只保留 GPS 位置先验、测距与 IMU 因子 |
| 状态未闭环 | 只输出末帧位置与位置协方差 | 保存每个窗口帧的 `p,v,R,bg,ba`；`state_carryover` 使用重叠帧最优值初始化下一窗口 |
| 零偏反复归零 | 每帧零均值零偏先验 | 零偏仅处于首帧完整先验中，后续通过随机过程残差连接 |
| 噪声双重计入 | `Acc_r` 同时作白噪声和零偏随机过程 | 加速度计白噪声设为零；`Acc_r` 仅映射为偏置过程驱动噪声 |
| 协方差传播 | 姿态转移近似单位阵；连续密度与离散增益重复乘以 `dt` | 使用右扰动的 `Exp(-omega*dt)` 与右雅可比；测量白噪声采用 `density^2/dt`，偏置驱动采用 `density^2*dt` |
| 重预积分 | 局部重算结果未写回因子缓存 | 写回 `obj.imu_factors(index).preint`，记录次数、每因子次数和耗时 |
| 配置开关 | 三个布尔开关即使设为 `false` 也无效果 | 删除误导性默认开关；15 状态、协方差传播和数值雅可比成为明确的固定实现 |
| 协方差提取 | 只有完整 `pinv(A'*A)` | 新增 `current_frame_only` 默认模式，仅解当前帧僚机位置所需列；旧 `full_pinv_legacy` 保留作消融 |
| 雅可比测试 | 仅比较同一中心差分的一列 | 全部 30 列与独立五点差分比较 |

## 状态与反馈模式

`cfg.imu_preint_prior_mode`：

- `first_frame_full`（默认）：首帧使用完整状态先验，后续帧只使用位置先验；
- `all_frames_full_legacy`：保留旧的每帧完整先验，仅用于回归/消融。

`cfg.imu_preint_feedback_mode`：

- `diagnostic`：保存图状态，不作为下一窗口初值；
- `state_carryover`（默认）：重叠帧的图最优 `p,v,R,bg,ba` 成为下一窗口初值；
- `closed_loop`：当前显式拒绝执行。直接把图速度、姿态和残余零偏写回既有
  18 状态 KF/SINS 会遗漏交叉协方差并形成双重更新；在实现严格融合前不能把
  这种松耦合替换设为可用结果。

图中的 `bg`、`ba` 是**已经经过 KF/SINS 修正的 IMU 输入的残余零偏**，不是
完整 IMU 零偏。预积分输入仍是原有 KF/SINS 修正后的角速度和比力。

## 噪声与协方差语义

| 参数 | 来源 | 单位/定义 | 使用方式 |
| --- | --- | --- | --- |
| gyro white | `Gyro_wg`，每 `dt` 样本 10 deg/h | 离散样本标准差；密度为 `sigma/sqrt(dt)` | 角速度测量白噪声 |
| gyro Markov | `Gyro_r`，稳态 10 deg/h，`T=3600 s` | 驱动密度 `sqrt(2/T)*sigma` | 残余陀螺偏置随机游走近似 |
| gyro constant | `Gyro_b`，10 deg/h | 初始不确定度 | 首帧残余偏置先验与 Markov 项合成 |
| accel white | 仿真中不存在独立项 | `0` | 不把 `Acc_r` 重复当成白噪声 |
| accel Markov | `Acc_r`，稳态 `0.001 g`，`T=1800 s` | 驱动密度 `sqrt(2/T)*sigma` | 残余加速度计偏置随机游走近似 |
| 数值正则 | 配置值 `1e-12` | 协方差对角微小正则 | 仅保证 Cholesky 白化正定，不替代物理噪声 |

离散协方差遵循：

```text
P(k+1) = Phi * P(k) * Phi' + G * Q * G'
P = (P + P') / 2
```

其中右扰动姿态项为 `Phi_theta_theta = Exp(-omega*dt)`，
`Phi_theta_bg = -J_r(omega*dt)*dt`。测量白噪声的 `Q` 为
`density^2/dt`，因为其状态增益已含 `dt`；偏置驱动噪声的 `Q` 为
`density^2*dt`，其状态增益为单位阵。

## 修改文件

- `factor_graph_sliding_window_imu_preint.m`：位置先验因子、局部协方差、
  重预积分缓存写回与诊断；
- `imu_preintegrate_interval.m`：右扰动协方差转移和一致的噪声离散化；
- `imu_preintegration_default_noise.m`：与仿真误差模型的明确映射；
- `run_stage1_cusum_comparison.m`：先验结构、状态延续和历史诊断；
- `run_stage1_imu_preintegration_comparison.m`：旧先验/修复先验的同输入对比；
- `stage1_cusum_default_config.m`：默认模式和噪声语义；
- `so3_exp.m`、`so3_log.m`：避免数值雅可比中无条件 SVD 投影；
- `test_imu_preintegration_jacobian.m`：独立五点差分测试；
- 新增静止重力、零残差、偏置、Monte Carlo、状态延续和重预积分缓存测试。

## 实际测试

以下均由 MATLAB R2025b 实际运行并通过：

```matlab
test_stage1_cusum_components
test_stage1_cusum_smoke
test_factor_graph_sliding_window
test_stage1_3f3l_smoke
test_imu_preintegration_zero_motion
test_imu_preintegration_constant_acceleration
test_imu_preintegration_constant_rotation
test_imu_preintegration_partition_consistency
test_imu_preintegration_jacobian
test_imu_preintegration_static_with_gravity
test_imu_preintegration_factor_zero_residual
test_imu_preintegration_bias_recovery
test_imu_preintegration_covariance_monte_carlo
test_imu_preintegration_repropagation_cache
test_stage1_3f3l_imu_preint_smoke
test_stage1_imu_preint_state_carryover
```

Monte Carlo 测试（160 次、1 s 区间）的预测/样本方差比范围为 `0.83–1.15`。

## 同输入 quick 结果

配置：3F3L、seed 23、20 s、故障边 `[2,4]`、5–15 s、`+5 m`。

| 场景 | 方法 | F1 全程 | F2 全程 | F3 全程 | F1 故障窗 | F2 故障窗 | F3 故障窗 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| healthy | 原始 FGO | 0.935 | 0.994 | 0.692 | 0.721 | 0.899 | 0.533 |
| healthy | SINS 滑窗 | 0.975 | 0.994 | 0.714 | 0.846 | 0.996 | 0.577 |
| healthy | 旧先验预积分 | 0.946 | 0.993 | 0.704 | 0.799 | 0.980 | 0.571 |
| healthy | 修复预积分 | 0.948 | 1.002 | 0.707 | 0.792 | 0.972 | 0.576 |
| fault | 原始 FGO | 4.160 | 4.301 | 1.953 | 5.379 | 5.592 | 2.475 |
| fault | SINS 滑窗 | 3.102 | 5.185 | 1.398 | 4.146 | 5.923 | 1.763 |
| fault | 旧先验预积分 | 6.312 | 4.498 | 1.871 | 6.284 | 5.587 | 2.295 |
| fault | 修复预积分 | 6.320 | 4.558 | 1.916 | 6.478 | 5.691 | 2.367 |

quick 运行时间：SINS 对比 `1.796 s`，旧预积分 `3.796 s`，修复预积分
`3.652 s`；修复预积分无未收敛图帧，平均 GN 迭代 `9.350`。

这些数值只说明修复后的逻辑可运行且状态延续测试通过。修复版在此短单种子
故障场景并未统一改善 RMSE，尤其 Follower1 故障窗变差；因此不能以此声称
预积分优于 SINS 滑窗。

## 剩余限制

- 因子雅可比仍以数值差分为主，尚未实现完整解析雅可比；
- 滑动窗口仍没有严格 Schur 边缘化；
- `closed_loop` 未实现，以避免不一致地重复更新现有 KF；
- 图仍使用项目既有 GPS 位置先验，不是完整 GNSS 伪距模型；
- 多种子、多故障边和可复核的 600 s 全量统计尚未完成；本次曾启动指定的
  600 s 单种子命令，但宿主命令监控超时后未保留 MATLAB 标准输出，因此其结果
  不作为已通过验证或实验结论；
- 当前结果不能支持“修复版整体更准”或“修复版整体更快”的结论。
