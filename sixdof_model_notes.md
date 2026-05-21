# 600 kg 级飞行器六自由度模型说明

本文档说明当前项目中的基准六自由度模型。模型实现位于 `matlab/` 目录，遵循 `research_conventions.md` 中的坐标系、速度分量和欧拉角约定。

## 1. 模型定位

该模型用于总重约 `600 kg`、最大飞行速度约 `M0.4` 的飞行器制导控制律研究。

当前版本是基准工程模型，重点是提供完整、可运行、可替换参数的六自由度动力学框架。气动导数、执行机构、发动机模型和结构参数均为初始估计值，后续应根据气动数据、总体参数或辨识结果更新。

## 2. 状态量定义

状态向量定义为：

```text
x = [Xg; Yg; Zg; Vx; Vy; Vz; k_theta; psi; gama; wx; wy; wz]
```

其中：

- `Xg, Yg, Zg` 为地面坐标系位置，单位为 `m`。
- `Vx, Vy, Vz` 为机体系速度分量，单位为 `m/s`。
- `k_theta, psi, gama` 为 `YZX` 欧拉角，变量顺序按教材写为 `[ϑ, ψ, γ]`，单位为 `rad`。
- `wx, wy, wz` 为机体系角速度，单位为 `rad/s`。

模型采用 `C_bg` 表示从机体系到地面坐标系的方向余弦矩阵。

## 3. 控制输入

当前气动/推力模型支持如下控制输入：

```text
control.de      升降舵，rad
control.da      副翼，rad
control.dr      方向舵，rad
control.thrust  推力，N
```

其中 `de` 正向定义为产生正俯仰力矩，`da` 正向定义为产生正滚转力矩，`dr` 正向定义为产生正偏航力矩。

## 4. 攻角、侧滑角与速度坐标系

攻角 `alpha` 和侧滑角 `beta` 按《有翼导弹飞行动力学》中的定义实现。机体系速度满足：

```text
V_b = V * [cos(alpha)*cos(beta);
           sin(alpha)*cos(beta);
          -sin(beta)]
```

因此：

```text
alpha = atan2(Vy, Vx)
beta  = -asin(Vz / V)
```

速度坐标系 `Ox3y3z3` 中，`Ox3` 与速度矢量重合，`Oy3` 为升力正向，`Oz3` 为侧向力正向。阻力 `X` 正值方向与 `Ox3` 负向一致，因此气动力在速度坐标系中的分量写作：

```text
F_aero_v = [-X; Y; Z]
```

从速度坐标系到机体系的方向余弦矩阵由 `dcm_velocity_to_body.m` 给出。

## 5. 动力学方程

机体系平动方程为：

```text
Vdot_b = F_b / m + C_gb * g_g - omega_b x V_b
```

转动方程为：

```text
omega_dot_b = I^-1 * (M_b - omega_b x I * omega_b)
```

位置运动学为：

```text
rdot_g = C_bg * V_b
```

姿态运动学采用 `YZX` 欧拉角关系，由 `euler_rates_yzx.m` 计算。欧拉角速率输出顺序为 `[d k_theta; d psi; d gama]`。

## 6. 当前参数状态

当前基准参数如下：

- 质量：`600 kg`。
- 配平速度：`0.4 Mach`，约 `136.1 m/s`。
- 配平攻角：约 `4 deg`，平飞配平俯仰角 `k_theta = -alpha`。
- 初始高度：`1000 m`。
- 参考面积：`1.20 m^2`。
- 参考展长：`2.00 m`。
- 参考弦长：`0.90 m`。
- 惯量矩阵：`diag([120, 480, 520]) kg*m^2`。

## 7. 文件说明

- `matlab/aircraft600_params.m`：模型参数、气动导数、配平点。
- `matlab/aero_angles_from_body_velocity.m`：按教材定义计算攻角与侧滑角。
- `matlab/dcm_velocity_to_body.m`：速度坐标系到机体系的方向余弦矩阵。
- `matlab/sixdof_initial_state.m`：初始状态。
- `matlab/sixdof_eom.m`：六自由度方程。
- `matlab/aero_prop_model.m`：气动与推力模型。
- `matlab/dcm_body_to_ground_yzx.m`：`YZX` 欧拉角方向余弦矩阵。
- `matlab/euler_rates_yzx.m`：`YZX` 欧拉角速率转换。
- `matlab/sixdof_postprocess.m`：仿真后处理。
- `matlab/check_sixdof_model.m`：轻量一致性检查。
- `matlab/run_sixdof_baseline.m`：基准仿真入口。

## 8. 运行方式

在 MATLAB 中进入项目根目录后运行：

```matlab
addpath('matlab');
check_sixdof_model;
run_sixdof_baseline;
```

也可以在命令行运行：

```text
matlab -batch "addpath('matlab'); check_sixdof_model; run_sixdof_baseline"
```
