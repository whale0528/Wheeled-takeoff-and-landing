# 无人机着陆阶段实现指南

> 基于论文《无人机进场着陆/地面滑跑控制与仿真》（胡浩，2011，电子科技大学）
> 结合当前 `planmodel.slx` 已有架构

---

## 一、当前状态概览

### 1.1 已完成的工作

| 子系统 | 状态 | 说明 |
|--------|------|------|
| 6DOF 动力学 | ✅ 完成 | body-frame 12状态非线性模型 |
| 气动模型 | ✅ 完成 | 升力/阻力/侧力/力矩系数完整 |
| 起落架模型 | ✅ 完成 | 弹簧阻尼+轮胎力+摩擦圆+刹车 |
| 起飞控制 | ✅ 完成 | 地面滑跑→抬轮(3°/s)→爬升过渡 |
| 巡航控制 | ✅ 完成 | 高度保持350m + 推力PID维持200m/s |
| 着陆轨迹设计 | ✅ 完成 | `landing_trajectory.m` 五段几何推导 |
| 着陆俯仰控制骨架 | ⚠️ 半成品 | `blk_4460` 有PI+nz/q阻尼结构但未调参/未连接 |
| 地面纠偏控制 | ✅ 完成 | 前轮转向控制跑道中心线 |

### 1.2 关键信号路由

所有信号通过**全局 Goto/From 标签**传递（共99个标签可用）：

| GotoTag | 含义 | 来源 |
|---------|------|------|
| `X_positon` | 飞机 X 坐标 (前向) | 动力学/Subsystem3 |
| `height` | 离地高度 (m) | 动力学/height |
| `Vg` | 地速 (m/s) | 动力学/Subsystem3 |
| `vzg` | 垂直速度 (m/s) | 动力学/height |
| `nz` | 法向过载 (g) | 动力学/dynamics EQ |
| `q` | 俯仰角速率 (rad/s) | 动力学/rotation EQ |
| `theta_eular` | 俯仰角 (rad) | 动力学/姿态角 |
| `contact_nose` | 前轮接地标志 | 起落架模型 |
| `contact_main` | 主轮接地标志 | 起落架模型 |
| `no_contact` | 全机离地标志 | 起落架模型 |
| `de` | 升降舵指令 | 俯仰通道 |
| `fly_height` | 高度>10m 标志 | control |
| `start_fly` | 高度>800m 巡航标志 | control |

### 1.3 blk_4460 (着陆俯仰控制) 现有骨架分析

```
输入: Vg (地速), Hcmd (高度指令)
内部信号流:
  Vz_ref  = sin(gamma1) * Vg           [下滑道垂直速度基准]
  Vz_error = Vz_ref + vzg              [vzg 向下为正, Gain=-1取反]
  H_error  = Hcmd - height             [高度误差]
  de_raw   = Vz_error + H_error - nz - q   [综合误差 + 阻尼]
  de       = K_total * de_raw + ∫ de_raw    [PI 控制器]
输出: de (升降舵指令)
```

**现有问题**：
1. 所有增益=1，未调谐
2. gamma1 固定不变（应用随轨迹段动态变化）
3. 积分器无复位逻辑
4. 输出无饱和限幅
5. 已连接到主 de 输出（Outport 已存在）

---

## 二、着陆轨迹设计（几何参考）

来自 `landing_trajectory.m`:

```
设计参数:
  X_zero = -3000 m    — 陡下滑线地面交点
  gamma1 = -5°        — 陡下滑角
  gamma2 = -2°        — 浅下滑角
  R      = 8000 m     — 圆弧拉平半径
  X_aim  = -500 m     — 接地点
  H_exp  = 20 m       — 拉飘起始高度
  sigma  = 6 s        — 拉飘时间常数
  H_entry = 350 m     — 进场高度

关键节点:
  X_entry ≈ -3714 m — 进场点 (H=350m)
  X_B     ≈ -2527 m — 陡下滑→圆弧切点
  X_C     ≈ -1950 m — 圆弧→浅下滑切点
  X_D     = -500 m  — 拉飘起始点 (H=20m)

五段轨迹:
  Phase ① 定高平飞:  H_cmd = H_entry            (X ≤ X_entry)
  Phase ② 陡下滑:    H_cmd = -(X-X_zero)*tan(a1) (X_entry < X ≤ X_B)
  Phase ③ 圆弧拉平:  H_cmd = H_R - √(R²-(X-X_R)²) (X_B < X ≤ X_C)
  Phase ④ 浅下滑:    H_cmd = (X_aim-X)*tan(a2)   (X_C < X ≤ X_D)
  Phase ⑤ 指数拉飘:  H_cmd = H_exp*exp(-t/6)     (X > X_D, H > 0.5)
```

---

## 三、分步实现

### 步骤 1：着陆轨迹指令生成器

**目标**：在 `planmodel/control` 内创建 MATLAB Function 块，根据飞机 X 位置实时输出 H_cmd。

#### 1.1 添加 From 块

在 `planmodel/control` 内添加:
- From 块，GotoTag = `"X_positon"`，命名为 `From_X`

#### 1.2 创建 MATLAB Function 块

```
块名: Landing_Trajectory_Generator
输入: X (X坐标, m), H_current (当前高度, m)
输出: H_cmd (高度指令, m), phase (轨迹段号 0-5)
```

MATLAB Function 代码:

```matlab
function [H_cmd, phase] = landing_trajectory(X, H_current)
%#codegen
% 着陆轨迹生成器 - 5段轨迹
% Phase: 1=定高 2=陡下滑 3=圆弧 4=浅下滑 5=拉飘 0=接地

persistent X_zero gamma1 gamma2 R X_aim H_exp sigma H_entry a1 a2
persistent X_R H_R X_B H_C X_C X_D X_entry
persistent init_done t_flare

if isempty(init_done)
    % 设计参数（与 landing_trajectory.m 一致）
    X_zero  = -3000;
    gamma1  = -5 * pi/180;
    gamma2  = -2 * pi/180;
    R       = 8000;
    X_aim   = -500;
    H_exp   = 20;
    sigma   = 6;
    H_entry = 350;
    
    a1 = abs(gamma1);
    a2 = abs(gamma2);
    
    % 几何推导
    M = [tan(a1), 1; tan(a2), 1];
    bv = [tan(a1)*X_zero + R/cos(a1); tan(a2)*X_aim + R/cos(a2)];
    sol = M \ bv;
    X_R = sol(1);  H_R = sol(2);
    X_B = X_R - R * sin(a1);
    X_C = X_R - R * sin(a2);
    X_D = X_aim - H_exp / tan(a2);
    X_entry = X_zero - H_entry / tan(a1);
    
    t_flare = 0;
    init_done = true;
end

% 五段分段函数
if X <= X_entry
    H_cmd = H_entry;
    phase = 1;
    t_flare = 0;  % 复位拉飘计时器
elseif X <= X_B
    H_cmd = -(X - X_zero) * tan(a1);
    phase = 2;
    t_flare = 0;
elseif X <= X_C
    dx = X - X_R;
    arg = R^2 - dx^2;
    if arg > 0
        H_cmd = H_R - sqrt(arg);
    else
        H_cmd = 0;
    end
    phase = 3;
    t_flare = 0;
elseif X <= X_D
    H_cmd = (X_aim - X) * tan(a2);
    phase = 4;
    t_flare = 0;
elseif H_current > 0.5
    Ts = 0.001;  % 与 Solver 步长一致
    t_flare = t_flare + Ts;
    H_cmd = H_exp * exp(-t_flare / sigma);
    if H_cmd < 0
        H_cmd = 0;
    end
    phase = 5;
else
    H_cmd = 0;
    phase = 0;
end
end
```

#### 1.3 添加 Goto 块

- Goto 块，Tag = `"landing_phase"`, Visibility = `global`
- Goto 块，Tag = `"H_cmd_landing"`, Visibility = `global`

#### 1.4 连接关系

```
From_X (X_positon) → traj_gen.X
From_height          → traj_gen.H_current
traj_gen.H_cmd       → goto_H_cmd_landing
traj_gen.phase       → goto_landing_phase
```

---

### 步骤 2：完善着陆俯仰控制器 (blk_4460)

**目标**：调谐增益、动态化 gamma1、添加积分器复位和输出饱和。

#### 2.1 调谐增益

在 blk_4460 内修改以下 Gain 块的 Value:

| 块 | 当前值 | 建议值 | 含义 |
|----|--------|--------|------|
| Gain1 (blk_4487) | 1 | `Kp_Vz = 0.15` | Vz误差比例增益 |
| Gain2 (blk_4492) | 1 | `Kp_H = 0.02` | 高度误差比例增益 |
| Gain3 (blk_4496) | 1 | `K_nz = 0.5` | 法向过载阻尼 |
| Gain4 (blk_4498) | 1 | `K_q = 0.3` | 俯仰角速率阻尼 |
| Gain5 (blk_4500) | 1 | `K_total = 2.0` | 总前向增益 |
| Gain6 (blk_4502) | 1 | `Ki_total = 0.1` | 积分增益 |

> **调谐说明**：
> - Kp_H 控制高度跟踪的响应速度，取值 0.02 对应 100m 误差产生 2° 升降舵偏转（经总增益后）
> - Kp_Vz 提供下滑道垂直速度前馈误差的阻尼
> - K_nz 和 K_q 提供内环阻尼，防止俯仰振荡
> - 积分增益 Ki 消除稳态高度误差

#### 2.2 动态 gamma_ref

将固定 Constant (blk_4479) 替换为根据 `landing_phase` 动态计算的 gamma_ref。

**方法**：添加 MATLAB Function 块 `gamma_ref_calc`:

```matlab
function gamma_ref = calc_gamma(phase, X, Vg)
%#codegen
% 根据轨迹段计算当前下滑角参考
persistent gamma1 gamma2 R X_R X_aim H_exp sigma

if isempty(gamma1)
    gamma1 = -5 * pi/180;
    gamma2 = -2 * pi/180;
    R = 8000;
    X_aim = -500;
    H_exp = 20;
    sigma = 6;
    X_R = X_R_calc;  % 需要传入或重新计算
end

switch phase
    case {1, 0}
        gamma_ref = 0;          % 平飞或接地
    case 2
        gamma_ref = gamma1;     % -5° 陡下滑
    case 3
        % 圆弧上的切线角 = atan2(dH/dX)
        % dH/dX = -(X-X_R)/√(R²-(X-X_R)²)
        dx = X - X_R;
        gamma_ref = atan2(-dx, sqrt(max(R^2 - dx^2, 1e-6)));
    case 4
        gamma_ref = gamma2;     % -2° 浅下滑
    case 5
        gamma_ref = 0;          % 拉飘用高度误差驱动
    otherwise
        gamma_ref = 0;
end
end
```

> **简化替代方案**：直接在 blk_4460 内添加 `From` 块读取 `landing_phase`，然后用 Switch 块选择不同 Constant 对应的 gamma 值（Phase 2→-5°, Phase 4→-2°, 其他→0）。

#### 2.3 积分器复位

在积分器 (blk_4501) 上设置：
- External reset: `rising`
- 添加复位信号：`NOT(landing_enable)` 或 `landing_phase == 0`

也可用 MATLAB Function 块实现带复位的积分器。

#### 2.4 输出饱和

在 Outport 前添加 Saturation 块：
- Upper limit: `de_max` (25°)
- Lower limit: `de_min` (-25°)

#### 2.5 Hcmd 信号连接

将 blk_4460 的 Hcmd 输入口连接到步骤1生成的 `H_cmd_landing` 信号（From 块读取 GotoTag `"H_cmd_landing"`）。

---

### 步骤 3：着陆模式检测逻辑

**目标**：创建 `landing_enable` 信号，供各子系统使用。

#### 3.1 手动触发（初期调试用）

在 `planmodel/control` 内：
- Constant 块，Value = 1（手动设为1触发着陆）
- Goto 块，Tag = `"landing_enable"`, Visibility = `global`

> 调试时可改 Constant 为 0 或 1 来控制是否进入着陆模式。

#### 3.2 自动触发（后续）

条件：飞机到达进场点 X_entry 附近 AND 已达到巡航高度
- CompareToConstant: `X_positon >= X_entry` (X_entry ≈ -3714)
- CompareToConstant: `height >= 340` (确保在巡航高度附近)
- AND 逻辑 → Goto("landing_enable")

---

### 步骤 4：俯仰通道模式切换

**目标**：修改 `planmodel/control/俯仰通道` (blk_3645)，增加巡航/着陆模式切换。

#### 4.1 现有架构回顾

```
巡航流程:
  H_cmd=350 → [高度控制 blk_3512] → nzc → [三回路自动驾驶仪 blk_1865] → de_3loop
  
起飞流程:
  theta_cmd=12° → [抬轮控制 blk_1948] → de_rotate
  
过渡:
  de_out = (1-λ)*de_rotate + λ*de_3loop  (H=10~30m区间)
  λ = smoothstep((H-10)/20)
```

#### 4.2 添加着陆高度指令切换

在 blk_3512 (高度控制) 的 h_cmd 输入前加 Switch：
```
u1: Constant 350          (巡航高度指令)
u3: From("H_cmd_landing") (着陆轨迹高度指令)
u2: From("landing_enable")
输出 → blk_3512.h_cmd
```

#### 4.3 添加着陆 nzc 源

`着陆俯仰控制` (blk_4460) 当前输出 de，不是 nzc。有两个方案：

**方案 A（推荐）**：修改 blk_4460 输出 nzc 而非 de

将 blk_4460 的输出端口从 de 改为 nzc_landing，然后在三回路前加 Switch：
```
u1: 巡航 nzc (来自 blk_3512)
u3: 着陆 nzc (来自 blk_4460)
u2: landing_enable
输出 → blk_1865.nzc
```

这样着陆也使用相同的三回路自动驾驶仪结构（伪攻角+俯仰阻尼），只需提供不同的 nzc 指令。

**方案 B**：blk_4460 直接输出 de

在三回路后、de_transition 前加 Switch 选择 de 来源：
```
u1: de_3loop (三回路输出)
u3: de_landing (blk_4460 输出)
u2: landing_enable
```

> 建议方案 A，更符合论文的"下滑道跟踪 → nzc → PID"思路。

#### 4.4 平滑过渡

在 nzc 切换后加 Transfer Fcn 块：
```
分子: [1]
分母: [0.5 1]   (一阶滤波 τ=0.5s)
```
避免 landing_enable 从 0→1 时的瞬态冲击。

---

### 步骤 5：推力控制修改

**目标**：进场时从 200m/s 减速到 V_app，接地后推力归零。

**位置**：`planmodel/control/推力控制` (blk_4251)

#### 5.1 速度指令切换

将 Constant block (blk_4253, Value=200) 替换为：
```
Switch:
  u1: Constant 200           (巡航速度)
  u3: Constant V_app         (进场速度, 1.4*Vs)
  u2: From("landing_enable")
```

V_app 约为 40-50 m/s（取决于 CLmax）。

#### 5.2 接地后推力归零

在推力输出 (blk_4260 Product 之后) 加 Switch：
```
u1: 推力指令值
u3: Constant 0
u2: From("touchdown")    (来自步骤6)
```

---

### 步骤 6：接地检测与地面滑跑控制

**目标**：检测接地，启用刹车，推力归零。

**位置**：在 `planmodel/control` 内新建逻辑。

#### 6.1 接地检测 (SR 锁存器)

用 Simulink 基础模块实现：
```
NOT(no_contact) ──→ AND ← landing_enable
                         │
                         ▼
                    SR Flip-Flop (可用 MATLAB Function 实现)
                         │
                         ▼
                    Goto("touchdown")
```

MATLAB Function 实现 SR 锁存器：

```matlab
function touchdown = sr_latch(set_signal, reset_signal)
%#codegen
% SR 锁存器: set=1→touchdown=1 且保持; reset=1→touchdown=0
persistent state
if isempty(state)
    state = false;
end
if reset_signal
    state = false;
elseif set_signal
    state = true;
end
touchdown = state;
end
```

- set_signal: NOT(no_contact) AND landing_enable
- reset_signal: Constant 0（仿真中不复位，调试时可手动改）

#### 6.2 刹车控制

在 control 内添加刹车逻辑。

**方案1（简单）**：使用 Switch 和 Constant

```
Switch:
  u1: Constant 0              (空中不刹车)
  u3: Constant 0.7            (刹车力度 70%)
  u2: touchdown AND Vg < 30   (接地且低速时刹车)
输出 → Goto("u_brake")
```

**方案2（渐进）**：使用 MATLAB Function

```matlab
function u_brake = brake_logic(touchdown, contact_nose, Vg)
%#codegen
if ~touchdown
    u_brake = 0;              % 空中不刹车
elseif contact_nose && Vg < 30
    u_brake = 0.7;            % 前轮接地且低速，全刹车
elseif Vg < 60
    u_brake = 0.3;            % 主轮接地，轻刹车
else
    u_brake = 0.0;
end
end
```

#### 6.3 连接 u_brake 到 6DOF 模型

在 `planmodel/6DOF-MODEL/动力学/起落架模型` 内：
- 检查 u_brake 输入口连接状态
- 如果未连接，添加 From 块读取 `"u_brake"` 并连接到刹车输入

#### 6.4 地面纠偏（无需修改）

已有的 `地面转向控制` (blk_2059) 使用 Y_cmd=0 保持跑道中心线，着陆滑跑时自动生效。

---

### 步骤 7：停止条件修改

**目标**：仿真在着陆滑跑完成（速度接近0）后才停止，而非一接地就停止。

**位置**：`planmodel/control` 内的 Stop Simulation 逻辑

#### 修改方法

当前逻辑链：
```
From2(height) → CompareToConstant(<=0) → Stop Simulation
```

修改为三条件 AND：
```
From("height")      → CompareToConstant(<=0)     ─┐
From("Vg")          → CompareToConstant(<1.0)     ─┼→ AND → Stop Simulation
From("touchdown")   → CompareToConstant(==1)      ─┘
```

用 Logical Operator 块（3输入 AND）组合三个条件。

---

### 步骤 8：plannew.m 初始化补充

在 `plannew.m` 末尾添加：

```matlab
%% 着陆阶段参数
Vs = sqrt(2 * m * g / (rho0 * S_ref * CLmax));   % 失速速度 (m/s)
V_app = 1.4 * Vs;                                  % 安全进场速度 (m/s)

fprintf('着陆参数: Vs=%.1f m/s, V_app=%.1f m/s\n', Vs, V_app);
```

---

## 四、完整模式切换状态机

```
                    ┌──────────────────────────────────────────┐
                    │                                          │
  GROUND ROLL ──────┤  TAKEOFF → ROTATION → CLIMB → CRUISE   │ (已有)
  (起飞)            │  (H<10m)   (Vg>VR)   (H<350m) (H=350m) │
                    │                                          │
                    │              ┌───────────────────────────┘
                    │              │ landing_enable = 1
                    │              ▼
                    │       APPROACH CAPTURE
                    │       (轨迹段 ① 定高350m)
                    │              │ X > X_entry
                    │              ▼
                    │       STEEP GLIDE (-5°)
                    │       (轨迹段 ②)
                    │              │ X > X_B
                    │              ▼
                    │       CIRCULAR ARC FLARE
                    │       (轨迹段 ③ R=8000m)
                    │              │ X > X_C
                    │              ▼
                    │       SHALLOW GLIDE (-2°)
                    │       (轨迹段 ④)
                    │              │ H < H_exp(20m)
                    │              ▼
                    │       EXPONENTIAL FLARE
                    │       (轨迹段 ⑤ σ=6s)
                    │              │ contact_main = 1
                    │              ▼
                    └────── GROUND ROLL (LANDING)
                            (刹车+纠偏→Vg≈0)
```

### 关键切换表

| 切换 | 触发条件 | 过渡方式 |
|------|----------|----------|
| 巡航→进近 | landing_enable=1 | 一阶滤波 τ=1.0s |
| 进近→陡下滑 | X > X_entry | 轨迹自动衔接 |
| 各轨迹段之间 | X 位置 | 几���连续过渡 |
| 空中→接地 | NOT(no_contact) | 硬切换 + 升降舵率限幅 |
| 滑跑→停止 | Vg<1.0 且高度≤0 且 touchdown | 停止仿真 |

---

## 五、实现顺序建议

```
步骤1 (轨迹生成器) ──┐
                      ├──→ 步骤4 (俯仰通道切换)
步骤2 (着陆控制器) ──┘        │
                               ├──→ 步骤7 (停止条件)
步骤3 (模式检测) ──────────────┤
                               │
步骤5 (推力控制) ──────────────┤
                               │
                      ┌────────┘
                      ▼
               步骤6 (接地+刹车+滑跑)
                      │
                      ▼
               步骤8 (初始化脚本)
```

**推荐执行顺序**：1→2→3→4→5→6→7→8

每步完成后保存模型并运行仿真验证。

---

## 六、验证与调参指南

### 6.1 轨迹生成器验证

```matlab
% 在 MATLAB 中运行
run('plannew');
run('landing_trajectory');  % 生成参考 H(X) 曲线
% 对比 Simulink 中 Landing_Trajectory_Generator 的输出
```

### 6.2 开环验证

设置初始条件使飞机在进场点附近：
```matlab
% 在 plannew.m 中临时设置
Xg0 = -7000;        % 进场点之前
Zg0 = -350;         % 高度350m (NED: Z向下)
u0_b = V_app;       % 进场速度
```

### 6.3 闭环跟踪验证

1. 检查高度曲线是否平滑跟踪 H_cmd
2. 检查各轨迹段过渡处无高度突变
3. 检查圆弧段的法向加速度 V²/R ≈ 3-4 m/s²（可接受）
4. 检查拉飘段接地时下沉率 < 0.5 m/s
5. 检查接地俯仰角 5-10° 抬头

### 6.4 增益调参

如果出现以下问题：

| 现象 | 调整 |
|------|------|
| 高度跟踪振荡 | 减小 Kp_H，增大 K_q |
| 跟踪滞后大 | 增大 Kp_H |
| 接地下沉率过大 | 增大 Kp_Vz |
| 俯仰振荡 | 增大 K_nz, K_q |
| 稳态高度误差 | 增大 Ki_total |

### 6.5 地面滑跑验证

1. 检查 touchdown 信号正确置位
2. 检查刹车力使飞机减速
3. 检查侧向偏差在一定范围内
4. 检查仿真在 Vg≈0 时正常停止

---

## 七、参考资料

1. 胡浩. 无人机进场着陆/地面滑跑控制与仿真[D]. 电子科技大学, 2011.
2. 论文核心方案：两次下滑两次拉平轨迹 + PID控制器 + 起落架建模
3. 当前着陆轨迹：`D:\轮式起降\planmodel\landing_trajectory.m`
4. 飞参初始化：`D:\轮式起降\planmodel\plannew.m`
5. 控制器设计：`D:\轮式起降\planmodel\control_design.m`
