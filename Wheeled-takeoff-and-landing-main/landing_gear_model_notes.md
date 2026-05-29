# 起落架模型公式化说明

本文档说明当前六自由度模型中新增的前三点式起落架模型。模型遵循
`research_conventions.md` 中的前、上、右坐标系约定：

- 机体系：\(OXYZ_1\)，\(X_1\) 向前，\(Y_1\) 向上，\(Z_1\) 向右。
- 地面系：\(OXYZ_g\)，\(X_g\) 向前，\(Y_g\) 向上，\(Z_g\) 向右。
- 地面平面取为 \(Y_g=Y_{\mathrm{ground}}\)。

当前起落架模型的目标是模拟静态支反力、滑跑起飞、着陆接地以及前轮转弯纠偏等低速地面运动工况。模型为工程简化模型，暂不包含轮胎转速、滑移率、复杂轮胎侧偏刚度曲线和起落架结构非线性。

## 1. 起落架布局

前三点式起落架包括一个前轮和两个主轮：

| 名称         | 含义   |
| ------------ | ------ |
| `nose`       | 前轮   |
| `left_main`  | 左主轮 |
| `right_main` | 右主轮 |

第 \(i\) 个轮点在机体系下相对质心的位置定义为

$$
\boldsymbol r_i^b =
\begin{bmatrix}
x_i \\
y_i \\
z_i
\end{bmatrix},
$$

其中 \(x_i\) 向前为正，\(y_i\) 向上为正，\(z_i\) 向右为正。由于轮子位于质心下方，因此一般有

$$
y_i < 0.
$$

当前参数采用

$$
\boldsymbol r_{\mathrm{nose}}^b =
\begin{bmatrix}
2.00 \\
-0.65 \\
0
\end{bmatrix}\ \mathrm{m},
$$

$$
\boldsymbol r_{\mathrm{left}}^b =
\begin{bmatrix}
-1/3 \\
-0.65 \\
-0.80
\end{bmatrix}\ \mathrm{m},
\qquad
\boldsymbol r_{\mathrm{right}}^b =
\begin{bmatrix}
-1/3 \\
-0.65 \\
0.80
\end{bmatrix}\ \mathrm{m}.
$$

主轮纵向位置 \(x_m=-1/3\ \mathrm{m}\) 是按照静态俯仰力矩平衡初步选取的。若前轮法向力为 \(N_n\)，左右主轮合力为 \(N_l+N_r\)，静态俯仰力矩平衡近似满足

$$
x_n N_n + x_m(N_l+N_r)=0,
$$

因此

$$
x_m=-\frac{x_n N_n}{N_l+N_r}.
$$

## 2. 轮点运动学

设飞行器质心在地面系下的位置为

$$
\boldsymbol r_{\mathrm{cg}}^g =
\begin{bmatrix}
X_g \\
Y_g \\
Z_g
\end{bmatrix},
$$

机体系到地面系的方向余弦矩阵为

$$
C_{bg}=C_{bg}(k_\theta,\psi,\gamma).
$$

则第 \(i\) 个轮点在地面系下的位置为

$$
\boldsymbol r_i^g =
\boldsymbol r_{\mathrm{cg}}^g + C_{bg}\boldsymbol r_i^b.
$$

设质心速度在机体系下为

$$
\boldsymbol V^b =
\begin{bmatrix}
V_x \\
V_y \\
V_z
\end{bmatrix},
$$

角速度在机体系下为

$$
\boldsymbol \omega^b =
\begin{bmatrix}
\omega_x \\
\omega_y \\
\omega_z
\end{bmatrix}.
$$

轮点相对地面的速度先在机体系中写为

$$
\boldsymbol V_i^b =
\boldsymbol V^b + \boldsymbol\omega^b \times \boldsymbol r_i^b,
$$

再转换到地面系：

$$
\boldsymbol V_i^g = C_{bg}\boldsymbol V_i^b.
$$

## 3. 接地判定与压缩量

设轮胎半径为 \(R_w\)。第 \(i\) 个轮点的地面系高度为

$$
Y_i^g = \boldsymbol e_Y^T\boldsymbol r_i^g,
\qquad
\boldsymbol e_Y =
\begin{bmatrix}
0 \\
1 \\
0
\end{bmatrix}.
$$

轮胎压缩量定义为

$$
\delta_i = R_w - \left(Y_i^g - Y_{\mathrm{ground}}\right).
$$

当

$$
\delta_i > 0
$$

时，认为该轮接地；当

$$
\delta_i \le 0
$$

时，认为该轮离地，法向力和摩擦力均取零。

压缩速度由轮点竖直速度给出：

$$
\dot\delta_i = -V_{i,Y}^g
=-\boldsymbol e_Y^T\boldsymbol V_i^g.
$$

这里取负号的原因是：轮点向下运动时 \(V_{i,Y}^g<0\)，压缩量应增加，因此 \(\dot\delta_i>0\)。

## 4. 法向支反力模型

每个轮子的法向支反力采用线性弹簧阻尼模型：

$$
N_i^\ast = k_i\delta_i + c_i\dot\delta_i,
$$

其中 \(k_i\) 为等效垂向刚度，\(c_i\) 为等效垂向阻尼。

由于地面只能提供推力形式的支反力，不能提供拉力，因此最终法向力为

$$
N_i =
\begin{cases}
\max\left(0,N_i^\ast\right), & \delta_i>0, \\
0, & \delta_i\le 0.
\end{cases}
$$

当前参数为

$$
k_{\mathrm{nose}}=1.0\times 10^5\ \mathrm{N/m},
$$

$$
k_{\mathrm{main}}=3.0\times 10^5\ \mathrm{N/m},
$$

$$
c_{\mathrm{nose}}=0.8\times 10^4\ \mathrm{N/(m/s)},
$$

$$
c_{\mathrm{main}}=1.4\times 10^4\ \mathrm{N/(m/s)}.
$$

对于 \(m=600\ \mathrm{kg}\) 的飞行器，重量为

$$
W=mg.
$$

三轮总垂向刚度为

$$
k_{\Sigma}=k_{\mathrm{nose}}+k_{\mathrm{left}}+k_{\mathrm{right}},
$$

静态平均压缩量估算为

$$
\delta_{\mathrm{static}}=\frac{W}{k_{\Sigma}}.
$$

代入当前参数，可得

$$
\delta_{\mathrm{static}}\approx 0.00841\ \mathrm{m}.
$$

静态配平检查中，法向力分配约为

$$
N_{\mathrm{nose}}\approx 840.57\ \mathrm{N},
$$

$$
N_{\mathrm{left}}\approx 2521.71\ \mathrm{N},
$$

$$
N_{\mathrm{right}}\approx 2521.71\ \mathrm{N}.
$$

因此前轮约承担

$$
\frac{N_{\mathrm{nose}}}{W}\approx 14.3\%,
$$

左右主轮合计约承担

$$
\frac{N_{\mathrm{left}}+N_{\mathrm{right}}}{W}\approx 85.7\%.
$$

## 5. 轮胎方向定义

对于主轮，滚动方向近似与机体系 \(X_1\) 轴一致，侧向方向近似与机体系 \(Z_1\) 轴一致。

对于前轮，考虑转向角 \(\delta_s\)。转向角限幅为

$$
\delta_s\in[-\delta_{s,\max},\delta_{s,\max}],
$$

当前取

$$
\delta_{s,\max}=25^\circ.
$$

前轮在机体系下的滚动方向定义为

$$
\boldsymbol e_{\mathrm{long}}^b =
\begin{bmatrix}
\cos\delta_s \\
0 \\
\sin\delta_s
\end{bmatrix},
$$

侧向方向定义为

$$
\boldsymbol e_{\mathrm{lat}}^b =
\begin{bmatrix}
-\sin\delta_s \\
0 \\
\cos\delta_s
\end{bmatrix}.
$$

对于主轮，有

$$
\delta_s=0,
$$

因此

$$
\boldsymbol e_{\mathrm{long}}^b =
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix},
\qquad
\boldsymbol e_{\mathrm{lat}}^b =
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}.
$$

将方向向量变换到地面系后，为避免俯仰和滚转姿态导致滚动方向带有竖直分量，模型会取其水平投影并单位化：

$$
\tilde{\boldsymbol e}_{\mathrm{long}}^g
=
\left(I-\boldsymbol e_Y\boldsymbol e_Y^T\right)
C_{bg}\boldsymbol e_{\mathrm{long}}^b,
$$

$$
\boldsymbol e_{\mathrm{long}}^g
=
\frac{\tilde{\boldsymbol e}_{\mathrm{long}}^g}
{\left\|\tilde{\boldsymbol e}_{\mathrm{long}}^g\right\|}.
$$

侧向方向同理：

$$
\tilde{\boldsymbol e}_{\mathrm{lat}}^g
=
\left(I-\boldsymbol e_Y\boldsymbol e_Y^T\right)
C_{bg}\boldsymbol e_{\mathrm{lat}}^b,
$$

$$
\boldsymbol e_{\mathrm{lat}}^g
=
\frac{\tilde{\boldsymbol e}_{\mathrm{lat}}^g}
{\left\|\tilde{\boldsymbol e}_{\mathrm{lat}}^g\right\|}.
$$

## 6. 轮胎纵向力与侧向力

轮点速度沿滚动方向和侧向方向的分量分别为

$$
V_{\mathrm{long},i}
=
(\boldsymbol V_i^g)^T
\boldsymbol e_{\mathrm{long},i}^g,
$$

$$
V_{\mathrm{lat},i}
=
(\boldsymbol V_i^g)^T
\boldsymbol e_{\mathrm{lat},i}^g.
$$

为避免速度过零时符号函数不连续，采用平滑符号函数

$$
\sigma(V)=\tanh\left(\frac{V}{V_\epsilon}\right),
$$

其中当前取

$$
V_\epsilon=0.20\ \mathrm{m/s}.
$$

滚动阻力按法向力比例给出：

$$
F_{\mathrm{roll},i}
=
-c_{\mathrm{roll},i}N_i\sigma(V_{\mathrm{long},i}).
$$

其中当前参数为

$$
c_{\mathrm{roll},\mathrm{nose}}=0.020,
$$

$$
c_{\mathrm{roll},\mathrm{main}}=0.018.
$$

制动力作用在主轮上，写为

$$
F_{\mathrm{brake},i}
=
-u_{\mathrm{brake}}\mu_{\mathrm{brake},i}N_i
\sigma(V_{\mathrm{long},i}),
$$

其中 \(u_{\mathrm{brake}}\in[0,1]\) 为刹车指令。当前设置为

$$
\mu_{\mathrm{brake},\mathrm{nose}}=0,
$$

$$
\mu_{\mathrm{brake},\mathrm{main}}=0.55.
$$

纵向合力为

$$
F_{\mathrm{long},i}
=
F_{\mathrm{roll},i}+F_{\mathrm{brake},i}.
$$

侧向力采用线性速度阻尼：

$$
F_{\mathrm{lat},i}^\ast
=
-c_{\mathrm{lat},i}V_{\mathrm{lat},i}.
$$

当前参数为

$$
c_{\mathrm{lat},\mathrm{nose}}
=1.4\times 10^4\ \mathrm{N/(m/s)},
$$

$$
c_{\mathrm{lat},\mathrm{main}}
=2.2\times 10^4\ \mathrm{N/(m/s)}.
$$

## 7. 摩擦限幅

轮胎地面摩擦合力不能超过库仑摩擦上限：

$$
\sqrt{
F_{\mathrm{long},i}^2+
\left(F_{\mathrm{lat},i}^\ast\right)^2
}
\le
\mu_i N_i.
$$

当前设置为

$$
\mu_{\mathrm{nose}}=0.65,
$$

$$
\mu_{\mathrm{main}}=0.70.
$$

若未超过摩擦上限，则

$$
F_{\mathrm{lat},i}=F_{\mathrm{lat},i}^\ast.
$$

若超过摩擦上限，则按比例缩放纵向力和侧向力：

$$
s_i=
\frac{\mu_iN_i}
{\sqrt{
F_{\mathrm{long},i}^2+
\left(F_{\mathrm{lat},i}^\ast\right)^2
}},
$$

$$
F_{\mathrm{long},i}\leftarrow s_iF_{\mathrm{long},i},
\qquad
F_{\mathrm{lat},i}\leftarrow s_iF_{\mathrm{lat},i}^\ast.
$$

这样可以保证

$$
\sqrt{
F_{\mathrm{long},i}^2+
F_{\mathrm{lat},i}^2
}
=
\mu_iN_i.
$$

## 8. 单轮力与总力矩

第 \(i\) 个轮子在地面系下的总作用力为

$$
\boldsymbol F_i^g
=
\begin{bmatrix}
0 \\
N_i \\
0
\end{bmatrix}
+
F_{\mathrm{long},i}\boldsymbol e_{\mathrm{long},i}^g
+
F_{\mathrm{lat},i}\boldsymbol e_{\mathrm{lat},i}^g.
$$

将其转换到机体系：

$$
\boldsymbol F_i^b=C_{gb}\boldsymbol F_i^g,
$$

其中

$$
C_{gb}=C_{bg}^T.
$$

单轮对质心产生的力矩为

$$
\boldsymbol M_i^b
=
\boldsymbol r_i^b\times \boldsymbol F_i^b.
$$

起落架总力和总力矩为

$$
\boldsymbol F_{\mathrm{LG}}^b
=
\sum_i \boldsymbol F_i^b,
$$

$$
\boldsymbol M_{\mathrm{LG}}^b
=
\sum_i \boldsymbol M_i^b.
$$

## 9. 与六自由度方程耦合

气动、推力部分给出的机体系力和力矩记为

$$
\boldsymbol F_{\mathrm{AP}}^b,
\qquad
\boldsymbol M_{\mathrm{AP}}^b.
$$

叠加起落架后，总力和总力矩为

$$
\boldsymbol F^b
=
\boldsymbol F_{\mathrm{AP}}^b
+
\boldsymbol F_{\mathrm{LG}}^b,
$$

$$
\boldsymbol M^b
=
\boldsymbol M_{\mathrm{AP}}^b
+
\boldsymbol M_{\mathrm{LG}}^b.
$$

质心平动动力学方程写为

$$
\dot{\boldsymbol V}^b
=
\frac{\boldsymbol F^b}{m}
+
C_{gb}
\begin{bmatrix}
0 \\
-g \\
0
\end{bmatrix}
-
\boldsymbol\omega^b\times\boldsymbol V^b.
$$

转动动力学方程写为

$$
\dot{\boldsymbol\omega}^b
=
I^{-1}
\left[
\boldsymbol M^b
-
\boldsymbol\omega^b
\times
\left(I\boldsymbol\omega^b\right)
\right].
$$

位置运动学方程为

$$
\dot{\boldsymbol r}_{\mathrm{cg}}^g
=
C_{bg}\boldsymbol V^b.
$$

欧拉角运动学仍采用项目约定的 \(YZX\) 顺序，即

$$
\boldsymbol \eta
=
\begin{bmatrix}
k_\theta \\
\psi \\
\gamma
\end{bmatrix}.
$$

## 10. 当前参数选取思路

轮距 \(2z_m=1.60\ \mathrm{m}\) 用于提供滚转支撑能力，使左右主轮在滚转姿态或侧向速度下能够产生合理的差动法向力和侧向纠偏力矩。

前轮纵向位置 \(x_n=2.00\ \mathrm{m}\) 用于提供足够的前轮转弯力矩臂。主轮纵向位置 \(x_m=-1/3\ \mathrm{m}\) 用于使静态三轮支反力满足近似俯仰力矩平衡。

前轮刚度小于主轮刚度，是为了让主轮承担主要重量，符合前三点式布局的基本受力特征。左右主轮刚度相同，用于保持无滚转姿态下的左右对称。

侧向阻尼 \(c_{\mathrm{lat}}\) 用于模拟轮胎侧偏纠偏能力。主轮侧向阻尼大于前轮侧向阻尼，用于提供较强的滑跑方向稳定性；前轮通过转向角 \(\delta_s\) 额外提供可控纠偏能力。

摩擦系数 \(\mu\) 和刹车摩擦系数 \(\mu_{\mathrm{brake}}\) 目前取为工程估计值，适合开展闭环逻辑验证和起降地面段仿真。若后续需要更真实的滑跑距离、刹车距离或侧风着陆响应，应结合轮胎、跑道和试验数据重新标定。

## 11. 静态与动态验证

当前静态检查满足

$$
\sum_i N_i = mg,
$$

且起落架对质心的静态合力矩近似为零：

$$
\sum_i \boldsymbol r_i^b\times\boldsymbol F_i^b
\approx
\boldsymbol 0.
$$

已建立的验证脚本包括：

- `matlab/check_landing_gear_static.m`：静态支反力和静态力矩检查。
- `matlab/run_takeoff_roll_demo.m`：滑跑起飞和前轮转向纠偏演示。
- `matlab/run_landing_touchdown_demo.m`：着陆接地和起落架缓冲演示。
- `matlab/plot_landing_gear_demo.m`：演示结果绘图。

建议后续验证顺序为：先检查静态支反力，再检查滑跑起飞，最后检查着陆接地瞬态响应。

## 12. 模型能力

当前起落架模型已经具备以下能力。

首先，模型可以给出静态三点支反力分配。在飞行器静止于水平地面时，三个轮子的法向力满足

$$
N_{\mathrm{nose}}+N_{\mathrm{left}}+N_{\mathrm{right}}
\approx
mg,
$$

并且通过前轮和主轮纵向位置的选取，可以使静态俯仰力矩近似平衡：

$$
x_{\mathrm{nose}}N_{\mathrm{nose}}
+
x_{\mathrm{left}}N_{\mathrm{left}}
+
x_{\mathrm{right}}N_{\mathrm{right}}
\approx
0.
$$

因此该模型可以用于检查起落架布置是否会导致明显的静态低头或抬头趋势。

其次，模型可以模拟滑跑过程中的地面支撑、滚动阻力、刹车阻力和侧向纠偏力。滑跑中每个轮子的地面力均由法向支反力、纵向摩擦力和侧向摩擦力组成：

$$
\boldsymbol F_i^g
=
\begin{bmatrix}
0 \\
N_i \\
0
\end{bmatrix}
+
F_{\mathrm{long},i}\boldsymbol e_{\mathrm{long},i}^g
+
F_{\mathrm{lat},i}\boldsymbol e_{\mathrm{lat},i}^g.
$$

其中前轮转向角 \(\delta_s\) 会改变前轮滚动方向和侧向方向，因此可以形成绕质心的偏航纠偏力矩：

$$
M_{Y,\mathrm{steer}}^b
\approx
\left(
\boldsymbol r_{\mathrm{nose}}^b
\times
\boldsymbol F_{\mathrm{nose}}^b
\right)_Y.
$$

这使模型可以用于验证低速滑跑阶段的前轮转弯控制逻辑，例如跑道中心线保持、初始偏航角纠偏和地面滑跑方向稳定性。

第三，模型可以表达两轮接地、三轮接地和离地切换。当某个轮子的压缩量满足

$$
\delta_i \le 0
$$

时，该轮的法向力和摩擦力自动变为零：

$$
N_i=0,
\qquad
F_{\mathrm{long},i}=0,
\qquad
F_{\mathrm{lat},i}=0.
$$

因此模型可以用于描述着陆接地瞬间的单轮、两轮或三轮接地状态，也可以描述起飞抬前轮后由三轮接地逐步过渡到主轮接地，再到完全离地的过程。

第四，模型可以与当前六自由度方程直接耦合。起落架力和力矩叠加到气动、推力模型之后，会同时影响质心加速度和角加速度：

$$
\dot{\boldsymbol V}^b
=
\frac{
\boldsymbol F_{\mathrm{AP}}^b
+
\boldsymbol F_{\mathrm{LG}}^b
}{m}
+
C_{gb}
\begin{bmatrix}
0 \\
-g \\
0
\end{bmatrix}
-
\boldsymbol\omega^b\times\boldsymbol V^b,
$$

$$
\dot{\boldsymbol\omega}^b
=
I^{-1}
\left[
\boldsymbol M_{\mathrm{AP}}^b
+
\boldsymbol M_{\mathrm{LG}}^b
-
\boldsymbol\omega^b
\times
\left(I\boldsymbol\omega^b\right)
\right].
$$

因此模型不仅能反映垂向支撑，还能反映地面力对俯仰、滚转和偏航运动的耦合影响。

## 13. 模型局限

当前模型仍然是一个用于控制律和总体仿真的简化工程模型，主要局限如下。

第一，轮胎模型没有显式描述轮胎转速和滑移率。实际轮胎纵向力通常与滑移率 \(\kappa\) 有关：

$$
F_x = f(\kappa,N),
$$

其中滑移率可近似写为

$$
\kappa =
\frac{R_w\Omega_w - V_{\mathrm{long}}}
{\max\left(\left|V_{\mathrm{long}}\right|,\epsilon\right)}.
$$

当前模型没有引入轮胎角速度 \(\Omega_w\)，因此不能准确描述抱死、空转、防滑刹车和驱动轮牵引等问题。

第二，侧向力采用线性速度阻尼，而不是完整的轮胎侧偏角模型。真实轮胎侧向力更常写为

$$
F_y=f(\alpha_t,N,\kappa),
$$

其中 \(\alpha_t\) 为轮胎侧偏角。小侧偏角时可以近似为

$$
F_y\approx -C_\alpha\alpha_t,
$$

但当前模型采用的是

$$
F_{\mathrm{lat}}=-c_{\mathrm{lat}}V_{\mathrm{lat}}.
$$

因此该模型适合模拟侧向纠偏趋势，但不适合精确评估轮胎侧偏角、侧偏刚度饱和和高速侧向稳定性。

第三，垂向缓冲支柱采用线性弹簧阻尼模型：

$$
N=k\delta+c\dot{\delta}.
$$

真实起落架缓冲器通常具有明显非线性，例如油气式缓冲支柱可能更接近

$$
N=f(\delta,\dot{\delta}),
$$

且压缩行程、回弹阻尼和结构限位均会影响接地峰值载荷。当前模型没有显式包含最大行程限位、非线性阻尼和结构硬碰撞，因此着陆冲击峰值只能作为定性参考。

第四，地面模型目前是刚性、水平、平整跑道：

$$
Y_g=Y_{\mathrm{ground}}.
$$

模型没有考虑跑道坡度、地面起伏、地面沉陷、湿滑跑道、结冰跑道和局部摩擦变化。因此它适合标准水平跑道工况，不适合直接用于复杂地面环境分析。

第五，当前模型没有考虑起落架收放机构、舱门、气动干扰和结构柔性。也就是说，起落架只作为地面接触力源进入六自由度方程，而没有改变气动模型：

$$
\boldsymbol F_{\mathrm{aero}}^b
\ne
\boldsymbol F_{\mathrm{aero}}^b(\mathrm{gear\ deployed})
$$

在当前实现中并未成立。若后续需要模拟起落架放下后的阻力增加，应在气动模型中额外加入起落架阻力项。

第六，当前参数主要用于验证模型结构和控制逻辑，还不是经过试验辨识的高可信参数。因此，滑跑距离、刹车距离、接地过载峰值、前轮纠偏响应速度等结果目前更适合作为趋势判断，而不应直接作为工程定型依据。

综上，当前起落架模型适合用于：

- 六自由度模型的起降地面段闭环仿真。
- 起飞滑跑过程中的前轮转弯纠偏控制验证。
- 着陆接地后的支反力、姿态响应和滑跑稳定性初步分析。
- 制导控制律在空地转换阶段的逻辑连贯性检查。

当前起落架模型暂不适合用于：

- 精确轮胎力学分析。
- 防滑刹车系统设计。
- 高保真着陆冲击载荷评估。
- 复杂跑道环境或强侧风着陆认证级仿真。
- 需要试验标定参数支撑的起落架结构强度设计。
