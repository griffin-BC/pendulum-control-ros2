<!-- ============================================================
     推之前必须做（做完删掉这段注释）：
     1. 录一段演示动图，存成 docs/demo.gif
     2. 把下面这行的行首「!- -」删掉，变成真正的图片引用
!- - ![演示](docs/demo.gif)
     ============================================================ -->

# Cart-Pole 倒立摆 · PD / LQR 最优控制（ROS 2）

用 ROS 2 从零实现的倒立摆（cart-pole）控制闭环：**非线性动力学建模 → 平衡点线性化 → LQR 求解 → 闭环验证**。同时实现 PD 控制器作为对照。

被控对象与控制器**完全解耦为独立的 ROS 2 节点**——对象不知道控制器是谁，控制器不知道对象如何积分。换控制器不需要动被控对象一行代码。

---

## 1. 系统结构

```
┌──────────────────────────────────────────────────┐
│                    plant 节点                     │
│    RK4 积分 cart-pole 非线性动力学 @ 100 Hz       │
└───────────────┬──────────────────────────────────┘
                │  /joint_states   (sensor_msgs/JointState)
                │  position = [x, θ]   velocity = [ẋ, θ̇]
                ▼
      ┌─────────┴─────────┐
      │                   │
┌─────▼──────┐    ┌───────▼────────┐
│  lqr 节点   │    │   viz 节点      │
│  u = −K·X  │    │ matplotlib 动画 │
└─────┬──────┘    └────────────────┘
      │  /pendulum/cmd_force   (std_msgs/Float64)
      └──────────► 回到 plant
```

`pd` 节点与 `lqr` 节点**接口完全相同**，可以互换——这正是 ROS 2 解耦的价值。

---

## 2. 动力学建模

采用**点质量在杆端**的简化模型（cart-pole）：小车质量 $M$，摆杆质量 $m$ 集中在距铰点 $l$ 处。

状态向量 $X = [x,\ \dot{x},\ \theta,\ \dot{\theta}]^\top$，控制输入为作用在小车上的水平力 $F$。

由拉格朗日方程可得：

```
(M + m)·ẍ + m·l·θ̈·cosθ − m·l·θ̇²·sinθ = F
        ẍ·cosθ + l·θ̈ − g·sinθ = 0
```

解出角加速度与线加速度（`plant_node.py` 中的 `dynamics()`）：

```
θ̈ = [ (M+m)·g·sinθ − F·cosθ − m·l·θ̇²·sinθ·cosθ ] / [ l·(M + m·sin²θ) ]
ẍ = [ F − m·l·(θ̈·cosθ − θ̇²·sinθ) ] / (M + m)
```

**数值积分**：定步长四阶龙格-库塔（RK4），`dt = 0.01 s`（100 Hz），由 `create_timer(dt)` 驱动。

**物理参数**：$M = 1.0\ \mathrm{kg}$，$m = 0.1\ \mathrm{kg}$，$l = 0.5\ \mathrm{m}$，$g = 9.81\ \mathrm{m/s^2}$。

> 直立点（$\theta = 0$）是不稳定平衡点——不加控制时摆杆必然倒下。

---

## 3. 平衡点线性化

在直立点附近做泰勒展开，$X \approx 0$ 时 $\sin\theta \approx \theta,\ \cos\theta \approx 1,\ \dot\theta^2 \approx 0$，得到线性模型 $\dot{X} = AX + Bu$：

```
      ⎡ 0  1        0      0 ⎤          ⎡    0    ⎤
A =   ⎢ 0  0   −m·g/M      0 ⎥      B = ⎢  1/M    ⎥
      ⎢ 0  0        0      1 ⎥          ⎢    0    ⎥
      ⎣ 0  0  (M+m)·g/(l·M) 0 ⎦          ⎣ −1/(l·M)⎦
```

代入参数后 $B_\theta = -1/(lM) = -2 < 0$。

> ⚠️ **注意这个符号**。多数教材取 $B_\theta > 0$，本模型相反，因此最终增益的符号与常见形式相反。这是**推导出来的**结果，不是抄的。

**能控性**：$\mathrm{rank}[B\ AB\ A^2B\ A^3B] = 4$，系统完全能控。

---

## 4. 控制设计

### 4.1 PD 控制器（对照）

```
u = Kp·θ + Kd·θ̇
```

只反馈摆角，不含位置的反馈——所以摆杆能立住，但**小车会漂走**。这是理解 LQR 价值的好对照。

### 4.2 LQR 控制器

最小化二次型代价：

```
J = ∫₀^∞ (XᵀQX + uᵀRu) dt
```

求解代数黎卡提方程 $A^\top P + PA - PBR^{-1}B^\top P + Q = 0$，得最优增益：

```
K = R⁻¹BᵀP ,   u = −K·X
```

**权重选择**（`Q = diag([10, 1, 100, 1])`，对应 $[x, \dot{x}, \theta, \dot{\theta}]$）：

| 状态 | 权重 | 意图 |
|---|---|---|
| $x$ | 10 | 希望小车回到原点 |
| $\dot{x}$ | 1 | 轻微抑制小车速度 |
| $\theta$ | **100** | **首要目标：保持直立** |
| $\dot{\theta}$ | 1 | 轻微抑制摆动 |
| $R$ | 1 | 单输入，控制代价基准 |

> Q 和 R 不是"调参旋钮"，而是**把你想要什么写成数学**。θ 的权重给了 100，就是在说"直立比位置重要一万倍"。

求解用 `scipy.linalg.solve_continuous_are`。

---

## 5. 结果

**计算得到的 LQR 增益**（启动时打印）：

```
K = [ -3.162   -4.425   -39.608   -8.48 ]
```

**闭环极点**：$-5.17 \pm 1.72j$ 与 $-1.10 \pm 0.94j$ —— **全部位于左半平面**，闭环稳定。

**非线性仿真验证**（用上面这个线性设计出的 $K$，回代到**非线性**模型）：

| 初始条件 | 结果 |
|---|---|
| $\theta_0 = 0.05$ rad（约 2.9°） | $x \to 0$，$\theta \to 0$ |
| $\theta_0 = 1.603$ rad（约 92°，几乎水平倒下） | 仍能恢复直立 |

第二行说明线性化设计的控制器有**相当可观的吸引域**——在远离平衡点处依然有效。

---

## 6. 怎么跑

### 依赖

- ROS 2 **Humble**（Ubuntu 22.04）
- `numpy`、`scipy`、`python3-matplotlib`

```bash
sudo apt install ros-humble-desktop python3-numpy python3-scipy python3-matplotlib
```

### 编译运行

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/griffin-BC/pendulum-control-ros2.git
cp -r pendulum-control-ros2/src/* .

cd ~/ros2_ws
colcon build --symlink-install --packages-select pendulum_control
source install/setup.bash
ros2 launch pendulum_control pendulum.launch.py
```

**一个命令启动三个节点**（plant + lqr + viz），`Ctrl+C` 一次性全部回收。

### 单独跑某个节点

```bash
ros2 run pendulum_control plant    # 被控对象
ros2 run pendulum_control pd       # PD 控制器（与 lqr 二选一）
ros2 run pendulum_control lqr      # LQR 控制器
ros2 run pendulum_control viz      # 可视化
```

> ⚠️ **同一个话题上只能有一个控制器发布者。** 如果同时跑 `pd` 和 `lqr`，两者会发出互相矛盾的指令，闭环会立刻发散成 `nan`。跑之前用 `ros2 topic info /pendulum/cmd_force` 确认 `Publisher count: 1`。

---

## 7. 话题接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | plant → 控制器 | `position = [x, θ]`，`velocity = [ẋ, θ̇]`，`name = ['cart_joint', 'pole_joint']` |
| `/pendulum/cmd_force` | `std_msgs/Float64` | 控制器 → plant | 作用在小车上的水平力，单位 N |

---

## 8. 调参

改 `lqr_controller.py` 里的 `Q`：

| 改动 | 效果 |
|---|---|
| θ 权重 100 → 500 | 更拼命保持直立，但小车晃动幅度变大 |
| x 权重 10 → 100 | 更在意回到原点，但摆杆稳得更慢 |
| R 1 → 10 | 整体控制更"温柔"（省力），响应变慢 |

---

## 9. 已知限制与后续

**当前限制**

- 被控对象是**纯数值仿真**，未接入物理引擎（无摩擦、无延迟、无执行器饱和）
- 点质量在杆端的简化模型，未考虑摆杆转动惯量

**后续计划**

- [ ] 用 URDF 描述模型并接入 **Gazebo**，验证 LQR 在含摩擦与延迟的真实物理引擎中的鲁棒性
- [ ] 通过 `ros2_control` 集成控制器，替代当前的独立节点
- [ ] 加入噪声与延迟，测试鲁棒性边界

---

## 10. 运行环境

| 项 | 版本 |
|---|---|
| OS | Ubuntu 22.04 (jammy) |
| ROS 2 | Humble |
| Python | 3.10 |
| 依赖 | numpy / scipy / matplotlib |

---

## 许可证

MIT —— 见 [LICENSE](LICENSE)
