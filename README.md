# WBR MuJoCo 气弹簧模型

本项目提供基于 `wbr.urdf` 转换并修正后的 MuJoCo MJCF 模型，用于模拟 WBR 机构左右两侧的气弹簧和关节驱动。

## 文件

```text
.
├── wbr.urdf       # 原始机器人 URDF
├── mjmodel.xml    # MuJoCo 模型文件
├── meshes/        # STL 几何模型
└── README.md
```

## 加载模型

使用 MuJoCo Python 直接加载：

```python
import mujoco

model = mujoco.MjModel.from_xml_path("mjmodel.xml")
data = mujoco.MjData(model)

while True:
    mujoco.mj_step(model, data)
```

也可以使用 MuJoCo 自带的 simulate 程序打开：

```bash
simulate mjmodel.xml
```

模型使用 SI 单位：米、牛顿、秒、弧度。

## 气弹簧模型

模型中包含左右两根气弹簧：

| 气弹簧 | 上端 | 下端 | MuJoCo tendon |
|---|---|---|---|
| 左侧 | `lspringsite1` | `lspringsite2` | `left_gas_spring` |
| 右侧 | `rspringsite1` | `rspringsite2` | `right_gas_spring` |

气弹簧通过 spatial tendon 计算两个安装点之间的实时距离，并通过长度相关的 general actuator 施加力：

```text
F(L) = 250 + 1800 × (L - 0.120) N
```

参数为：

- 最小长度：`120 mm`
- 总长上限：`195 mm`
- 有效行程：`75 mm`
- 最小工作力：`250 N`
- 最大工作力：`385 N`
- 等效刚度斜率：`1800 N/m`
- 速度阻尼项：`-4 × Ldot`
- actuator 力限制：`250–385 N`

在 MJCF 中对应：

```xml
<general ... gaintype="fixed" gainprm="0"
  biastype="affine" biasprm="34 1800 -4"
  forcelimited="true" forcerange="250 385"/>
```

`biasprm` 的计算为：

```text
34 = 250 - 1800 × 0.120
```

因此 actuator 不依赖外部控制输入，气弹簧力会根据 tendon 当前长度自动计算。

## 安装点映射

`site1` 使用 URDF 中原有的 `lspringsite1/rspringsite1` 固定关节位置。

`site2` 的空间位置沿用 URDF 原始气弹簧安装点，但运动 parent 设置为 `llink1_child1/rlink1_child1`，以便安装点跟随对应的连杆运动。重新设置 parent 时，已将原始空间坐标转换为新的局部坐标：

```xml
lspringsite2 pos="0.04321618 0.0175 -0.02514666"
rspringsite2 pos="0.04321400 0.008 -0.02514710"
```

当前 home 姿态下，两侧气弹簧安装距离约为：

```text
159.9 mm
```

## 关节参数

四个主驱动关节保留粘性阻尼，但移除了低速库仑摩擦：

| 关节 | 粘性阻尼 | frictionloss |
|---|---:|---:|
| `ljoint1` | `0.2 N·m·s/rad` | `0` |
| `ljoint4` | `0.2 N·m·s/rad` | `0` |
| `rjoint1` | `0.2 N·m·s/rad` | `0` |
| `rjoint4` | `0.2 N·m·s/rad` | `0` |

内部连杆关节和轮关节仍保留原有阻尼、摩擦和转动惯量参数。

## 传感器

模型提供：

- IMU 姿态、加速度和角速度；
- 六个关节位置和速度；
- 左右气弹簧 tendon 长度和速度；
- 左右气弹簧 actuator 力；
- 六个关节 actuator 力矩。

Python 中读取气弹簧状态：

```python
left_tendon = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_TENDON, "left_gas_spring"
)
left_actuator = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_gas_spring_actuator"
)

length = data.ten_length[left_tendon]
force = data.actuator_force[left_actuator]
```

## 验证结果

当前 `mjmodel.xml` 已验证：

- 可以被 MuJoCo 3.x 正常加载；
- 两根气弹簧 tendon 和 actuator 均能创建；
- home 姿态下安装长度约为 `159.9 mm`；
- 完成短时步进后状态保持有限值，无 NaN 或 Inf；
- `git diff --check` 无格式错误。

## 注意事项

1. 该气弹簧采用图纸曲线的线性化表达，不是完整的热力学气体状态方程。
2. `250–385 N` 是当前模型采用的 9 MPa 保守工作力范围，超过 `195 mm` 时 actuator 力不会继续增大。
3. tendon 行程限制是 MuJoCo 约束，实际数值可能因求解器柔性略微超过边界。
4. 如果需要严格复现不同压力曲线，应增加压力状态变量或使用 MuJoCo plugin，根据压力、有效容积和活塞面积计算非线性力。
