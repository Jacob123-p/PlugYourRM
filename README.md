# PlugYourRM

> 可插拔的 RoboMaster 自瞄跟踪器仿真系统 —— 给社团新生学自瞄用。

## 这是什么

一个**仿真平台**，把 RM 自瞄系统里三层核心算法拆成可插拔的组件，让新生在不接触真车、不调试真实相机的情况下，把自瞄的每一个环节都亲手写一遍：

- **位姿解算** —— 从四个像素点解出装甲板的三维位姿（PnP）
- **跟踪器** —— 把一帧帧离散的观测串成对目标的连续估计（EKF / 多目标关联 / 选板）
- **弹道解算** —— 从打击点到云台角度与飞行时间

平台负责物理、传感器、云台伺服、渲染；你只写算法。

## 它不是什么

- **不是完整比赛系统**。没有裁判系统、没有通信协议、没有真车接口。
- **不是"帮你写好自瞄"**。平台不给参考答案：只给工具（单板 PnP、弹道、滤波器），方案从零写。
- **不是纯视觉项目**。相机是仿真的——你看到的是完美的针孔投影 + 像素量化 + 可选噪声，不用标定、不用对齐、不用处理真实世界的脏数据。

## 快速开始

### 环境

Python 3.10+，依赖 `numpy` / `pyyaml` / `scipy` / `pygame`：

```bash
pip install -e .
```

### 运行

```bash
python main.py
```

### 你会看到

1. **选择界面**：两列 —— `Tracker`（方案）和 `PoseSolver`（位姿解算）。按 `Enter` 逐列确认，进入游戏。
2. **仿真窗口**：左侧俯视图（跟随受控车），右侧相机视图 + 信息面板。
3. **自瞄**：
   - `KP9` —— 开/关自动瞄准（云台自动指向目标）
   - `KP8` —— 开/关自动开火
   - `R` —— 复位

**进场先造一辆车** —— 场地一开始是空的：按 `Enter + 2` 造一辆步兵，它会自动成为受控车。没有受控车就没有相机、没有检测，方案一帧输入都收不到。

**第一次跑起来云台不会动** —— `custom/schemes/your_scheme/` 里的方案还是空的：`solve()` 不产出观测、`step()` 不产出预测。这不是平台坏了，是留给你写的。

### 常用按键

| 键                   | 功能                                     |
| -------------------- | ---------------------------------------- |
| `↑↓←→`             | 底盘平移（场地轴）                       |
| `A/D`              | 底盘慢转，`Z/C` 快转                   |
| `Q/E`              | 云台偏航，`W/X` 俯仰（自瞄关闭时）     |
| `Enter + 1/2/3/4`  | 创建 Hero / Infantry / Sentry / Engineer |
| `Enter + KP1/KP2`  | 创建 Outpost / Rune                      |
| `Backspace + 数字` | 删除对应实体                             |
| `1`                | 切换受控车（场上有几辆车时轮换）         |
| `KP9` / `KP8`    | 自动瞄准 / 自动开火                      |
| `KP0`              | 手动连发（不受自瞄约束，试弹道用）       |
| `M` / `KP4`      | 曲线面板开关 / 切换预设                  |
| `R` / `Space`    | 复位 / 急停                              |
| `ESC`              | 退出当前局，回到选择界面                 |

## 系统结构

三层，加一个装配层：

```
core/       契约 + 数学。不认识 sim / custom。
  bus/        消息定义 + 发布订阅
  interfaces/ Tracker / PoseSolver 基类
  entities/   领域实体（车、云台、装甲板、弹丸）
  algorithms/ 纯数学 + 无方案假设的算法

sim/        世界。物理、传感器、电控、渲染、键盘。
  comm.py     sim 侧通信模块
  simulator.py
  managers/   各子系统
  render/     渲染层

custom/     方案 + 组件。你写代码的地方。
  schemes/    一套方案（PoseSolver + Tracker 实现）
  components/ 可复用的库（PnP、弹道）
  pipeline.py custom 内编排
  comm.py     custom 侧通信模块

entrance/   装配。唯一同时认识 sim 和 custom 的地方。
```

**依赖方向**：`core` 不认识 `sim` / `custom`；`sim` 与 `custom` **互相不认识**——两边靠 `core.bus` 的消息和 `core.interfaces` 的接口相遇。

**数据流**（一帧）：

```
sensor 出检测 ──→ SimComm ──→ [public_bus] ──→ CustomComm
                                                    │
                                                    ↓
                                              pipeline.tick
                                                    │
                                                    ↓
                                          PoseSolver.solve → Observation
                                                    │
                                                    ↓
                                    Tracker.step（跟踪 + 火控）→ Prediction
                                                    │
                            ┌───────────────────────┴───────────────────────┐
                            ↓                                               ↓
                    [public_bus.pred/*]                            [public_bus.control]
                            ↓                                               ↓
                      RenderManager                                  ControlManager
                      （画预测点）                                    （驱动云台 + 开火）
```

## 写你自己的自瞄

**一套方案 = 一个 PoseSolver + 一个 Tracker。** `custom/schemes/your_scheme/` 里已经有骨架，把这两个实现写完，就是一套能跑的自瞄：

```
custom/schemes/your_scheme/
├── __init__.py           包标记（没有它 discover() 扫不到这个目录）
├── your_pose_solver.py   检测 → 观测：像素解成三维位姿
└── your_tracker.py       观测 → 预测 + 火控：跟踪、选板、发 ShotSolution
```

### 1. 位姿解算（先做这个）

`PoseSolver.solve(frame, self_state) -> Observation`：遍历 `frame.detections`，把每块板的四角像素解成三维位姿，打包成 `Observation`。

单板 PnP 在 `custom/components/armor_solvers/` 里给了一份（`ippe`）：用它、换掉它、都不用，都行。真正的难点是换系——PnP 出口在 `C'` 系，你要换到 `G` 或 `W`。坐标约定见 `docs/coordinate_systems.md`。

### 2. 跟踪器（方案主体）

`Tracker.step(obs, self_state) -> List[Prediction]`：一帧观测进，一帧预测出。用 `@register_tracker("my_name")` 注册，启动界面的 Tracker 列就会多一行。

按 `core/interfaces/tracker.py` 的契约走：

```python
from core.interfaces.tracker import Tracker, register_tracker
from core.bus.dataflow import Prediction

@register_tracker("my_first")
class MyFirstTracker(Tracker):
    description = "我的第一个跟踪器"

    def step(self, obs, self_state):
        if obs is None or not obs.obs_armors:
            return []
        # 就拿第一块观测板，直接当作预测点
        armor = obs.obs_armors[0]
        return [Prediction(
            target_type=...,      # 按 entity_type 分类
            center=armor.rel_pos,
            armors=[armor.rel_pos],
            timestamp=obs.timestamp,
            is_tracking=True,
        )]
```

**一帧观测进，一帧预测出**——这是 Tracker 的全部契约。

**Prediction 只喂渲染，云台不认它。** 要让云台动，得在 `step()` 里自己把 `ShotSolution` 发到 `CONTROL` 通道——指南里有最小例子。

**选板、火控、多目标关联都要你自己写** —— `custom/components/` 里只给单板 PnP 和弹道解算。

详细的写作指南见 `docs/writing_your_tracker.md`。

### 3. 换一个 PnP 算法（选做）

在 `custom/components/armor_solvers/` 下新开一个模块（照抄 `ippe.py`），实现

```python
estimate(*, camera_k, armor_size, image_points_2d,
         rot_camera_to_abs_camera, mount_pitch=0.0) -> (pos, rpy) | (None, None)
```

再在 `__init__.py` 的 `_SOLVERS` 里登记一行。没有抽象基类。

## 想再写一套方案

复制 `custom/schemes/your_scheme/` 改个名，换掉 `@register_*` 的名字。**新目录里必须有 `__init__.py`**——`discover()` 只扫包，没有它你写的代码不会被认到（表现是"我明明写了，选择界面里没有"）。

## 项目结构

```
PlugYourRM/
├── main.py
├── pyproject.toml
├── README.md
├── docs/                     # 全部文档
├── config/                   # data.yaml + 生成物
│   ├── data/data.yaml
│   ├── generator.py
│   ├── loader.py
│   ├── reader.py
│   └── schema.py             # 生成物，勿手改
├── core/                     # 契约 + 数学
│   ├── bus/
│   ├── interfaces/
│   ├── entities/
│   └── algorithms/
├── sim/                      # 世界
│   ├── comm.py
│   ├── simulator.py
│   ├── managers/
│   └── render/
├── custom/                   # 方案 + 组件（你写这里）
│   ├── schemes/
│   │   └── your_scheme/      # 你的方案：pose_solver + tracker
│   ├── components/
│   │   ├── armor_solvers/    # 单板 PnP（ippe）
│   │   └── traj_solvers/     # 弹道
│   ├── pipeline.py
│   └── comm.py
└── entrance/                 # 装配
```

## 文档

| 文档                             | 内容                     |
| -------------------------------- | ------------------------ |
| `docs/architecture.md`         | 分层、边界、稳定 vs 变化 |
| `docs/writing_your_tracker.md` | 从零写一个 Tracker       |
| `docs/coordinate_systems.md`   | 坐标系系统               |
| `docs/bus_and_messages.md`     | 总线、消息契约、异步模型 |

## 已知限制

诚实列出：

- **弹道解算给了一份现成的**（`custom/components/traj_solvers/`，带阻力的数值解，不是解析解）。不想用就自己写。
- **单板 PnP 只给了一种**（`ippe`）。想换几何解，自己加一个模块。
- **没有单元测试**。回归靠手动跑仿真。这是最应该补的一块。
- **错误可见性有限**。`EventBus.publish` 的回调异常打到 stderr，不显示在游戏里——游戏里看到"卡住但没报错"时，记得看终端。

## 许可 / 反馈

社团内部使用。问题直接找作者。

---

**下一步**：读 `docs/architecture.md`，然后 `docs/writing_your_tracker.md`
