# 与 Maya 对比修形

用同一套姿势表，分别在 **Maya 2024（MetaHumanForMaya）** 和 **MotionBuilder 2019（本插件）** 中
驱动同一套 MetaHuman 绑定，逐项对比 RigLogic 修形结果。

用途：

- 验证本插件的身体修形 / 面部修形与 Maya 侧是否一致（回归门禁）；
- 定位差异是「输入姿势不一致」「输出值不一致」还是「父级缩放/层级不一致」；
- 换 DNA、升级 OpenRigLogic 或改动求值逻辑后，快速确认没有引入回归。

## 三套 profile

| profile | 输入 | 采集输出 | 判定指标 |
| --- | --- | --- | --- |
| `body` | 44 根 DNA raw control 驱动骨骼的局部欧拉增量，35 个定向姿势 | 258 根修形关节（世界矩阵 + 本地 TRS） | 关节世界位置动态偏差、旋转输出偏差 |
| `body_sweep` | 同上，但 44 根驱动骨骼 **逐根逐轴 ±45°**（265 个姿势） | 同上 | 同上 |
| `face` | FaceBoard 控制器 `CTRL_*` 的 translate（0..1），27 个定向姿势（含眼球朝向） | 840 根 `FACIAL_*` 关节、858 个 BlendShape 通道、5 个 DNA 网格 | 关节位置/旋转增量、BlendShape 权重 |
| `face_sweep` | 同上，但逐个控制器单独摆到 1.0（Maya 自动发现控制器） | 同上 | 同上 |
| `head` | `neck_01` / `neck_02` / `head` 的局部旋转（head.dna 的颈部输入），16 个姿势 | 同上（面部输出） | 同上 |

两侧输入路径不同但等价：

- Maya：`CTRL_*` translate → `animCurveUU` → `CTRL_expressions.*`（0..1）→ RigLogic raw control；
  body 则是直接改 `rotate`（raw control 关节的局部旋转）。
- MoBu：face 走 `CTRL_*` **Lcl Translation** → RigLogic **GUI control**（RigLogic 内部 GUI→raw 映射）；
  body 由插件从全局旋转反推局部后写入 raw control。

## 文件

```text
tools\compare_with_maya\
├─ poses.json             # body 定向姿态表
├─ poses_body_sweep.json  # body 全驱动扫描配置（±45°）
├─ poses_face.json        # face 定向姿态表（含眼球朝向）
├─ poses_face_sweep.json  # face_sweep 配置（扫描值、跳过列表）
├─ poses_head.json        # head（颈部/头部）姿态表
├─ capture.py             # 采样实现（profile 化，兼容 Maya py3 与 MoBu py2.7）
├─ run_maya.py            # Maya 侧入口
├─ run_mobu.py            # MoBu 侧入口
├─ compare.py             # 对比与报告（普通 Python 3）
└─ out\                   # 采样数据与报告（git 忽略）
```

## 使用流程

顺序固定：**Maya 采样 → MoBu 采样 → 对比**。Maya 侧会导出 `out/names_<profile>.json`
（关节/通道/控制器名单），MoBu 侧复用它，确保两侧对比同一批对象。

### 1. Maya 采样

打开带 MetaHuman 绑定的场景，在脚本编辑器或 MayaMCP 中执行：

```python
import runpy
runpy.run_path(r"D:\Code\MoBuRigLogic\tools\compare_with_maya\run_maya.py", run_name="__main__")
```

### 2. MotionBuilder 采样

确认场景里有对应约束（`RigLogic Body Corrective` / `RigLogic Head Expression`，DNA 与 LOD 与 Maya 一致；
面部还需要 `Input Mode = FaceBoard Panel`），然后执行：

```python
import runpy
runpy.run_path(r"D:\Code\MoBuRigLogic\tools\compare_with_maya\run_mobu.py", run_name="__main__")
```

### 3. 对比

```powershell
python tools\compare_with_maya\compare.py --profile body
python tools\compare_with_maya\compare.py --profile body_sweep
python tools\compare_with_maya\compare.py --profile face
python tools\compare_with_maya\compare.py --profile face_sweep
python tools\compare_with_maya\compare.py --profile head
```

默认 profile 是 `body`。切换 profile 时在应用里设置环境变量：

```python
import os
os.environ["MH_CMP_PROFILE"] = "face"      # 或 face_sweep
```

输出：

```text
out\report_<profile>.md      # 完整报告
out\summary_<profile>.json   # 机器可读摘要，供 CI 判定
```

常用参数：

```powershell
python tools\compare_with_maya\compare.py --profile face --rot-deg 0.05   # 放宽阈值
python tools\compare_with_maya\compare.py --profile body --no-fail        # 只出报告
python tools\compare_with_maya\compare.py --profile body --out-dir D:\tmp\cmp
```

退出码：`0` 通过，`1` 超阈值（可用 `--no-fail` 关掉）。默认阈值：

| profile | 位置 | 旋转 | BlendShape（0..100） |
| --- | --- | --- | --- |
| `body` | 0.8 mm | 0.6° | — |
| `body_sweep` | 1.5 mm | 3.5° | — |
| `face` / `face_sweep` / `head` | 0.01 mm | 0.01° | 0.01 |

> 采样脚本会临时改写骨骼/控制器属性（面部还会临时解锁 FaceBoard 的通道锁），结束时自动还原；
> 但 Maya 场景会被标记为“已修改”，不要在采样过程中保存场景。MoBu 侧同理。

## 基线结果（H10100 / body.dna + head.dna / LOD 0）

### body（35 姿势）

| 指标 | 实测 |
| --- | --- |
| 驱动骨骼姿态偏差 | 0.00024 mm |
| 平移修形输出 | 完全一致 |
| 缩放修形输出 | 完全一致 |
| 旋转修形输出 | 最大 0.478°，多数 <0.03° |
| 修形关节世界位置 | 246 / 258 根在所有姿势完全一致；最大动态偏差 0.668 mm |
| 腿部（髋/膝/踝） | 0.000 mm |
| 躯干/锁骨、上臂/前臂、手腕 | ≤0.05 mm |

已知差异：8 根手指 `_half_`（半段/twist 类）辅助关节存在静置旋转差（最大 1.07°），
响应增益有约 4% 差异，沿手指链放大。已排除 DNA/LOD、驱动骨骼属性基准、
链上非均匀缩放、ML 行为、平移/缩放通路等因素；两侧 riglogic 二进制不同
（Maya 内置版 vs OpenRigLogic 13.2.5）+ 插件自建的 raw control 反推路径是最可能的原因。

### body_sweep（44 根驱动骨骼 × 3 轴 × ±45°，265 姿势）

定向姿势只跳着摆了部分驱动骨骼，**全驱动扫描后差异比 `body` 更大**：

| 指标 | 实测 |
| --- | --- |
| 驱动骨骼姿态偏差 | 0.00014 mm |
| 修形关节动态位置差 | 1.175 mm（`middle_01_palm_r`） |
| 修形旋转输出差 | 2.843° |
| 腿部、躯干/锁骨、上臂/前臂、手腕 | 0.000 mm（全部 44 根驱动、±45° 下仍逐位一致） |
| 手指 | 1.175 mm / 2.843° |

最大差异出现在手指的 Z 轴（twist）拧到 ±45° 时：`middle_01_l/r`、`index_03`、`middle_03`、
`ring_02/03` 这些带动 `_half_` 修形的关节。这也说明**扭曲类输入越饱和，两侧增益差越明显**。

### face（27 姿势，含 `CTRL_L_eye` / `CTRL_R_eye` 的眼球朝向）

| 指标 | 实测 |
| --- | --- |
| 关节世界位置差 | 9.1e-05 mm（0.09 µm） |
| 关节本地旋转增量差 | 1.6e-06° |
| BlendShape 权重差 | 0（0..100 制） |
| DNA 网格位姿 | 完全一致 |

### face_sweep（178 个控制器 × 逐个 1.0）

| 指标 | 实测 |
| --- | --- |
| 关节世界位置差 | 9.1e-05 mm |
| 关节本地旋转增量差 | 1.4e-04° |
| BlendShape 权重差 | 3.3e-04 |
| 眼球朝向（`CTRL_L_eye` / `CTRL_R_eye` 的 tx/ty） | 3.5e-07° |
| 有实际响应的控制器 | 约 4/5（其余在本 DNA 的 LOD0 上没有可见输出，两侧表现一致） |

面部差异都在 float32 舍入量级，等同于逐位一致：两侧最终喂给 RigLogic 的是同一组 raw control 值
（Maya 用 animCurveUU 得到 0..1 表情值，MoBu 用 DNA 自带的 GUI→raw 映射，两者等价），
再跑同一套 RigLogic 求值。身体修形的 0.7 mm 差异不来自 RigLogic 求值，而来自插件自己
「由全局旋转反推局部 raw control」的输入路径。

## 覆盖范围与已知限制

### 两侧 RigLogic 实际吃哪些输入

| DNA | 输入 | 数量 |
| --- | --- | --- |
| `body.dna` | 44 根骨骼 × 3 轴局部旋转（`calf/clavicle/foot/hand/lowerarm/thigh/upperarm` + 五指 `01-03`，左右各一套） | 132 |
| `head.dna` | 252 个表情控制（`CTRL_expressions.*`）+ `neck_01` / `neck_02` / `head` × 3 轴局部旋转 | 252 + 9 |

所以：**spine / pelvis 不是任何一侧的输入**（驱动它们不会有修形响应）；
手指 `_04` 段、metacarpal 也不是输入；颈部的两节（`neck_01`/`neck_02`）和 `head` 属于**头部**输入，
已在 `head` profile 里覆盖。

### 各 profile 的覆盖

| 输入面 | 覆盖情况 |
| --- | --- |
| body 44 根驱动骨骼 | `body_sweep` 逐根逐轴 ±45° 全覆盖 |
| head 的 252 个表情 | `face_sweep` 逐个控制器覆盖（178 个控制轴，`CTRL_C_eye` 除外） |
| head 的 `neck_01`/`neck_02`/`head` | `head` profile 单轴 + 组合共 16 个姿势 |
| 输出侧 | 840 根 `FACIAL_*`、258 根身体修形关节、858 个 BlendShape、5 个 DNA 网格 |
| `neck_01`/`neck_02`/`head` → 面部 | `head` profile：位置 1.1e-04 mm、旋转 5.0e-06°、BlendShape 3.0e-05（两侧一致） |

### 尚未覆盖

1. **LOD > 0**：所有 profile 都用场景当前 LOD（本次为 0）。换 LOD 会改变驱动集/通道映射，建议单独跑一轮。
2. **HIK 激活状态**：本次 MotionBuilder 侧的 Character 是 **inactive**。插件关于“HIK 激活时驱动输入改由全局旋转反推局部”的修复
   在 `head`/`body_sweep` 里间接验证了同样的机制（全局→局部），但真 HIK 驱动下的姿态未覆盖。
3. **`CTRL_C_eye`（双眼同步朝向）**：Maya 面板便利控制器，DNA GUI 未声明，MoBu 无法驱动；已列入 `skip`。
4. **非 `CTRL_*` 的绑定层行为**（IK、拉伸、FacialRoot 跟随头部的权重等）不在对比范围内——只比 RigLogic 修形。

### 面板与求值层面的已知差异

1. **眼球朝向可以正常驱动，两侧一致。**
   FaceBoard 上 `CTRL_L_eye` / `CTRL_R_eye` 的 **translate**（tx = 左右、ty = 上下，量程 -1..1）
   就是眼球朝向的输入：Maya 经 `animCurveUA` + `unitConversion` 混进 `LOC_L/R_eyeDriver`，
   再转成 `CTRL_expressions.eyeLook*`；MoBu 直接读这两个控制器的 Lcl Translation。
   实测 tx=+1 / ty=+1 时两侧 `FACIAL_L_Eye` 旋转逐位相同（42.217° / −29.519°）。
2. **唯一的面板覆盖差异：`CTRL_C_eye`（双眼同步朝向）。**
   它是 Maya 侧的面板便利控制器（驱动 `LOC_C_eyeDriver` → 两只眼），DNA GUI 没有声明它，
   所以 MoBu 的 FaceBoard 模式无法驱动；已列入 `poses_face_sweep.json` 的 `skip`。
   Maya 的 `CTRL_L/R_eyeAim`、`LOC_*_eyeDriver`、`lookDir*` 等 aim 骨架同理也不在 MoBu 场景里，
   但眼球朝向并不依赖它们（用 `CTRL_L/R_eye` 即可）。
3. **部分控制器在本 DNA 上无可见输出**（`nose_wrinkleUpper` 静置已饱和在 1、`mouth_lipsTogether*`、
   `mouth_stretchLipsClose`、`neck_swallow`、`teeth*`、`jaw_openExtreme` 等），两侧表现一致，
   不是绑定问题；`face_sweep` 的报告会逐条列出。
4. **DNA 网格的 transform 不动**属正常：面部的顶点变形来自骨骼 + BlendShape，报告里的网格位姿列恒为 0。
5. **一个场景里多个不同 DNA 的角色**时，`body` 的关节名/`face` 的通道名会混在一起；
   建议一次只对比一个角色，或使用不同 namespace 后只打开一个角色的可见性。

## 实现注意

- MoBu 插件的缩放输出写在 **`Lcl Scaling`**，`Scaling` 属性恒为 1；读错属性会误判成“缩放不一致”。
- MoBu 侧读取驱动骨骼的**全局旋转**反推局部（HIK 激活时 `Lcl Rotation` 的动画节点读到的是全局值），
  所以采样用 `Scene.Evaluate()` 触发求值；Maya 侧用 `refresh(force=True)` 触发 DG 求值。
- 对比用「世界位置」与「本地 TRS 增量」，避免两侧 euler 分解差异造成的误判
  （例如 clavicle 的 `jointOrient` 与 `PreRotation` 是等价解，数值不同但旋转相同）。
- FaceBoard 控制器在 Maya 里对 X/Z 轴设了通道锁（`setAttr -lock on`），采样时临时解锁、
  结束后按原值**逐轴还原**（`capture.py` 的 `finally` 分支），否则会因为“锁状态被改掉”污染用户的绑定。
- 入口脚本会 `sys.modules.pop("capture")` 后再 import：Maya/MoBu 是常驻进程，
  否则修改 `capture.py` 后重复运行会命中旧模块（Python 2 还会生成同名 `.pyc`）。
- `capture.py` 同时要跑在 Maya 2024（py3）与 MotionBuilder 2019（py2.7）：
  不用 f-string / 类型注解，JSON 统一以二进制写入再 `encode`；
  MoBu 的 `FBPropertyManager.Find` 只接受 `char*`，传名字前要 `str()`（py2 下 JSON 给的是 unicode）。
- `face_sweep` 的控制器列表由 Maya 侧生成（扫描动画曲线的上游 `CTRL_*`），
  按 `(控制器, 轴)` 去重——同一个轴上可能挂多条曲线（例如 `CTRL_L_eye_blink.translateY`
  同时驱动 `eyeBlinkL` 与 `eyeWidenL`），去重后每个物理控制轴只摆一次。
- 扫描动画曲线时**不能只认 `animCurveUU`**：眼球朝向（`CTRL_L_eye` / `CTRL_R_eye`）走的是
  `animCurveUA`（无单位→角度），只扫 UU 会把整条眼珠控制链漏掉，看起来像“眼球不可驱动”。
