# 与 Maya 对比身体修形

用同一套姿态表，分别在 **Maya 2024（MetaHumanForMaya）** 和 **MotionBuilder 2019（本插件）** 中
驱动同一套 MetaHuman 绑定，逐关节对比 RigLogic 身体修形（不含表情）的输出差异。

用途：

- 验证本插件的身体修形与 Maya 侧是否一致（回归门禁）；
- 定位差异是「输入姿势不一致」「本地 TRS 输出不一致」还是「父级缩放/层级不一致」；
- 换 DNA、OpenRigLogic 版本或修形求值逻辑后，快速确认没有引入回归。

## 测试口径

| 项目 | 说明 |
| --- | --- |
| 驱动骨骼 | DNA raw control 关节（本 DNA 为 44 根），两侧施加**相同的局部欧拉增量** |
| 修形关节 | RigLogic 输出关节（本 DNA 为 258 根） |
| 对比量 | ① 修形关节世界位置（可见效果）② 本地 TRS 增量（输出值本身） |
| 判定 | 世界位置的**动态**偏差（扣除静置基准）与旋转输出偏差都不超过阈值 |
| 姿势 | `poses.json`，35 个：肩/肘/腕/髋/膝/踝/锁骨/手指 curl/组合/左右镜像 |

## 文件

```text
tools\compare_with_maya\
├─ poses.json     # 测试姿态表（两侧共用）
├─ capture.py     # 采样实现（同时兼容 Maya py3 与 MoBu py2.7）
├─ run_maya.py    # Maya 侧入口
├─ run_mobu.py    # MoBu 侧入口
├─ compare.py     # 对比与报告（普通 Python 3）
└─ out\           # 采样数据与报告（git 忽略）
```

## 使用流程

顺序固定：**Maya 采样 → MoBu 采样 → 对比**。Maya 侧会导出 `out/names.json`
（驱动/修形关节名单），MoBu 侧复用它，确保两侧对比同一批骨骼。

### 1. Maya 采样

打开带 MetaHuman 绑定（含 `body_rl4Embedded` 节点）的场景，在脚本编辑器执行：

```python
import runpy
runpy.run_path(r"D:\Code\MoBuRigLogic\tools\compare_with_maya\run_maya.py", run_name="__main__")
```

通过 MayaMCP 调用时同理：

```python
def main():
    import runpy
    return runpy.run_path(r"D:\Code\MoBuRigLogic\tools\compare_with_maya\run_maya.py",
                          run_name="__main__")
```

### 2. MotionBuilder 采样

确认场景里有 `RigLogic Body Corrective` 约束（DNA 与 Maya 侧相同、LOD 相同），然后执行：

```python
import runpy
runpy.run_path(r"D:\Code\MoBuRigLogic\tools\compare_with_maya\run_mobu.py", run_name="__main__")
```

### 3. 对比

```powershell
python tools\compare_with_maya\compare.py
```

输出：

```text
out\report.md       # 完整报告（逐姿势、逐部位、超阈值关节）
out\summary.json    # 机器可读摘要，供 CI 判定
```

常用参数：

```powershell
python tools\compare_with_maya\compare.py --pos-mm 0.2 --rot-deg 0.2   # 收紧阈值
python tools\compare_with_maya\compare.py --no-fail                    # 只出报告
python tools\compare_with_maya\compare.py --out-dir D:\tmp\mh_cmp      # 换输出目录
```

退出码：`0` 通过，`1` 超阈值（可用 `--no-fail` 关掉）。

默认阈值 **0.8 mm / 0.6°** 是在当前基线（0.668 mm / 0.478°）上留了约 20% 余量，用来拦“变差”
而不是拦“有差异”；需要更严的指标时用 `--pos-mm` / `--rot-deg` 收紧。

> 采样脚本会临时改写驱动骨骼的旋转属性并在结束时还原。Maya 场景会被标记为“已修改”，
> 不要在采样过程中保存场景；MoBu 侧同理。

## 基线结果（H10100 / body.dna，LOD 0）

| 指标 | 实测 |
| --- | --- |
| 驱动骨骼姿态偏差 | 0.00024 mm（float32 量级，两侧确实摆在同一姿势） |
| 平移修形输出 | 完全一致 |
| 缩放修形输出 | 完全一致 |
| 旋转修形输出 | 最大 0.48°，多数 <0.03° |
| 修形关节世界位置 | 246 / 258 根在所有姿势完全一致；最大动态偏差 0.668 mm |
| 腿部（髋/膝/踝） | 0.000 mm，逐值一致 |
| 躯干/锁骨、上臂/前臂、手腕 | ≤0.05 mm |

已知差异：8 根手指 `_half_`（半段/twist 类）辅助关节存在最大 1.07° 的静置旋转差，
且响应增益有约 4% 差异（滑扫 `middle_01_l` 时 Maya 侧 0.955、MoBu 侧 0.995），
沿手指链放大到 0.7 mm 量级。已排除 DNA/LOD 不同、驱动骨骼属性基准不同、
链上非均匀缩放、ML 行为（`body_rl4Embedded.enableML` 开关对输出无影响）、
平移/缩放通路等因素；两侧 riglogic 二进制不同（Maya 内置版 vs OpenRigLogic 13.2.5）
是最可能的原因。若需要仲裁哪侧更接近 DNA 真值，请用 Unreal 或同版本 OpenRigLogic 作参考。

## 实现注意

- MoBu 插件的缩放输出写在 **`Lcl Scaling`**，`Scaling` 属性恒为 1；读错属性会误判成“缩放不一致”。
- MoBu 侧读取驱动骨骼的**全局旋转**反推局部（HIK 激活时 `Lcl Rotation` 的动画节点读到的是全局值），
  所以采样用 `Scene.Evaluate()` 触发求值；Maya 侧用 `refresh(force=True)` 触发 DG 求值。
- 对比用「世界位置」与「本地 TRS 增量」，避免两侧 euler 分解差异造成的误判
  （例如 clavicle 的 `jointOrient` 与 `PreRotation` 是等价解，数值不同但旋转相同）。
- 入口脚本会 `sys.modules.pop("capture")` 后再 import：Maya/MoBu 是常驻进程，
  否则修改 `capture.py` 后重复运行会命中旧模块（Python 2 还会生成同名 `.pyc`）。
- `capture.py` 同时要跑在 Maya 2024（py3）与 MotionBuilder 2019（py2.7）：
  不用 f-string / 类型注解，JSON 统一以二进制写入再 `encode`。
