# MoBuRigLogic

MoBuRigLogic 是面向 **Autodesk MotionBuilder 2024** 的 Windows x64 C++ 插件。插件在 MotionBuilder 求值图中调用 [Epic Games OpenRigLogic](https://github.com/EpicGames/OpenRigLogic)，根据 MetaHuman DNA 计算头部表情和身体修形，并将结果写入对应的骨骼与 BlendShape 属性。

本项目主要面向使用 MetaHuman DNA 的绑定技术美术，以及维护 MotionBuilder C++ 插件的开发者。

> [!IMPORTANT]
> 当前版本只支持 **MotionBuilder 2024**。不支持 MotionBuilder 2022、2023、2025 或 2026。
> MotionBuilder 2019 用户请使用 [`mobu2019` 分支](../../tree/mobu2019)（相同源码，仅构建目标不同，已在 MotionBuilder 2019 中实测）。

## 支持范围

| 项目          | 当前支持                                     |
| ------------- | -------------------------------------------- |
| 操作系统      | Windows x64                                  |
| MotionBuilder | 2024                                         |
| C++ 标准      | C++17                                        |
| 编译器        | MSVC / Visual Studio 2022 C++ 工具链         |
| MSVC Runtime  | `/MD`                                        |
| OpenRigLogic  | OpenRigLogic 13.2.5，项目内置 Release 静态库 |
| 构建系统      | CMake 3.20 或更高版本                        |
| 输出插件      | `build/Release/moburiglogic_2024.dll`        |

## 功能概览

插件注册以下两个约束及其自定义 Layout：

### RigLogic Head Expression

C++ 类型：`RigLogicHeadConstraint`

- 从 `head.dna` 创建 RigLogic 和 RigInstance。
- 支持两种表情输入模式：
  - `Expression Properties`：在约束自身创建可 K 帧的动态表情属性。
  - `FaceBoard Panel`：读取场景中的 `CTRL_*` GUI 控制器（含眼睛摇杆联动，见[眼睛摇杆联动](#眼睛摇杆联动)）。
- 自动读取头部 DNA 所需的颈部和头部局部旋转。
- 输出面部关节的局部平移与旋转。
- 根据 DNA 的 mesh-channel mapping 输出 BlendShape 权重。
- 将 RigLogic 的 `0–1` BlendShape 输出转换为 MotionBuilder 的 `0–100` 属性值。
- 提供 DNA 路径、LOD、输入模式、上次求解耗时、重建绑定和表情归零操作。
- DNA 路径无法加载时，自动回退到 `Source DNA Path` 或绑定 FBX 同目录的固定文件名 `head.dna`（见[DNA 路径回退](#dna-路径回退棚内移动作业)）。

头部约束当前不创建关节缩放输出。

### RigLogic Body Corrective

C++ 类型：`RigLogicBodyConstraint`

- 从 `body.dna` 创建 RigLogic 和 RigInstance。
- 读取 DNA 指定的身体驱动关节局部旋转。
- 在 MotionBuilder 求值图中运行身体修形求解。
- 输出身体修形关节的局部平移、旋转和缩放。
- 提供 DNA 路径、LOD、上次求解耗时和重建绑定操作。
- DNA 路径无法加载时，自动回退到 `Source DNA Path` 或绑定 FBX 同目录的固定文件名 `body.dna`（见[DNA 路径回退](#dna-路径回退棚内移动作业)）。

身体约束当前驱动修形关节，不输出 BlendShape。

## 环境要求

编译前需要安装：

1. Autodesk MotionBuilder 2024。
2. MotionBuilder 2024 OpenReality SDK。
3. Visual Studio 2022，并安装“使用 C++ 的桌面开发”工具。
4. CMake 3.20 或更高版本。
5. Git（仅源码管理需要）。

默认 MotionBuilder 安装目录为：

```text
C:\Program Files\Autodesk\MotionBuilder 2024
```

CMake 会从以下位置读取 OpenReality SDK：

```text
C:\Program Files\Autodesk\MotionBuilder 2024\OpenRealitySDK
```

其中必须包含：

```text
OpenRealitySDK\include
OpenRealitySDK\lib\x64\fbsdk.lib
```

## OpenRigLogic 依赖

OpenRigLogic 的公开头文件、许可证和预编译静态库已经存放在当前工程中：

```text
third_party\OpenRigLogic\
├─ include\
├─ lib\win64\Release\riglogic413_2_5.lib
├─ LICENSE
└─ VERSION.txt
```

因此，编译本项目不再依赖外部的：

```text
D:\Code\OpenRigLogic
```

内置依赖信息：

| 项目              | 值                                                                 |
| ----------------- | ------------------------------------------------------------------ |
| OpenRigLogic 版本 | 13.2.5                                                             |
| 上游分支          | 5.8                                                                |
| 上游提交          | `7bd4c65`                                                          |
| 平台              | Windows x64                                                        |
| 配置              | Release                                                            |
| Runtime           | `/MD`                                                              |
| 静态库            | `riglogic413_2_5.lib`                                              |
| SHA256            | `CED01C2D41090DB41D4194DC64A49FB9E1D75D8B3E7DE19945960BDA92CE7507` |

静态库会直接链接进插件。部署到 MotionBuilder 时不需要复制 `.lib` 和 OpenRigLogic 头文件。

## 项目结构

```text
MoBuRigLogic\
├─ CMakeLists.txt                     # MotionBuilder 2024 构建配置
├─ src\                               # 约束、Layout 和公共求值逻辑
│  ├─ library.cxx                     # MotionBuilder 插件入口
│  ├─ riglogichead_constraint.*       # 头部表情约束
│  ├─ riglogicbody_constraint.*       # 身体修形约束
│  ├─ riglogic_layouts.*              # 自定义约束面板
│  └─ riglogic_common.h               # 公共 LOD、BlendShape 和输出路由逻辑
├─ tests\                             # C++ 单元测试和源码回归检查
├─ third_party\OpenRigLogic\         # 内置 OpenRigLogic 头文件、静态库和许可证
└─ build\                             # CMake 构建输出
```

## 配置与编译

在 PowerShell 中进入工程根目录：

```powershell
cd D:\Code\MoBuRigLogic
```

### 1. 配置工程

```powershell
cmake -S . -B build `
  -DMOBU_ROOT="C:/Program Files/Autodesk/MotionBuilder 2024" `
  -DBUILD_TESTING=ON
```

如果 MotionBuilder 安装在其他目录，请修改 `MOBU_ROOT`。

### 2. 编译 Release

```powershell
cmake --build build --config Release --clean-first
```

成功后会生成：

```text
D:\Code\MoBuRigLogic\build\Release\moburiglogic_2024.dll
```

这里的绝对路径对应当前工作区。如果仓库位于其他目录，输出仍然位于该仓库的：

```text
build\Release\moburiglogic_2024.dll
```

> [!NOTE]
> 插件使用 MotionBuilder 2024 SDK 和 `/MD` Runtime。不要将其他 MotionBuilder 版本的 SDK、头文件或 `fbsdk.lib` 混入当前构建目录。

## 运行测试

配置时启用 `BUILD_TESTING=ON` 后执行：

```powershell
ctest --test-dir build -C Release --output-on-failure
```

当前测试包括：

| 测试                              | 作用                                                                   |
| --------------------------------- | ---------------------------------------------------------------------- |
| `moburiglogic_unit_tests`         | 验证公共 LOD、BlendShape mapping 和属性宿主解析逻辑                    |
| `moburiglogic_source_regressions` | 检查关键源码回归、MotionBuilder 2024 配置和 vendored OpenRigLogic 路径 |

预期结果：

```text
100% tests passed, 0 tests failed out of 2
```

## 安装插件

### 安装或更新

1. 完全关闭 MotionBuilder 2024。
2. 编译 Release 配置。
3. 复制：

   ```text
   D:\Code\MoBuRigLogic\build\Release\moburiglogic_2024.dll
   ```

4. 粘贴到：

   ```text
   C:\Program Files\Autodesk\MotionBuilder 2024\bin\x64\plugins
   ```

5. 启动 MotionBuilder 2024。
6. 在约束列表中检查以下约束是否出现：
   - `RigLogic Head Expression`
   - `RigLogic Body Corrective`

写入 `C:\Program Files` 通常需要管理员权限。本项目不会自动部署 DLL，也不会自动修改 MotionBuilder 安装目录。

### 卸载

1. 关闭 MotionBuilder 2024。
2. 从插件目录删除：

   ```text
   moburiglogic_2024.dll
   ```

## MotionBuilder 使用流程

### 通用准备

1. 在 MotionBuilder 中导入与 DNA 对应的 MetaHuman 骨骼、网格和控制器。
2. 确认场景中的骨骼短名称与 DNA 关节名称一致。
3. 多角色场景中，为每个角色保留独立 namespace。
4. 避免让多个约束同时写入同一批骨骼或 BlendShape 属性。

### 使用头部约束

1. 创建 `RigLogic Head Expression` 约束。
2. 将角色的 `head` 关节拖入约束的 `Skeleton Root` 引用组。
3. 在 `DNA File` 中选择对应的 `head.dna`。
4. 设置 `LOD Level`。
5. 选择 `Input Mode`：
   - `Expression Properties (K-frame on constraint)`：使用约束上的表情属性。
   - `FaceBoard Panel (drive by CTRL_* controls)`：使用 FaceBoard 控制器。
6. 勾选约束的 `Active`。
7. 单击 `Rebuild Bindings`，或在 DNA、LOD、输入模式、角色引用发生变化后重新绑定。
8. 检查状态区中的 expression、panel control、neck joint、joint output 和 BlendShape 数量。
9. 单击 `Zero Expressions` 可让面部表情回到中立位（可用 Ctrl+Z 撤销）：
   - FaceBoard 面板：DNA 读取的全部表情控制器（`CTRL_C_jaw` 等）位移归零。面板框控制器 `CTRL_faceGUI`、`CTRL_faceAndEyesAimFollowHeadGUI`、`CTRL_faceTweakersGUI` 及开关类控制器不被 DNA 读取，不会移动。
   - 表情属性模式：约束上的全部表情属性归零。
   - 控制器若已有关键帧，归零只改当前值，切换帧后会被关键帧覆盖；需要保留请手动 K 帧。
10. 勾选 `Lock Panel Frames` 可让 FaceBoard 面板的框模型（`FRM_*`）不可在视图中选中（即关闭 Enable Selection），方便动画师点选 `CTRL_*` 控制器；需要调整面板时取消勾选即可恢复。只作用于本角色 namespace，状态随场景保存，也可用 Python 设置约束属性 `Lock Panel Frames`。

如果使用 FaceBoard 模式，GUI 控制器不一定是 `head` 的子节点。插件会在相同角色 namespace 内继续查找对应控制器。

### 使用身体约束

1. 创建 `RigLogic Body Corrective` 约束。
2. 将角色的 `pelvis` 关节拖入约束的 `Skeleton Root` 引用组。
3. 在 `DNA File` 中选择对应的 `body.dna`。
4. 设置 `LOD Level`。
5. 勾选约束的 `Active`。
6. 单击 `Rebuild Bindings`，或在 DNA、LOD、角色引用发生变化后重新绑定。
7. 检查状态区中的 driver joint 和 corrective joint 数量。
8. 播放身体动画，检查修形关节是否随驱动关节旋转正确求值。

`Skeleton Root` 引用组只接受一个模型。虽然插件会从该模型向上找到角色层级顶端，但建议头部使用 `head`，身体使用 `pelvis`，便于状态提示、namespace 解析和问题排查。

### DNA 路径回退（棚内/移动作业）

公司室内通常把 DNA 放在固定的共享盘路径，但动画师带绑定去动捕棚、外拍或异地工位时，往往无法创建完全相同的目录结构。为此插件内置了按文件名兜底的能力，并按三级顺序解析：

| 尝试顺序 | 路径 | 说明 |
| --- | --- | --- |
| 1 | `DNA Path` 属性 | 当前实际使用的路径（见下方写回规则） |
| 2 | `Source DNA Path` 属性 | 上次回退时被替换掉的路径，通常是公司共享盘；拿回公司后靠它自动恢复 |
| 3 | `<绑定 FBX 所在目录>/head.dna`（头部）<br>`<绑定 FBX 所在目录>/body.dna`（身体） | 文件名固定，按约束类型各取一份 |

写回规则：

- 只要实际加载的路径不等于 `DNA Path` 属性，插件就把实际路径**写回 `DNA Path`**，并把被替换的原路径存进 `Source DNA Path`（两者都随 FBX 保存）。因此属性上看到的永远是“当前真正在用的那份 DNA”。
- 第二级（`Source DNA Path`）排在第三级之前：公司机器上如果 FBX 旁边恰好也有 DNA，不会把仍然可用的公司路径挤掉。
- 三个候选都是“能加载才算命中”：路径存在性不做预判，直接交给 RigLogic 读取，因此相对路径、网络路径等行为与原来的 `DNA Path` 完全一致。如果三级全部打不开，插件不改动任何属性。

往返行为（公司 ↔ 棚内）：

| 场景 | `DNA Path` | `Source DNA Path` | 命中级别 |
| --- | --- | --- | --- |
| 公司打开（公司路径可用） | 公司路径 | 不变 | 1 |
| 棚内首次打开（只在 FBX 旁放了 DNA） | 公司路径打不开 → 写回棚内路径 | 公司路径 | 3 |
| 棚内存盘后再次打开 | 棚内路径 | 公司路径 | 1 |
| 拿回公司打开 | 棚内路径打不开 → 写回公司路径 | 棚内路径 | 2 |

其他规则：

- 场景未存盘（`FBXFileName` 为空）时不会尝试第三级，避免用相对路径误读到无关文件。
- 状态区会显示实际使用的 DNA 与来源，例如 `DNA: body.dna (from scene folder)` + `Source: //studio/share/dna/body.dna`，或 `DNA: body.dna (restored from Source DNA Path)`；MotionBuilder 日志中另有 `[RigLogicHead]`/`[RigLogicBody]` 的 `using scene folder DNA` / `using Source DNA Path` 记录。
- 因为回退时插件会写回属性，场景打开后会处于“已修改”状态，关闭时可能提示保存。在棚内/异地本来就要另存一个 working 文件，可直接保存。
- 回退生效后会检查 DNA 关节在场景中的命中情况。若存在未找到的 DNA 关节，状态区显示 `Warning: N DNA joints not found in scene`，日志同时输出告警——这通常说明目录里那份 DNA 与当前绑定不匹配（例如同目录放了多个角色的 DNA）。插件只告警不阻断，此时应人工确认 DNA 是否正确。
- 由于文件名固定，**一个目录只能对应一个角色**。需要在一个棚内包中放多个角色时，请分目录存放各自的绑定 FBX 与 DNA。
- 回退只在场景打开（或 `Rebuild Bindings`）时解析。中途把 DNA 拷到 FBX 旁边后，点一次 `Rebuild Bindings` 即可生效。

## 属性说明

### 眼睛摇杆联动

FaceBoard 面板上有三个眼睛摇杆：`CTRL_L_eye`、`CTRL_R_eye`（单眼）和 `CTRL_C_eye`（双眼）。三者在 Maya 里**不是父子关系**，而是在下游汇合：

```text
CTRL_L_eye.tx ─→ 角度曲线 ─┐
                          ├→ 相加（blendWeighted）→ 左眼注视驱动 locator 角度 → eyeLook* 激活值
CTRL_C_eye.tx ─→ 角度曲线 ─┘
右眼同理（另一套曲线 + 另一个 blendWeighted）
```

即：`CTRL_C_eye` 的贡献是**与左右眼摇杆在角度空间相加**，相加结果再经“角度 → 0~1”映射成 DNA 的 `eyeLookLeft/Right/Up/Down` 通道。

MoBu 侧现状与处理：

| 通道 | DNA 里有没有 | MoBu 实现 |
| --- | --- | --- |
| `CTRL_L_eye.tx/ty`、`CTRL_R_eye.tx/ty` | 有（GUI 通道） | 插件 FaceBoard 模式直接驱动，与 Maya 逐值一致 |
| `CTRL_C_eye.tx/ty` | **没有**（只有 `CTRL_C_eye_parallelLook.ty`） | 插件读它的 `Lcl Translation`，叠加到左右眼两条 GUI 通道上 |
| `CTRL_C_eyesAim` / `CTRL_L_eyeAim` / `CTRL_R_eyeAim` | 无 | MoBu 的绑定里不存在，3D 注视控制需另做方案 |

实现细节：

- 叠加发生在**摇杆域**：`左右眼通道值 = clamp(L/R 摇杆 + C 摇杆, -1, 1)`。因为 DNA 的“摇杆值 → 激活值”在 [-1,1] 内是线性的，摇杆域相加与 Maya 的角度域相加等价。
- **必须夹紧**：摇杆量程就是 ±1（满激活），DNA 对超出 [-1,1] 的输入会外插成 0（实测 `tx=1.05` 眼睛直接回中），所以插件对左右眼通道统一 clamp。
- 关闭联动：约束属性 `Eye Linkage` 设为 `0`（或 Python `constraint.EyeLinkage = False`）。
- 状态区显示 `Eye linkage: CTRL_C_eye -> L/R (tx,ty)`；找不到 `CTRL_C_eye` 时显示 `CTRL_C_eye not found` 并在日志记一次。
- `CTRL_C_eye` 必须与 `Skeleton Root` 同 namespace，多角色场景不会串角色（与现有绑定规则一致）。

与 Maya 的实测对照（`FACIAL_*_Eye` 的 Lcl Rotation，单位度）：

| 输入 | Maya 左眼 / 右眼 | MoBu 左眼 / 右眼 |
| --- | --- | --- |
| `CTRL_L_eye.tx=0.5` | (0, 21.1085) / 0 | 一致 |
| `CTRL_R_eye.tx=0.5` | 0 / (0, 18.7924) | 一致 |
| `CTRL_L_eye.ty=0.5` | (−14.7593, 0.1072) / 0 | 一致 |
| `CTRL_C_eye.tx=0.5` | (0, 21.1085) / (0, 18.7924) | 一致（联动叠加） |
| `CTRL_L_eye.tx=0.3 + CTRL_C_eye.tx=0.2` | (0, 21.1085) / (0, 7.5170) | 一致（等价于 `tx=0.5`） |
| `CTRL_C_eye.ty=0.5` | (−14.7593, 0.1072) / (−14.8516, 0) | 一致 |
| `CTRL_L_eye.ty=0.5 + CTRL_C_eye.ty=0.25` | (−22.1389, 0.1609) / (−7.4258, 0) | 一致 |
| `CTRL_L_eye.tx=0.8 + CTRL_C_eye.tx=0.5`（溢出） | (0, 42.2170) / (0, 18.7924) | 一致（夹紧到 1.0 后饱和） |

已知差异：Maya 在摇杆之和超过 ±1 时，`LOC_*_eyeUIDriver` 的角度会继续放大（例如 80°），但注视激活值同样饱和在 1；MoBu 侧以激活值为准，所以眼球不会“过转”。正常作画范围（±1 以内）无差异。

### 共同属性

| 属性                                | 说明                                                                     |
| ----------------------------------- | ------------------------------------------------------------------------ |
| `DNA Path` / `DnaPath`              | 当前**实际使用**的 DNA 文件路径；回退命中时会自动写回实际路径            |
| `Source DNA Path` / `SourceDnaPath` | 只读：上次回退时被替换掉的路径（通常是公司共享盘），用于回到公司后自动恢复 |
| `LOD Level` / `LodLevel`            | RigLogic 求值和输出绑定使用的 LOD；超出范围时会限制到 DNA 的有效范围     |
| `Last Solve Ms` / `LastSolveMs`     | 上一次完整 RigLogic 求解耗时，单位为毫秒，只读                           |

### 头部专用属性

| 属性                       | 说明                                            |
| -------------------------- | ----------------------------------------------- |
| `Input Mode` / `InputMode` | `0` 为动态表情属性，`1` 为 FaceBoard GUI 控制器 |
| `Eye Linkage` / `EyeLinkage` | `1`（默认）：把 `CTRL_C_eye` 的位移叠加到左右眼摇杆，复现 Maya 的双眼联动；`0` 关闭联动，只读 `CTRL_L_eye` / `CTRL_R_eye` |

不同 MetaHuman DNA 的输入、输出、LOD 和 BlendShape 数量可能不同。README 不把某个角色的固定数量作为所有 DNA 的保证。

## 模型名称、namespace 与 BlendShape 规则

### 骨骼名称

绑定以 DNA 中的关节名和控制名为依据。插件从 `Skeleton Root` 所属角色中提取 namespace，然后使用剥离 namespace 后的名称匹配 DNA。

例如：

```text
MotionBuilder LongName:  HeroA:head
MotionBuilder Name:      head
DNA joint name:          head
解析出的 namespace:     HeroA:
```

因此：

- 角色名称和 namespace 可以变化。
- 不需要所有角色使用固定的完整 `LongName`。
- 应保留 DNA 期望的骨骼短名称，例如 `head`、`pelvis` 和对应的面部/身体关节名。
- 不建议任意修改 DNA 关节短名称。
- 多角色场景应将正确角色的 `head` 或 `pelvis` 拖入对应约束，避免跨 namespace 绑定。

### BlendShape 属性

头部 BlendShape 属性名按 DNA mesh-channel mapping 组合：

```text
<DNA mesh name>__<DNA blendshape channel name>
```

插件会优先在与 DNA mesh 名称匹配的模型上查找该可动画属性；如果该模型不存在或不包含属性，则会在同一角色 namespace 的其他模型中查找完全相同的可动画属性。

这意味着：

- 制作绑定时不强制要求所有角色使用固定的 Mesh 对象名。
- Mesh 改名后，只要相同角色 namespace 中仍存在 DNA 期望的完整 BlendShape 属性名，插件仍有机会解析到正确宿主。
- BlendShape 属性名本身必须与 DNA mapping 一致，并且属性必须可动画。
- 多个模型包含同名属性时可能产生歧义，应通过约束状态、MotionBuilder 日志和实际驱动结果确认绑定对象。

## FBX 保存与重新加载

约束的 DNA 路径、LOD、输入模式和引用关系会通过 MotionBuilder/FBX 对象系统保存。重新打开场景后应检查：

1. DNA 文件路径仍然有效；不可用时会自动回退到 `Source DNA Path` 或绑定 FBX 同目录的 `head.dna`/`body.dna`，并把实际路径写回 `DNA Path`（状态区会标明来源）。
2. `Skeleton Root` 仍指向正确角色。
3. 当前角色 namespace 未发生不兼容变化。
4. 约束处于 `Active` 状态。
5. 状态区显示 `Bindings OK`。

如果角色被替换、DNA 路径改变或模型名称发生变化，请执行 `Rebuild Bindings`。

## 常见问题

### MotionBuilder 中没有出现约束

检查：

- DLL 是否位于 MotionBuilder 2024 的 `bin\x64\plugins`。
- 文件名是否为 `moburiglogic_2024.dll`。
- 是否在复制 DLL 前关闭了 MotionBuilder。
- DLL 是否确实使用 MotionBuilder 2024 SDK 和 Windows x64 Release 配置编译。
- MotionBuilder 启动日志中是否存在插件加载或依赖错误。

### DLL 无法覆盖或删除

MotionBuilder 加载插件后会占用 DLL。先关闭所有 MotionBuilder 2024 进程，再更新或删除插件。

### DNA 加载失败

检查：

- `DNA File` 路径是否存在且可读。
- 头部约束是否使用 `head.dna`，身体约束是否使用 `body.dna`。
- DNA 是否与当前角色资产匹配。
- OpenRigLogic 是否支持该 DNA 数据版本。
- MotionBuilder 日志中的 `[RigLogicHead]` 或 `[RigLogicBody]` 信息。
- 若依赖回退：绑定 FBX 是否已保存到磁盘；`head.dna`/`body.dna` 是否与该 FBX 在同一目录且文件名严格一致；`Source DNA Path` 指向的那份 DNA 是否真的不可用。

### DNA 已加载，但 Bindings NOT ready

检查：

- 是否将 `head` 或 `pelvis` 放入了 `Skeleton Root` 引用组。
- 引用骨骼是否属于正确角色和 namespace。
- DNA 关节短名称与场景骨骼短名称是否一致。
- LOD 是否适合当前角色资源。
- 约束是否已激活。
- 修改后是否单击了 `Rebuild Bindings`。

### 骨骼没有被驱动

检查：

- 约束状态是否为 `Bindings OK`。
- 输入和输出计数是否大于零。
- 是否有其他约束、Relation、Character 或动画层同时占用输出属性。
- 身体输入是否来自局部旋转，而不是全局 Rotation。
- 时间轴求值和播放时是否能看到 `Last Solve Ms` 更新。

### BlendShape 没有被驱动

BlendShape 仅由头部约束输出。检查：

- 当前 LOD 是否包含对应 mesh-channel mapping。
- 属性名是否严格符合 `<meshName>__<channelName>`。
- 属性是否位于同一角色 namespace 中。
- 属性是否可动画。
- 状态区中的 BlendShape 输出数量是否大于零。
- MotionBuilder 日志中的 requested、bound 和 missing mapping 数量。

### CMake 找不到 fbsdk.lib

确认 `MOBU_ROOT` 指向 MotionBuilder 2024 安装目录，并检查：

```text
<MOBU_ROOT>\OpenRealitySDK\lib\x64\fbsdk.lib
```

不要将 `MOBU_ROOT` 设置为 `OpenRealitySDK` 子目录本身。

### CMake 提示 vendored OpenRigLogic 文件缺失

确认以下文件仍存在：

```text
third_party\OpenRigLogic\include\riglogic\RigLogic.h
third_party\OpenRigLogic\lib\win64\Release\riglogic413_2_5.lib
third_party\OpenRigLogic\LICENSE
```

如果这些资产被删除，请从版本控制恢复 `third_party/OpenRigLogic`，不要临时回退到外部绝对路径。

### CMakeCache 指向旧的源码目录

仓库移动位置或复制构建目录后，CMake 可能报告缓存目录不匹配。删除旧的 `build\CMakeCache.txt` 和 `build\CMakeFiles`，或使用新的构建目录重新配置：

```powershell
cmake -S . -B build_fresh `
  -DMOBU_ROOT="C:/Program Files/Autodesk/MotionBuilder 2024" `
  -DBUILD_TESTING=ON
```

## 并发与稳定性（动捕 / 长时播放）

插件与 MoBu 的求值线（可能多线程）并行工作，为此做了以下保证：

- 绑定表（输入/输出/路由）**只在锁内访问**；`ReadData`/`WriteData` 在锁外调用（读可能触发上游 HIK 求值并回到本约束，持锁会自锁死）。
- **播放/录制中不会重建绑定、不会重载 DNA**：重建请求会挂起，等运输器停止且空闲稳定（约 1 秒）后自动补做。
- MoBu 拆掉本约束的动画节点时（`RemoveAllAnimationNodes`），插件立即锁内失效全部绑定，避免求值线继续使用已释放节点。

现场使用规则（重要）：

| 操作 | 播放/录制中 | 说明 |
| --- | --- | --- |
| 点 `Rebuild Bindings` / 拖 `LOD Level` / 改 `DNA Path` | 可以（会自动延后） | 停播后自动生效 |
| 改动 `Skeleton Root` 引用组、拖入新角色、Merge | **禁止** | 这是 MoBu 自己发起的节点拆除，插件无法拦截；请先停播 |
| Plot（Animation → Plot） | 先停播 | Plot 会逐帧写缓存；Plot 后建议重建绑定或关闭 `Active`，避免“关键帧 + 约束”双重驱动 |
| 删除/替换面模、换角色 | 先取消 `Active` 或删除约束 | 避免输出节点指向已删除的宿主 |

一个角色一个约束（头部 + 身体各一个），不要多个约束写同一批关节。验收建议：开绑 → 播放 30 秒 → 录一段 → 停 → Plot → 回放核对。

## 开发与维护注意事项

- 当前 CMake 配置固定输出 `moburiglogic_2024.dll`。
- 不要把 MotionBuilder 2019 或其他版本的 SDK 路径重新加入当前构建。
- 更新 OpenRigLogic 静态库时，必须同步更新公开头文件、`VERSION.txt`、许可证和 SHA256。
- 预编译 OpenRigLogic 静态库必须与 Windows x64、Release 和 `/MD` 配置兼容。
- 修改约束绑定或求值逻辑后，应同时运行 C++ 单元测试和源码回归测试。
- 重建路径改动必须保留三件事：播放中不重建、重建前锁内失效旧绑定、`ReadData`/`WriteData` 不进锁。
- 部署测试应在关闭 MotionBuilder 后手动更新 DLL。

## 项目许可证

MoBuRigLogic 项目自身源码采用 [MIT License](LICENSE)。OpenRigLogic 等第三方组件仍遵循各自的许可证，项目的 MIT License 不会替代或覆盖第三方许可证。

## 第三方组件与许可证

OpenRigLogic 版本和构建信息见：

```text
third_party\OpenRigLogic\VERSION.txt
```

OpenRigLogic 许可证见：

```text
third_party\OpenRigLogic\LICENSE
```

分发插件或第三方资产前，请确认项目自身的发布策略以及 OpenRigLogic 许可证要求。README 不替代许可证原文。
