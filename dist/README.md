# dist —— 插件发布产物

本目录存放已编译好的 MoBuRigLogic 插件 DLL，供直接部署使用。**不要手动修改这里的文件**，它们由 `build/`、`build2019/` 的编译结果复制而来。

## 产物清单

| 文件 | 目标 MotionBuilder | 源码分支 | 源码提交 | 构建目录 |
| --- | --- | --- | --- | --- |
| `moburiglogic_2019.dll` | 2019 | `mobu2019` | `ae20447` | `build2019/` |
| `moburiglogic_2024.dll` | 2024 | `main` | `36c648d` | `build/` |

两个提交为同一功能节点（`feat: Head 约束新增 Lock Panel Frames`）在两条分支上的平行提交。

编译日期：2026-10-10　配置：Release / x64 / MSVC 2022 / `/MD` / C++17

## 版本对应关系

> [!IMPORTANT]
> DLL 与 MotionBuilder 版本**严格一一对应**，不可混用。2019 的 DLL 无法在 2024 中加载，反之亦然（SDK 与 ABI 不同）。

- MotionBuilder 2019 → `moburiglogic_2019.dll`
- MotionBuilder 2024 → `moburiglogic_2024.dll`

当前不支持 MotionBuilder 2022、2023、2025、2026。

## 安装方式

1. **完全关闭 MotionBuilder**（运行时会锁定插件 DLL，不关闭则替换失败）。
2. 将对应版本的 DLL 复制到 MotionBuilder 插件目录：

   ```text
   <MotionBuilder 安装目录>\bin\x64\plugins\
   ```

3. 重新启动 MotionBuilder，在 `Character > Constraints` 中应能看到 `RigLogic Head Expression` 与 `RigLogic Body Corrective`。

目标机器需安装 Microsoft Visual C++ 2015-2022 Redistributable (x64)。

## 重新生成产物

先切到对应分支，再用**配套的构建目录**编译，最后复制到本目录：

```powershell
# MotionBuilder 2019 版本
git checkout mobu2019
cmake --build build2019 --config Release --clean-first
ctest --test-dir build2019 -C Release --output-on-failure
Copy-Item build2019\Release\moburiglogic_2019.dll dist\ -Force

# MotionBuilder 2024 版本
git checkout main
cmake --build build --config Release --clean-first
ctest --test-dir build -C Release --output-on-failure
Copy-Item build\Release\moburiglogic_2024.dll dist\ -Force
```

> [!WARNING]
> `build/` 与 `build2019/` 的 CMake 缓存分别锁定了 MotionBuilder 2024 与 2019 的 SDK 路径。切分支后必须使用配套目录，混用会把错误版本的 SDK 头文件与 `fbsdk.lib` 链入插件。

复制后请同步更新上方产物清单中的源码提交与编译日期，再提交。
