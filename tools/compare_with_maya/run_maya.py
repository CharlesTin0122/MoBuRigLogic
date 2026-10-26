# -*- coding: utf-8 -*-
"""Maya 侧采样入口。

在 Maya 的脚本编辑器或 MayaMCP 中执行：

    import runpy
    runpy.run_path(r"D:\\Code\\MoBuRigLogic\\tools\\compare_with_maya\\run_maya.py",
                   run_name="__main__")

输出：
    out/names.json  驱动/修形关节名单（MoBu 侧复用，保证对比同一批骨骼）
    out/maya.json   35 个姿势的修形采样数据

注意：脚本会临时修改驱动骨骼的 rotate 属性，采样结束后自动还原；
      但 Maya 场景会被标记为“已修改”，请勿在采样过程中保存。
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# 同一应用进程内重复运行时，避免命中旧模块/旧字节码（改过 capture.py 后尤其需要）
sys.modules.pop("capture", None)
import capture  # noqa: E402


def main():
    out_dir = os.environ.get("MH_CMP_OUT_DIR") or os.path.join(HERE, "out")
    poses_path = os.path.join(HERE, "poses.json")
    names_path = os.path.join(out_dir, "names.json")
    out_path = os.path.join(out_dir, "maya.json")
    message = capture.run_side("maya", poses_path, out_path, names_path)
    print(message)
    return message


if __name__ == "__main__":
    main()
