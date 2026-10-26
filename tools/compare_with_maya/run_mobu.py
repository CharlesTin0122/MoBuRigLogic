# -*- coding: utf-8 -*-
"""MotionBuilder 侧采样入口（MotionBuilder 2019 / Python 2.7）。

先跑 run_maya.py 生成 out/names.json，再在 MotionBuilder 中执行：

    import runpy
    runpy.run_path(r"D:\\Code\\MoBuRigLogic\\tools\\compare_with_maya\\run_mobu.py",
                   run_name="__main__")

输出：
    out/mobu.json   35 个姿势的修形采样数据

注意：脚本会临时修改驱动骨骼的 Lcl Rotation，采样结束后自动还原。
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# 同一应用进程内重复运行时，避免命中旧模块/旧字节码（改过 capture.py 后尤其需要）
sys.modules.pop("capture", None)
import capture


def main():
    out_dir = os.environ.get("MH_CMP_OUT_DIR") or os.path.join(HERE, "out")
    poses_path = os.path.join(HERE, "poses.json")
    names_path = os.path.join(out_dir, "names.json")
    out_path = os.path.join(out_dir, "mobu.json")
    if not os.path.isfile(names_path):
        raise RuntimeError("缺少 %s，请先在 Maya 侧运行 run_maya.py" % names_path)
    message = capture.run_side("mobu", poses_path, out_path, names_path)
    print(message)
    return message


if __name__ == "__main__":
    main()
