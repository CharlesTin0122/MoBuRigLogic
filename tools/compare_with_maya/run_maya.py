# -*- coding: utf-8 -*-
"""Maya 侧采样入口。

在 Maya 的脚本编辑器或 MayaMCP 中执行（默认 profile=body）：

    import runpy
    runpy.run_path(r"D:\\Code\\MoBuRigLogic\\tools\\compare_with_maya\\run_maya.py",
                   run_name="__main__")

切换对比对象（body / face / face_sweep）：

    import os
    os.environ["MH_CMP_PROFILE"] = "face"
    import runpy
    runpy.run_path(r"...\\run_maya.py", run_name="__main__")

输出（out/ 目录）：
    names_<profile>.json   驱动/输出关节、BlendShape 通道、控制器名单（MoBu 侧复用）
    maya_<profile>.json    采样数据

注意：脚本会临时修改骨骼/控制器属性（面部还会临时解锁通道锁），结束时自动还原；
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

PROFILE = os.environ.get("MH_CMP_PROFILE", "body")


def main():
    out_dir = os.environ.get("MH_CMP_OUT_DIR") or os.path.join(HERE, "out")
    poses_path = os.path.join(HERE, capture.PROFILES[PROFILE]["poses"])
    names_path = os.path.join(out_dir, "names_%s.json" % PROFILE)
    out_path = os.path.join(out_dir, "maya_%s.json" % PROFILE)
    message = capture.run_side(PROFILE, "maya", poses_path, out_path, names_path)
    print(message)
    return message


if __name__ == "__main__":
    main()
