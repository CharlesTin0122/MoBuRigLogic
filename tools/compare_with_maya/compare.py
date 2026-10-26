# -*- coding: utf-8 -*-
"""对比 Maya 与 MotionBuilder 两侧的 MetaHuman 身体修形采样结果。

用法（普通 Python 3 即可，不需要 Maya/MoBu）：

    python compare.py                     # 读 out/maya.json 与 out/mobu.json
    python compare.py --pos-mm 0.1 --rot-deg 0.1
    python compare.py --no-fail           # 只出报告，不因超阈值返回非 0

判定
----
* ``position``: 修形关节世界位置的动态偏差（扣除静置基准），单位 mm；
* ``rotation``: 修形关节本地旋转输出增量的偏差，单位度；
两者任一超过阈值即返回退出码 1，可用于回归门禁。
默认阈值（0.8 mm / 0.6°）是在当前基线（0.668 mm / 0.478°）上留了余量，
用来拦“变差”而不是拦“有差异”；需要更严的指标时用 --pos-mm / --rot-deg 收紧。

输出
----
* ``out/report.md``    Markdown 报告
* ``out/summary.json`` 机器可读摘要
"""

import argparse
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

GROUPS = [
    ("legs", "腿部(髋/膝/踝)", ("thigh", "calf", "ankle")),
    ("torso", "躯干/锁骨", ("spine", "clavicle")),
    ("arm", "上臂/前臂", ("upperarm", "lowerarm")),
    ("wrist", "手腕", ("wrist",)),
    ("fingers", "手指", ("index", "middle", "ring", "pinky", "thumb")),
]


# ------------------------------------------------------------------ 数学工具
def decompose(matrix):
    """4x4 行主序矩阵 -> (旋转矩阵, 各轴缩放, 平移)。"""
    cols = [(matrix[0], matrix[4], matrix[8]),
            (matrix[1], matrix[5], matrix[9]),
            (matrix[2], matrix[6], matrix[10])]
    scale = [math.sqrt(c[0] ** 2 + c[1] ** 2 + c[2] ** 2) for c in cols]
    rot = [[cols[j][i] / (scale[j] or 1.0) for j in range(3)] for i in range(3)]
    return rot, tuple(scale), (matrix[12], matrix[13], matrix[14])


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rotation_angle_deg(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def length(v):
    return math.sqrt(sum(x * x for x in v))


# ------------------------------------------------------------------ 数据加载
def load_side(path):
    with io.open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def group_of(joint):
    for key, _label, prefixes in GROUPS:
        if joint.startswith(prefixes):
            return key
    return "other"


# ------------------------------------------------------------------ 主流程
def analyse(maya, mobu):
    joints = [j for j in maya["outputs"] if j in mobu["outputs"]]
    drivers = [d for d in maya["drivers"] if d in mobu["drivers"]]
    maya_poses = dict((p["label"], p) for p in maya["poses"])
    mobu_poses = dict((p["label"], p) for p in mobu["poses"])
    labels = [label for label in maya_poses if label in mobu_poses]
    if not labels:
        raise RuntimeError("两侧没有共同的姿势标签，无法对比")

    rest_maya = maya_poses["P00_rest"]["mats"]
    rest_mobu = mobu_poses["P00_rest"]["mats"]
    attrs_maya_rest = maya_poses["P00_rest"]["attrs"]
    attrs_mobu_rest = mobu_poses["P00_rest"]["attrs"]

    # 1. 驱动骨骼姿态一致性（driver 纯 FK，必须逐位一致）
    driver_pos = 0.0
    for label in labels:
        m, o = maya_poses[label]["mats"], mobu_poses[label]["mats"]
        for joint in drivers:
            if joint in m and joint in o:
                driver_pos = max(driver_pos, length(sub(decompose(m[joint])[2], decompose(o[joint])[2])))

    # 2. 输出值层面：本地 TRS 增量差
    static_rot = {}
    for joint in joints:
        static_rot[joint] = max(abs(attrs_maya_rest[joint][3 + i] - attrs_mobu_rest[joint][3 + i])
                                for i in range(3))
    per_pose = {}
    worst_rot = {}
    for label in labels:
        bm, bo = maya_poses[label]["attrs"], mobu_poses[label]["attrs"]
        dt = dr = ds = 0.0
        for joint in joints:
            for i in range(3):
                dt = max(dt, abs((bm[joint][i] - attrs_maya_rest[joint][i])
                                 - (bo[joint][i] - attrs_mobu_rest[joint][i])))
                ds = max(ds, abs((bm[joint][6 + i] - attrs_maya_rest[joint][6 + i])
                                 - (bo[joint][6 + i] - attrs_mobu_rest[joint][6 + i])))
                value = abs((bm[joint][3 + i] - attrs_maya_rest[joint][3 + i])
                            - (bo[joint][3 + i] - attrs_mobu_rest[joint][3 + i]))
                dr = max(dr, value)
                if value > worst_rot.get(joint, (0.0, ""))[0]:
                    worst_rot[joint] = (value, label)
        per_pose[label] = {"translation": dt, "rotation": dr, "scale": ds}

    # 3. 世界位置层面
    static_pos = {}
    for joint in joints:
        static_pos[joint] = length(sub(decompose(rest_maya[joint])[2], decompose(rest_mobu[joint])[2]))
    pose_pos = {}
    for label in labels:
        m, o = maya_poses[label]["mats"], mobu_poses[label]["mats"]
        dynamic = 0.0
        absolute = 0.0
        worst_joint = ""
        for joint in joints:
            dyn = length(sub(sub(decompose(m[joint])[2], decompose(rest_maya[joint])[2]),
                             sub(decompose(o[joint])[2], decompose(rest_mobu[joint])[2])))
            if dyn > dynamic:
                dynamic, worst_joint = dyn, joint
            absolute = max(absolute, length(sub(decompose(m[joint])[2], decompose(o[joint])[2])))
        pose_pos[label] = {"dynamic": dynamic, "absolute": absolute, "joint": worst_joint}

    # 4. 按部位汇总
    groups = {}
    for key, label, _prefixes in GROUPS + [("other", "其他", ())]:
        members = [j for j in joints if group_of(j) == key]
        if not members:
            continue
        dynamic = 0.0
        worst = ""
        worst_pose = ""
        for pose_label in labels:
            m, o = maya_poses[pose_label]["mats"], mobu_poses[pose_label]["mats"]
            for joint in members:
                dyn = length(sub(sub(decompose(m[joint])[2], decompose(rest_maya[joint])[2]),
                                 sub(decompose(o[joint])[2], decompose(rest_mobu[joint])[2])))
                if dyn > dynamic:
                    dynamic, worst, worst_pose = dyn, joint, pose_label
        groups[key] = {
            "label": label,
            "count": len(members),
            "static": max(static_pos[j] for j in members),
            "dynamic": dynamic,
            "joint": worst,
            "pose": worst_pose,
        }

    return {
        "joints": joints,
        "drivers": drivers,
        "labels": labels,
        "driverPos": driver_pos,
        "perPose": per_pose,
        "staticRot": static_rot,
        "worstRot": worst_rot,
        "staticPos": static_pos,
        "posePos": pose_pos,
        "groups": groups,
        "mayaApp": maya["app"],
        "mobuApp": mobu["app"],
    }


def build_markdown(result, thresholds):
    pos_mm, rot_deg = thresholds
    lines = []
    add = lines.append

    add("# MetaHuman 身体修形对比报告（Maya vs MotionBuilder）")
    add("")
    add("- Maya 侧：%s" % result["mayaApp"].get("app", "?"))
    add("  - 场景：`%s`" % result["mayaApp"].get("scene", "?"))
    add("  - 修形节点：`%s`" % result["mayaApp"].get("riglogicNode", "?"))
    add("- MotionBuilder 侧：%s" % result["mobuApp"].get("app", "?"))
    add("  - 约束：`%s`" % result["mobuApp"].get("constraint", "?"))
    add("  - DNA：`%s`，LOD %s" % (result["mobuApp"].get("dna", "?"), result["mobuApp"].get("lod", "?")))
    add("- 驱动关节 %d 根，修形输出关节 %d 根，姿势 %d 个"
        % (len(result["drivers"]), len(result["joints"]), len(result["labels"])))
    add("")
    add("## 1. 驱动骨骼姿态一致性")
    add("")
    add("两侧施加相同欧拉增量后，驱动骨骼世界位置最大偏差 **%.5f mm**。" % (result["driverPos"] * 10))
    if result["driverPos"] * 10 > 0.01:
        add("")
        add("> 偏差偏大，请检查两侧驱动骨骼的静置姿态或父级层级是否一致。")
    add("")
    add("## 2. 修形输出值（骨骼本地 TRS 增量）")
    add("")
    add("| 姿势 | Δ平移 | Δ旋转 | Δ缩放 |")
    add("|---|---|---|---|")
    for label in result["labels"]:
        row = result["perPose"][label]
        add("| %s | %.5f | %.3f° | %.5f |" % (label, row["translation"], row["rotation"], row["scale"]))
    add("")
    static_joints = sorted(((v, j) for j, v in result["staticRot"].items() if v > rot_deg), reverse=True)
    add("静置姿态下的本地旋转差超过阈值的关节：%d 根" % len(static_joints))
    if static_joints:
        add("")
        add("| 关节 | 静置旋转差 | 最大动态旋转差 | @ 姿势 |")
        add("|---|---|---|---|")
        for value, joint in static_joints:
            worst, label = result["worstRot"].get(joint, (0.0, ""))
            add("| %s | %.3f° | %.3f° | %s |" % (joint, value, worst, label))
    add("")
    add("## 3. 修形关节世界位置")
    add("")
    add("| 姿势 | 动态偏差（扣除静置基准） | 绝对偏差 |")
    add("|---|---|---|")
    for label in result["labels"]:
        row = result["posePos"][label]
        add("| %s | %.3f mm (%s) | %.3f mm |"
            % (label, row["dynamic"] * 10, row["joint"], row["absolute"] * 10))
    add("")
    add("## 4. 按部位汇总")
    add("")
    add("| 部位 | 关节数 | 静置偏差 | 动态偏差 | 最大偏差关节 | 姿势 |")
    add("|---|---|---|---|---|---|")
    for key, _label, _prefixes in GROUPS + [("other", "其他", ())]:
        info = result["groups"].get(key)
        if not info:
            continue
        add("| %s | %d | %.3f mm | %.3f mm | %s | %s |"
            % (info["label"], info["count"], info["static"] * 10, info["dynamic"] * 10,
               info["joint"], info["pose"]))
    add("")

    exact = sum(1 for joint in result["joints"] if result["staticPos"][joint] * 10 <= pos_mm
                and max(result["posePos"][label]["dynamic"] for label in result["labels"]) * 10 <= pos_mm)
    worst_pos = max(result["posePos"][label]["dynamic"] for label in result["labels"])
    worst_rot = max(result["perPose"][label]["rotation"] for label in result["labels"])
    worst_pos_label = max(result["labels"], key=lambda l: result["posePos"][l]["dynamic"])
    add("## 5. 判定")
    add("")
    add("| 指标 | 阈值 | 实测 | 结果 |")
    add("|---|---|---|---|")
    add("| 修形关节动态位置偏差 | %.3f mm | %.3f mm | %s |"
        % (pos_mm, worst_pos * 10, "PASS" if worst_pos * 10 <= pos_mm else "FAIL"))
    add("| 修形旋转输出偏差 | %.3f° | %.3f° | %s |"
        % (rot_deg, worst_rot, "PASS" if worst_rot <= rot_deg else "FAIL"))
    add("")
    add("最大动态位置偏差出现在 `%s`（%s）。静置与全姿势均一致（≤%.2f mm）的修形关节：%d / %d。"
        % (result["posePos"][worst_pos_label]["joint"], worst_pos_label, pos_mm, exact,
           len(result["joints"])))
    return "\n".join(lines), worst_pos * 10, worst_rot


def main(argv=None):
    parser = argparse.ArgumentParser(description="对比 Maya / MotionBuilder 的 MetaHuman 身体修形采样结果")
    parser.add_argument("--out-dir", default=os.path.join(HERE, "out"), help="采样数据与报告目录")
    parser.add_argument("--maya", default=None, help="Maya 采样 JSON（默认 <out-dir>/maya.json）")
    parser.add_argument("--mobu", default=None, help="MoBu 采样 JSON（默认 <out-dir>/mobu.json）")
    parser.add_argument("--pos-mm", type=float, default=0.8,
                        help="修形关节动态位置偏差阈值（mm），基线 0.668，默认留约 20%% 余量")
    parser.add_argument("--rot-deg", type=float, default=0.6,
                        help="修形旋转输出偏差阈值（度），基线 0.478，默认留约 25%% 余量")
    parser.add_argument("--no-fail", action="store_true", help="超阈值也返回 0")
    args = parser.parse_args(argv)

    maya_path = args.maya or os.path.join(args.out_dir, "maya.json")
    mobu_path = args.mobu or os.path.join(args.out_dir, "mobu.json")
    for path in (maya_path, mobu_path):
        if not os.path.isfile(path):
            raise SystemExit("缺少采样文件：%s" % path)

    result = analyse(load_side(maya_path), load_side(mobu_path))
    report, worst_pos_mm, worst_rot_deg = build_markdown(result, (args.pos_mm, args.rot_deg))

    report_path = os.path.join(args.out_dir, "report.md")
    with io.open(report_path, "w", encoding="utf-8") as f:
        f.write(report)
    summary = {
        "drivers": len(result["drivers"]),
        "outputs": len(result["joints"]),
        "poses": len(result["labels"]),
        "driverPosMm": result["driverPos"] * 10,
        "maxDynamicPosMm": worst_pos_mm,
        "maxRotDeg": worst_rot_deg,
        "thresholds": {"posMm": args.pos_mm, "rotDeg": args.rot_deg},
        "pass": worst_pos_mm <= args.pos_mm and worst_rot_deg <= args.rot_deg,
    }
    with io.open(os.path.join(args.out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("报告：%s" % report_path)
    print("驱动姿势一致：%.5f mm | 修形动态位置最大偏差：%.3f mm | 旋转输出最大偏差：%.3f°"
          % (summary["driverPosMm"], worst_pos_mm, worst_rot_deg))
    print("判定：%s" % ("PASS" if summary["pass"] else "FAIL"))
    if not summary["pass"] and not args.no_fail:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
