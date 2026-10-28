# -*- coding: utf-8 -*-
"""对比 Maya 与 MotionBuilder 两侧的 MetaHuman 修形采样结果。

支持三个 profile（与 capture.py / run_*.py 一一对应）：

* ``body``       身体修形（修形关节）
* ``face``       面部修形（FACIAL_* 关节 + BlendShape + 网格）
* ``face_sweep`` 面部全控制器扫描

用法（普通 Python 3 即可，不需要 Maya/MoBu）：

    python tools/compare_with_maya/compare.py --profile body
    python tools/compare_with_maya/compare.py --profile face
    python tools/compare_with_maya/compare.py --profile face_sweep

判定
----
超过阈值即返回退出码 1，可用于回归门禁；默认阈值按 profile 给（见 DEFAULT_THRESHOLDS），
是在当前基线上留余量、用来拦“变差”而不是拦“有差异”。
"""

import argparse
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

DEFAULT_THRESHOLDS = {
    # profile: (位置 mm, 旋转 deg, BlendShape 0..100 制)
    "body": (0.8, 0.6, None),
    "face": (0.01, 0.01, 0.01),
    "face_sweep": (0.01, 0.01, 0.01),
}

GROUPS = [
    ("legs", "腿部(髋/膝/踝)", ("thigh", "calf", "ankle")),
    ("torso", "躯干/锁骨", ("spine", "clavicle")),
    ("arm", "上臂/前臂", ("upperarm", "lowerarm")),
    ("wrist", "手腕", ("wrist",)),
    ("fingers", "手指", ("index", "middle", "ring", "pinky", "thumb")),
]


# ------------------------------------------------------------------ 数学工具
def decompose(matrix):
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


def sci(v):
    if v == 0:
        return "0"
    if abs(v) < 1e-3:
        return "%.2e" % v
    return "%.4f" % v


def load_side(path):
    with io.open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def group_of(joint):
    for key, _label, prefixes in GROUPS:
        if joint.startswith(prefixes):
            return key
    return "other"


# ------------------------------------------------------------------ 身体修形
def compare_body(maya, mobu, thresholds):
    joints = [j for j in maya["outputs"] if j in set(mobu["outputs"])]
    drivers = [d for d in maya["drivers"] if d in set(mobu["drivers"])]
    mp = dict((p["label"], p) for p in maya["poses"])
    op = dict((p["label"], p) for p in mobu["poses"])
    labels = [l for l in mp if l in op]
    if not labels:
        raise RuntimeError("两侧没有共同的姿势标签")
    rest_m, rest_o = mp["P00_rest"]["mats"], op["P00_rest"]["mats"]
    am_rest, ao_rest = mp["P00_rest"]["attrs"], op["P00_rest"]["attrs"]
    pos_mm, rot_deg, _bs = thresholds

    driver_pos = 0.0
    for label in labels:
        m, o = mp[label]["mats"], op[label]["mats"]
        for joint in drivers:
            if joint in m and joint in o:
                driver_pos = max(driver_pos,
                                 length(sub(decompose(m[joint])[2], decompose(o[joint])[2])))

    per_pose = {}
    worst_rot = {}
    static_rot = {}
    for joint in joints:
        static_rot[joint] = max(abs(am_rest[joint][3 + i] - ao_rest[joint][3 + i]) for i in range(3))
    for label in labels:
        bm, bo = mp[label]["attrs"], op[label]["attrs"]
        dt = dr = ds = 0.0
        for joint in joints:
            for i in range(3):
                dt = max(dt, abs((bm[joint][i] - am_rest[joint][i]) - (bo[joint][i] - ao_rest[joint][i])))
                ds = max(ds, abs((bm[joint][6 + i] - am_rest[joint][6 + i]) - (bo[joint][6 + i] - ao_rest[joint][6 + i])))
                value = abs((bm[joint][3 + i] - am_rest[joint][3 + i]) - (bo[joint][3 + i] - ao_rest[joint][3 + i]))
                dr = max(dr, value)
                if value > worst_rot.get(joint, (0.0, ""))[0]:
                    worst_rot[joint] = (value, label)
        per_pose[label] = {"translation": dt, "rotation": dr, "scale": ds}

    static_pos = dict((j, length(sub(decompose(rest_m[j])[2], decompose(rest_o[j])[2]))) for j in joints)
    pose_pos = {}
    for label in labels:
        m, o = mp[label]["mats"], op[label]["mats"]
        dynamic = absolute = 0.0
        worst_joint = ""
        for joint in joints:
            dyn = length(sub(sub(decompose(m[joint])[2], decompose(rest_m[joint])[2]),
                             sub(decompose(o[joint])[2], decompose(rest_o[joint])[2])))
            if dyn > dynamic:
                dynamic, worst_joint = dyn, joint
            absolute = max(absolute, length(sub(decompose(m[joint])[2], decompose(o[joint])[2])))
        pose_pos[label] = {"dynamic": dynamic, "absolute": absolute, "joint": worst_joint}

    groups = {}
    for key, label, _prefixes in GROUPS + [("other", "其他", ())]:
        members = [j for j in joints if group_of(j) == key]
        if not members:
            continue
        dynamic = 0.0
        worst = worst_pose = ""
        for pose_label in labels:
            m, o = mp[pose_label]["mats"], op[pose_label]["mats"]
            for joint in members:
                dyn = length(sub(sub(decompose(m[joint])[2], decompose(rest_m[joint])[2]),
                                 sub(decompose(o[joint])[2], decompose(rest_o[joint])[2])))
                if dyn > dynamic:
                    dynamic, worst, worst_pose = dyn, joint, pose_label
        groups[key] = {"label": label, "count": len(members),
                       "static": max(static_pos[j] for j in members),
                       "dynamic": dynamic, "joint": worst, "pose": worst_pose}

    worst_pos = max(pose_pos[l]["dynamic"] for l in labels)
    worst_rot_pose = max(per_pose[l]["rotation"] for l in labels)
    worst_label = max(labels, key=lambda l: pose_pos[l]["dynamic"])

    L = []
    A = L.append
    A("# MetaHuman 身体修形对比（Maya vs MotionBuilder）")
    A("")
    A("- Maya：%s，场景 `%s`，修形节点 `%s`"
      % (maya["app"].get("app", ""), maya["app"].get("scene", ""), maya["app"].get("node", "")))
    A("- MoBu：%s，约束 `%s`（DNA `%s`，LOD %s）"
      % (mobu["app"].get("app", ""), mobu["app"].get("constraint", ""),
         os.path.basename(mobu["app"].get("dna", "")), mobu["app"].get("lod", "?")))
    A("- 驱动关节 %d 根，修形输出关节 %d 根，姿势 %d 个" % (len(drivers), len(joints), len(labels)))
    A("")
    A("## 1. 驱动姿态一致性")
    A("")
    A("两侧施加相同欧拉增量后，驱动骨骼世界位置最大偏差 **%.5f mm**。" % (driver_pos * 10))
    A("")
    A("## 2. 修形输出值（本地 TRS 增量）")
    A("")
    A("| 姿势 | Δ平移 | Δ旋转 | Δ缩放 |")
    A("|---|---|---|---|")
    for label in labels:
        row = per_pose[label]
        A("| %s | %s | %s° | %s |"
          % (label, sci(row["translation"]), sci(row["rotation"]), sci(row["scale"])))
    static_bad = sorted(((v, j) for j, v in static_rot.items() if v > rot_deg), reverse=True)
    A("")
    A("静置姿态旋转差超过阈值的关节：%d 根" % len(static_bad))
    if static_bad:
        A("")
        A("| 关节 | 静置旋转差 | 最大动态旋转差 | 姿势 |")
        A("|---|---|---|---|")
        for value, joint in static_bad:
            worst, label = worst_rot.get(joint, (0.0, ""))
            A("| %s | %.3f° | %.3f° | %s |" % (joint, value, worst, label))
    A("")
    A("## 3. 修形关节世界位置")
    A("")
    A("| 姿势 | 动态偏差 | 绝对偏差 |")
    A("|---|---|---|")
    for label in labels:
        row = pose_pos[label]
        A("| %s | %.3f mm (%s) | %.3f mm |" % (label, row["dynamic"] * 10, row["joint"], row["absolute"] * 10))
    A("")
    A("## 4. 按部位汇总")
    A("")
    A("| 部位 | 关节数 | 静置偏差 | 动态偏差 | 最大偏差关节 | 姿势 |")
    A("|---|---|---|---|---|---|")
    for key, *_rest in GROUPS + [("other", "其他", ())]:
        info = groups.get(key)
        if not info:
            continue
        A("| %s | %d | %.3f mm | %.3f mm | %s | %s |"
          % (info["label"], info["count"], info["static"] * 10, info["dynamic"] * 10,
             info["joint"], info["pose"]))
    exact = sum(1 for joint in joints
                if static_pos[joint] * 10 <= pos_mm
                and max(pose_pos[l]["dynamic"] for l in labels) * 10 <= pos_mm)
    A("")
    A("## 5. 判定")
    A("")
    A("| 指标 | 阈值 | 实测 | 结果 |")
    A("|---|---|---|---|")
    A("| 修形关节动态位置偏差 | %.3f mm | %.3f mm | %s |"
      % (pos_mm, worst_pos * 10, "PASS" if worst_pos * 10 <= pos_mm else "FAIL"))
    A("| 修形旋转输出偏差 | %.3f° | %.3f° | %s |"
      % (rot_deg, worst_rot_pose, "PASS" if worst_rot_pose <= rot_deg else "FAIL"))
    A("")
    A("最大动态位置偏差在 `%s`（%s）。静置与全姿势都一致的修形关节：%d / %d。"
      % (pose_pos[worst_label]["joint"], worst_label, exact, len(joints)))

    summary = {"outputs": len(joints), "drivers": len(drivers), "poses": len(labels),
               "driverPosMm": driver_pos * 10, "maxDynamicPosMm": worst_pos * 10,
               "maxRotDeg": worst_rot_pose,
               "pass": worst_pos * 10 <= pos_mm and worst_rot_pose <= rot_deg}
    return "\n".join(L), summary


# ------------------------------------------------------------------ 面部修形
def compare_face(maya, mobu, thresholds, sweep):
    joints = [j for j in maya["joints"] if j in set(mobu["joints"])]
    mobu_channels = set(tuple(c) for c in mobu["channels"])
    channels = [tuple(c) for c in maya["channels"] if tuple(c) in mobu_channels]
    meshes = [m for m in maya.get("meshes", []) if m in set(mobu.get("meshes", []))]
    mp = dict((p["label"], p) for p in maya["poses"])
    op = dict((p["label"], p) for p in mobu["poses"])
    labels = [l for l in mp if l in op]
    if not labels:
        raise RuntimeError("两侧没有共同的姿势标签")
    rest_m, rest_o = mp[labels[0]], op[labels[0]]
    pos_mm, rot_deg, bs_eps = thresholds

    rows = []
    for label in labels:
        a, b = mp[label], op[label]
        d_rot = d_tra = d_pos = d_bs = 0.0
        resp_rot = resp_tra = resp_pos = resp_bs = resp_mesh = 0.0
        worst_joint = worst_bs = ""
        for j in joints:
            rm, ro = a["R"][j], b["R"][j]
            tm, to = a["T"][j], b["T"][j]
            r0m, r0o = rest_m["R"][j], rest_o["R"][j]
            t0m, t0o = rest_m["T"][j], rest_o["T"][j]
            dr = max(abs((rm[i] - r0m[i]) - (ro[i] - r0o[i])) for i in range(3))
            dt = max(abs((tm[i] - t0m[i]) - (to[i] - t0o[i])) for i in range(3))
            dp = length(sub(a["p"][j], b["p"][j]))
            if dr + dp * 10 > d_rot + d_pos * 10:
                worst_joint = j
            d_rot = max(d_rot, dr)
            d_tra = max(d_tra, dt)
            d_pos = max(d_pos, dp)
            resp_rot = max(resp_rot, max(abs(rm[i] - r0m[i]) for i in range(3)))
            resp_tra = max(resp_tra, max(abs(tm[i] - t0m[i]) for i in range(3)))
            resp_pos = max(resp_pos, length(sub(a["p"][j], rest_m["p"][j])))
        for mesh, ch in channels:
            key = "%s::%s" % (mesh, ch)
            d = abs(a["bs"].get(key, 0.0) * 100.0 - b["bs"].get(key, 0.0))
            if d > d_bs:
                d_bs, worst_bs = d, key
            resp_bs = max(resp_bs, abs(a["bs"].get(key, 0.0) - rest_m["bs"].get(key, 0.0)) * 100.0)
        for m in meshes:
            if m in a.get("meshP", {}) and m in rest_m.get("meshP", {}):
                resp_mesh = max(resp_mesh, length(sub(a["meshP"][m], rest_m["meshP"][m])))
        rows.append({"label": label, "d_rot": d_rot, "d_tra": d_tra, "d_pos": d_pos, "d_bs": d_bs,
                     "resp_rot": resp_rot, "resp_tra": resp_tra, "resp_pos": resp_pos,
                     "resp_bs": resp_bs, "resp_mesh": resp_mesh,
                     "worst_joint": worst_joint, "worst_bs": worst_bs})
    by_label = dict((r["label"], r) for r in rows)

    worst_rot = max(rows, key=lambda r: r["d_rot"])
    worst_pos = max(rows, key=lambda r: r["d_pos"])
    worst_bs = max(rows, key=lambda r: r["d_bs"])
    worst_mesh = max(rows, key=lambda r: r.get("resp_mesh", 0.0))
    ok_pos = worst_pos["d_pos"] * 10 <= pos_mm
    ok_rot = worst_rot["d_rot"] <= rot_deg
    ok_bs = bs_eps is None or worst_bs["d_bs"] <= bs_eps
    resp_any = any(r["resp_rot"] > 1e-4 or r["resp_tra"] > 1e-5 or r["resp_bs"] > 1e-4 for r in rows[1:])

    L = []
    A = L.append
    A("# MetaHuman 面部修形对比（Maya vs MotionBuilder）%s" % ("：全控制器扫描" if sweep else ""))
    A("")
    A("- Maya：%s，场景 `%s`，修形节点 `%s`"
      % (maya["app"].get("app", ""), maya["app"].get("scene", ""), maya["app"].get("node", "")))
    A("- MoBu：%s，约束 `%s`（Input Mode=%s，DNA `%s`，LOD %s）"
      % (mobu["app"].get("app", ""), mobu["app"].get("constraint", ""),
         mobu["app"].get("inputMode", "?"), os.path.basename(mobu["app"].get("dna", "")),
         mobu["app"].get("lod", "?")))
    A("- 输入：FaceBoard 控制器 `CTRL_*` 的 translate（0..1），两侧施加相同数值")
    A("- 采样：FACIAL_* 关节 %d 根、BlendShape 通道 %d 个、DNA 网格 %d 个、姿势 %d 个"
      % (len(joints), len(channels), len(meshes), len(labels)))
    A("")
    A("## 1. 响应幅度（确认两侧都被驱动）")
    A("")
    A("| 姿势 | 关节最大旋转 | 关节最大平移 | 关节最大位移 | BlendShape 最大变化 |")
    A("|---|---|---|---|---|")
    for r in rows:
        A("| %s | %.2f° | %.2f mm | %.2f mm | %.2f |"
          % (r["label"], r["resp_rot"], r["resp_tra"] * 10, r["resp_pos"] * 10, r["resp_bs"]))
    if not resp_any:
        A("")
        A("> 除静置外没有任何响应，请检查控制器绑定/约束是否生效。")
    dead = [r["label"] for r in rows[1:]
            if r["resp_rot"] <= 1e-4 and r["resp_tra"] <= 1e-5 and r["resp_bs"] <= 1e-4]
    if dead:
        A("")
        A("> 两侧都没有可见输出的姿势（%d 个）：%s" % (len(dead), ", ".join(dead)))
    A("")
    A("## 2. FACIAL_* 修形关节差异")
    A("")
    A("| 姿势 | 世界位置差 | 本地旋转增量差 | 本地平移增量差 |")
    A("|---|---|---|---|")
    for r in rows:
        A("| %s | %s mm | %s° | %s mm |" % (r["label"], sci(r["d_pos"] * 10), sci(r["d_rot"]), sci(r["d_tra"] * 10)))
    A("")
    A("## 3. BlendShape 权重差异（统一换算成 0..100）")
    A("")
    A("| 姿势 | 最大差 | 最大差通道 |")
    A("|---|---|---|")
    for r in rows:
        A("| %s | %s | %s |" % (r["label"], sci(r["d_bs"]), r["worst_bs"] or "-"))
    if meshes:
        A("")
        A("## 4. DNA 网格位姿")
        A("")
        A("| 姿势 | 网格最大位移 |")
        A("|---|---|")
        for r in rows:
            A("| %s | %.3f mm |" % (r["label"], r["resp_mesh"] * 10))
        A("")
        A("> 网格的顶点变形来自骨骼 + BlendShape，位姿（transform）不变属正常。")
    A("")
    A("## 5. 汇总")
    A("")
    A("| 指标 | 阈值 | 实测 | 结果 |")
    A("|---|---|---|---|")
    A("| 关节世界位置差 | %.3f mm | %s mm | %s |"
      % (pos_mm, sci(worst_pos["d_pos"] * 10), "PASS" if ok_pos else "FAIL"))
    A("| 关节本地旋转增量差 | %.3f° | %s° | %s |"
      % (rot_deg, sci(worst_rot["d_rot"]), "PASS" if ok_rot else "FAIL"))
    A("| BlendShape 权重差 | %s | %s | %s |"
      % ("-" if bs_eps is None else "%.3f" % bs_eps, sci(worst_bs["d_bs"]), "PASS" if ok_bs else "FAIL"))
    A("")
    A("- 关节位置最大差出现在 `%s`（%s）" % (worst_pos["worst_joint"], worst_pos["label"]))
    A("- 旋转最大差出现在 %s" % worst_rot["label"])
    A("- BlendShape 最大差出现在 `%s`（%s）" % (worst_bs["worst_bs"] or "两侧全部为 0", worst_bs["label"]))

    if sweep:
        active = [r for r in rows[1:] if r["label"] not in dead]
        A("")
        A("## 6. 控制器覆盖")
        A("")
        A("- 有实际响应：%d / %d 个控制器" % (len(active), len(rows) - 1))
        A("")
        A("| 控制器 | 关节最大旋转变化 | 关节最大平移变化 | BlendShape 最大变化 |")
        A("|---|---|---|---|")
        for r in sorted(active, key=lambda x: -(x["resp_rot"] + x["resp_tra"] * 10 + x["resp_bs"]))[:25]:
            A("| %s | %.2f° | %.2f mm | %.2f |"
              % (r["label"], r["resp_rot"], r["resp_tra"] * 10, r["resp_bs"]))
        if dead:
            A("")
            A("无可见输出的控制器（两侧一致）：")
            A("")
            for label in dead:
                A("- `%s`" % label)

    summary = {"outputs": len(joints), "channels": len(channels), "meshes": len(meshes),
               "poses": len(labels),
               "maxPosMm": worst_pos["d_pos"] * 10,
               "maxRotDeg": worst_rot["d_rot"],
               "maxBs": worst_bs["d_bs"],
               "pass": ok_pos and ok_rot and ok_bs}
    return "\n".join(L), summary


# ------------------------------------------------------------------ 入口
def main(argv=None):
    parser = argparse.ArgumentParser(description="对比 Maya / MotionBuilder 的 MetaHuman 修形采样")
    parser.add_argument("--profile", default="body", choices=sorted(DEFAULT_THRESHOLDS.keys()),
                        help="对比对象：body / face / face_sweep")
    parser.add_argument("--out-dir", default=os.path.join(HERE, "out"))
    parser.add_argument("--maya", default=None)
    parser.add_argument("--mobu", default=None)
    parser.add_argument("--pos-mm", type=float, default=None)
    parser.add_argument("--rot-deg", type=float, default=None)
    parser.add_argument("--bs", type=float, default=None, help="BlendShape 阈值（0..100 制，仅面部）")
    parser.add_argument("--no-fail", action="store_true")
    args = parser.parse_args(argv)

    d_pos, d_rot, d_bs = DEFAULT_THRESHOLDS[args.profile]
    pos_mm = d_pos if args.pos_mm is None else args.pos_mm
    rot_deg = d_rot if args.rot_deg is None else args.rot_deg
    bs_eps = d_bs if args.bs is None else args.bs

    maya_path = args.maya or os.path.join(args.out_dir, "maya_%s.json" % args.profile)
    mobu_path = args.mobu or os.path.join(args.out_dir, "mobu_%s.json" % args.profile)
    for path in (maya_path, mobu_path):
        if not os.path.isfile(path):
            raise SystemExit("缺少采样文件：%s" % path)

    maya, mobu = load_side(maya_path), load_side(mobu_path)
    if args.profile == "body":
        report, summary = compare_body(maya, mobu, (pos_mm, rot_deg, bs_eps))
    else:
        report, summary = compare_face(maya, mobu, (pos_mm, rot_deg, bs_eps),
                                       sweep=(args.profile == "face_sweep"))

    report_path = os.path.join(args.out_dir, "report_%s.md" % args.profile)
    with io.open(report_path, "w", encoding="utf-8") as f:
        f.write(report)
    summary["profile"] = args.profile
    summary["thresholds"] = {"posMm": pos_mm, "rotDeg": rot_deg, "bs": bs_eps}
    with io.open(os.path.join(args.out_dir, "summary_%s.json" % args.profile), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("报告：%s" % report_path)
    if args.profile == "body":
        print("驱动姿态一致：%.5f mm | 修形动态位置最大偏差：%.3f mm | 旋转输出最大偏差：%.3f°"
              % (summary["driverPosMm"], summary["maxDynamicPosMm"], summary["maxRotDeg"]))
    else:
        print("关节位置最大差 %s mm | 旋转最大差 %s° | BlendShape 最大差 %s"
              % (sci(summary["maxPosMm"]), sci(summary["maxRotDeg"]), sci(summary["maxBs"])))
    print("判定：%s" % ("PASS" if summary["pass"] else "FAIL"))
    if not summary["pass"] and not args.no_fail:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
