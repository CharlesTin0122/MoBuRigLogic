# -*- coding: utf-8 -*-
"""MetaHuman 修形采样（Maya 2024 与 MotionBuilder 2019 共用）。

三个 profile：

* ``body``       身体修形：输入 = 驱动骨骼（DNA raw control）的局部欧拉增量；
                 输出 = RigLogic 修形关节，采集世界矩阵 + 本地 TRS。
* ``face``       面部修形：输入 = FaceBoard 控制器 ``CTRL_*`` 的 translate（0..1）；
                 输出 = ``FACIAL_*`` 关节 + BlendShape/morph 权重 + DNA 网格位姿。
* ``face_sweep`` 面部全控制器扫描：Maya 侧从 ``animCurveUU`` 反查每个
                 ``CTRL_*`` 控制器，逐个单独摆到 1.0，MoBu 侧复用同一份列表。

本模块需要同时兼容：
  - Maya 2024 的 Python 3
  - MotionBuilder 2019 的 Python 2.7
因此不使用 f-string、类型注解与 keyword-only 参数。
"""

import io
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))

PROFILES = {
    "body": {
        "kind": "body",
        "mayaNode": "body_rl4Embedded",
        "mobuConstraint": "RigLogic Body Corrective",
        "poses": "poses.json",
    },
    "face": {
        "kind": "face",
        "mayaNode": "head_rl4Embedded",
        "mobuConstraint": "RigLogic Head Expression",
        "poses": "poses_face.json",
    },
    "face_sweep": {
        "kind": "face",
        "mayaNode": "head_rl4Embedded",
        "mobuConstraint": "RigLogic Head Expression",
        "poses": "poses_face_sweep.json",
    },
}


# ----------------------------------------------------------------- 通用工具
def load_json(path):
    with io.open(path, "rb") as f:
        return json.loads(f.read().decode("utf-8"))


def save_json(path, data):
    folder = os.path.dirname(path)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    text = json.dumps(data, ensure_ascii=True)
    with io.open(path, "wb") as f:
        f.write(text.encode("utf-8"))


def load_poses(path):
    """读取姿态文件，返回 (mode, poses, extra)。

    poses: [(label, delta_dict)]，delta 的语义由 profile 决定。
    """
    data = load_json(path)
    mode = data.get("mode", "static")
    poses = []
    for item in data.get("poses", []):
        delta = {}
        for joint, value in item.get("delta", {}).items():
            delta[joint] = tuple(value)
        poses.append((item["label"], delta))
    extra = {"value": float(data.get("value", 1.0)),
             "axis": data.get("axis", None),
             "skip": data.get("skip", [])}
    return mode, poses, extra


def _log(msg):
    print(msg)


# ----------------------------------------------------------------- Maya 侧
def maya_body_rig(cmds, node):
    drivers = set()
    for plug in cmds.listConnections(node, s=True, d=False, p=True) or []:
        upstream = cmds.listConnections(plug.split(".")[0], s=True, d=False, p=True) or []
        if upstream:
            drivers.add(upstream[0].split(".")[0])
    plugs = cmds.listConnections(node, s=False, d=True, p=True) or []
    outputs = sorted(set(plug.split(".")[0] for plug in plugs))
    return sorted(drivers), outputs


def maya_face_rig(cmds, node):
    plugs = cmds.listConnections(node, s=False, d=True, p=True) or []
    joints = sorted(set(p.split(".")[0] for p in plugs if p.split(".")[0].startswith("FACIAL")))
    channels = []
    for bs in cmds.ls(type="blendShape"):
        geo = cmds.blendShape(bs, q=True, g=True) or []
        if not geo:
            continue
        mesh = geo[0]
        if mesh.endswith("Shape"):
            mesh = mesh[: -len("Shape")]
        for index, channel in enumerate(cmds.listAttr(bs + ".w", m=True) or []):
            channels.append([mesh, channel, bs, index])
    return joints, channels, sorted(set(c[0] for c in channels))


def maya_face_controls(cmds):
    """从 animCurve* 上游反查 FaceBoard 的 translate 控制器：[[ctrl, axis, expr]]

    注意不能只扫 ``animCurveUU``：眼球朝向（``CTRL_L_eye``/``CTRL_R_eye``）走的是
    ``animCurveUA``（无单位→角度），只扫 UU 会漏掉整条眼珠控制链。
    """
    result = []
    seen = set()
    curves = []
    for curve_type in ("animCurveUU", "animCurveUA", "animCurveUL", "animCurveLA"):
        curves.extend(cmds.ls(type=curve_type) or [])
    for curve in curves:
        srcs = cmds.listConnections(curve, s=True, d=False, p=True) or []
        dsts = cmds.listConnections(curve, s=False, d=True, p=True) or []
        if not srcs or not dsts or not srcs[0].startswith("CTRL_"):
            continue
        ctrl, attr = srcs[0].split(".")
        if attr not in ("translateX", "translateY", "translateZ"):
            continue
        axis = {"translateX": 0, "translateY": 1, "translateZ": 2}[attr]
        key = (ctrl, axis)
        if key in seen:
            continue
        seen.add(key)
        result.append([ctrl, axis, dsts[0].split(".")[1]])
    result.sort(key=lambda r: (r[2], r[0]))
    return result


def _maya_snapshot_body(cmds, names):
    mats = {}
    attrs = {}
    for name in names:
        mats[name] = [v for v in cmds.getAttr(name + ".worldMatrix[0]")]
        t = cmds.getAttr(name + ".translate")[0]
        r = cmds.getAttr(name + ".rotate")[0]
        s = cmds.getAttr(name + ".scale")[0]
        attrs[name] = [t[0], t[1], t[2], r[0], r[1], r[2], s[0], s[1], s[2]]
    return {"mats": mats, "attrs": attrs}


def _maya_snapshot_face(cmds, joints, channels, meshes):
    T = {}
    R = {}
    S = {}
    pos = {}
    for j in joints:
        t = cmds.getAttr(j + ".translate")[0]
        r = cmds.getAttr(j + ".rotate")[0]
        s = cmds.getAttr(j + ".scale")[0]
        mat = cmds.getAttr(j + ".worldMatrix[0]")
        T[j] = [t[0], t[1], t[2]]
        R[j] = [r[0], r[1], r[2]]
        S[j] = [s[0], s[1], s[2]]
        pos[j] = [mat[12], mat[13], mat[14]]
    bs = {}
    for mesh, channel, node, index in channels:
        bs["%s::%s" % (mesh, channel)] = cmds.getAttr("%s.w[%d]" % (node, index))
    mesh_pos = {}
    for m in meshes:
        if cmds.objExists(m):
            mat = cmds.getAttr(m + ".worldMatrix[0]")
            mesh_pos[m] = [mat[12], mat[13], mat[14]]
    return {"T": T, "R": R, "S": S, "p": pos, "bs": bs, "meshP": mesh_pos}


def run_maya(profile_name, poses_path, out_path, names_path):
    import maya.cmds as cmds

    profile = PROFILES[profile_name]
    node = profile["mayaNode"]
    if not cmds.objExists(node):
        raise RuntimeError("找不到 Maya 修形节点：%s" % node)
    mode, poses, extra = load_poses(poses_path)

    info = {"side": "maya", "kind": profile["kind"], "profile": profile_name,
            "app": "Maya " + cmds.about(version=True),
            "scene": cmds.file(q=True, sn=True), "node": node}
    result = {"app": info, "poses": []}
    names = {"kind": profile["kind"], "profile": profile_name}

    if profile["kind"] == "body":
        drivers, outputs = maya_body_rig(cmds, node)
        if not drivers or not outputs:
            raise RuntimeError("修形节点 %s 没有连接驱动/输出关节" % node)
        rest = {}
        for j in drivers:
            rest[j] = list(cmds.getAttr(j + ".rotate")[0])
        parents = {}
        for name in outputs:
            p = cmds.listRelatives(name, p=True)
            parents[name] = p[0] if p else None

        def apply_body(delta):
            for j in drivers:
                base = rest[j]
                d = delta.get(j, (0.0, 0.0, 0.0))
                cmds.setAttr(j + ".rotate", base[0] + d[0], base[1] + d[1], base[2] + d[2])

        for label, delta in poses:
            apply_body(delta)
            cmds.refresh(force=True)
            snap = _maya_snapshot_body(cmds, outputs + drivers)
            snap.update({"label": label, "delta": delta})
            result["poses"].append(snap)
            _log("  [maya] %s" % label)
        for j in drivers:
            cmds.setAttr(j + ".rotate", *rest[j])
        cmds.refresh(force=True)

        result.update({"drivers": drivers, "outputs": outputs, "parent": parents, "rest": rest})
        names.update({"drivers": drivers, "outputs": outputs})
    else:
        joints, channels, meshes = maya_face_rig(cmds, node)
        if not joints or not channels:
            raise RuntimeError("Maya 侧没有找到面部修形关节/BlendShape")

        controls = maya_face_controls(cmds) if mode == "sweep" else []
        if mode == "sweep":
            skip = set(extra["skip"] or [])
            value = extra["value"]
            poses = [("F000_rest", {})]
            for n, (ctrl, axis, _expr) in enumerate(controls):
                if ctrl in skip:
                    continue
                v = [0.0, 0.0, 0.0]
                v[axis] = value
                poses.append(("F%03d_%s_t%s" % (n + 1, ctrl, "xyz"[axis]), {ctrl: tuple(v)}))

        used = sorted(set(k for _l, d in poses for k in d))
        rest = {}
        locks = {}
        driven = {}
        for ctrl in used:
            if not cmds.objExists(ctrl):
                rest[ctrl] = None
                continue
            rest[ctrl] = list(cmds.getAttr(ctrl + ".translate")[0])
            for axis in ("X", "Y", "Z"):
                attr = "%s.translate%s" % (ctrl, axis)
                locks[(ctrl, axis)] = bool(cmds.getAttr(attr, lock=True))
                driven[(ctrl, axis)] = bool(cmds.listConnections(attr, s=True, d=False))
                if locks[(ctrl, axis)] and not driven[(ctrl, axis)]:
                    cmds.setAttr(attr, lock=False)

        def reset_controls():
            for ctrl, value in rest.items():
                if value is None:
                    continue
                for i, axis in enumerate("XYZ"):
                    if driven.get((ctrl, axis)):
                        continue
                    cmds.setAttr("%s.translate%s" % (ctrl, axis), value[i])

        try:
            for label, delta in poses:
                reset_controls()
                for ctrl, value in delta.items():
                    if rest.get(ctrl) is None:
                        continue
                    for i, axis in enumerate("XYZ"):
                        if driven.get((ctrl, axis)):
                            continue
                        cmds.setAttr("%s.translate%s" % (ctrl, axis), value[i])
                cmds.refresh(force=True)
                snap = _maya_snapshot_face(cmds, joints, channels, meshes)
                snap.update({"label": label, "delta": delta})
                result["poses"].append(snap)
                _log("  [maya] %s" % label)
        finally:
            reset_controls()
            cmds.refresh(force=True)
            for (ctrl, axis), was_locked in locks.items():
                if driven.get((ctrl, axis)):
                    continue
                cmds.setAttr("%s.translate%s" % (ctrl, axis), lock=was_locked)

        result.update({"joints": joints,
                       "channels": [[m, c] for m, c, _n, _i in channels],
                       "meshes": meshes, "rest": rest,
                       "missingControls": [c for c, v in rest.items() if v is None]})
        names.update({"joints": joints,
                      "channels": [[m, c] for m, c, _n, _i in channels],
                      "meshes": meshes, "controls": controls})

    save_json(names_path, names)
    save_json(out_path, result)
    return "%s/%s: %d poses -> %s" % (profile_name, "maya", len(result["poses"]), out_path)


# ----------------------------------------------------------------- MoBu 侧
def _mobu_models(fb):
    models = {}
    for comp in fb.FBSystem().Scene.Components:
        if isinstance(comp, fb.FBModel):
            models.setdefault(comp.Name, comp)
    return models


def _mobu_constraint(fb, name):
    for constraint in fb.FBSystem().Scene.Constraints:
        if constraint.Name == name:
            return constraint
    return None


def run_mobu(profile_name, poses_path, out_path, names_path):
    import pyfbsdk as fb
    fb.FBSystem()

    profile = PROFILES[profile_name]
    mode, poses, extra = load_poses(poses_path)
    names = load_json(names_path)
    models = _mobu_models(fb)
    constraint = _mobu_constraint(fb, profile["mobuConstraint"])
    if constraint is None:
        raise RuntimeError("找不到 MoBu 约束：%s" % profile["mobuConstraint"])

    info = {"side": "mobu", "kind": profile["kind"], "profile": profile_name,
            "app": "MotionBuilder 2019", "constraint": profile["mobuConstraint"],
            "dna": str(constraint.PropertyList.Find("DNA Path").Data),
            "lod": int(constraint.PropertyList.Find("LOD Level").Data)}
    if profile["kind"] == "face":
        info["inputMode"] = int(constraint.PropertyList.Find("Input Mode").Data)
    result = {"app": info, "poses": []}

    if profile["kind"] == "body":
        drivers = names["drivers"]
        outputs = [n for n in names["outputs"] if n in models]
        missing = [n for n in drivers if n not in models]
        rest = {}
        for j in drivers:
            if j in models:
                rest[j] = list(models[j].PropertyList.Find("Lcl Rotation").Data)
        parents = {}
        for name in outputs:
            parent = models[name].Parent
            parents[name] = parent.Name if parent else None

        def apply_body(delta):
            for j in drivers:
                model = models.get(j)
                if model is None:
                    continue
                base = rest[j]
                d = delta.get(j, (0.0, 0.0, 0.0))
                model.PropertyList.Find("Lcl Rotation").Data = fb.FBVector3d(
                    base[0] + d[0], base[1] + d[1], base[2] + d[2])
            fb.FBSystem().Scene.Evaluate()

        matrix = fb.FBMatrix()
        for label, delta in poses:
            apply_body(delta)
            fb.FBSystem().Scene.Evaluate()
            mats = {}
            attrs = {}
            for name in outputs + drivers:
                model = models.get(name)
                if model is None:
                    continue
                model.GetMatrix(matrix, fb.FBModelTransformationType.kModelTransformation, True)
                mats[name] = [matrix[i] for i in range(16)]
                t = model.PropertyList.Find("Lcl Translation").Data
                r = model.PropertyList.Find("Lcl Rotation").Data
                s = model.PropertyList.Find("Lcl Scaling").Data
                attrs[name] = [t[0], t[1], t[2], r[0], r[1], r[2], s[0], s[1], s[2]]
            result["poses"].append({"label": label, "delta": delta, "mats": mats, "attrs": attrs})
            _log("  [mobu] %s" % label)
        apply_body({})

        result.update({"drivers": drivers, "outputs": outputs, "parent": parents,
                       "rest": rest, "missingDrivers": missing})
    else:
        joints = [j for j in names["joints"] if j in models]
        bs_bindings = []
        for mesh, channel in names["channels"]:
            model = models.get(mesh)
            if model is None:
                continue
            prop = model.PropertyList.Find(str("%s__%s" % (mesh, channel)))
            if prop is None:
                prop = model.PropertyList.Find(str(channel))
            if prop is not None:
                bs_bindings.append([mesh, channel, prop])
        meshes = [m for m in names.get("meshes", []) if m in models]

        controls = names.get("controls", []) if mode == "sweep" else []
        if mode == "sweep":
            skip = set(extra["skip"] or [])
            value = extra["value"]
            poses = [("F000_rest", {})]
            for n, (ctrl, axis, _expr) in enumerate(controls):
                if ctrl in skip:
                    continue
                v = [0.0, 0.0, 0.0]
                v[axis] = value
                poses.append(("F%03d_%s_t%s" % (n + 1, ctrl, "xyz"[axis]), {ctrl: tuple(v)}))

        used = sorted(set(k for _l, d in poses for k in d))
        rest = {}
        for ctrl in used:
            model = models.get(ctrl)
            rest[ctrl] = list(model.PropertyList.Find("Lcl Translation").Data) if model else None

        def reset_controls():
            for ctrl, value in rest.items():
                model = models.get(ctrl)
                if model is None or value is None:
                    continue
                model.PropertyList.Find("Lcl Translation").Data = fb.FBVector3d(value[0], value[1], value[2])
            fb.FBSystem().Scene.Evaluate()

        matrix = fb.FBMatrix()
        joint_props = {}
        for j in joints:
            m = models[j]
            joint_props[j] = (m, m.PropertyList.Find("Lcl Translation"),
                              m.PropertyList.Find("Lcl Rotation"),
                              m.PropertyList.Find("Lcl Scaling"))

        for label, delta in poses:
            reset_controls()
            for ctrl, value in delta.items():
                model = models.get(ctrl)
                if model is None:
                    continue
                model.PropertyList.Find("Lcl Translation").Data = fb.FBVector3d(value[0], value[1], value[2])
            fb.FBSystem().Scene.Evaluate()

            T = {}
            R = {}
            S = {}
            pos = {}
            for j in joints:
                model, pT, pR, pS = joint_props[j]
                t = pT.Data
                r = pR.Data
                s = pS.Data
                model.GetMatrix(matrix, fb.FBModelTransformationType.kModelTransformation, True)
                T[j] = [t[0], t[1], t[2]]
                R[j] = [r[0], r[1], r[2]]
                S[j] = [s[0], s[1], s[2]]
                pos[j] = [matrix[12], matrix[13], matrix[14]]
            bs = {}
            for mesh, channel, prop in bs_bindings:
                bs["%s::%s" % (mesh, channel)] = prop.Data
            mesh_pos = {}
            for m in meshes:
                models[m].GetMatrix(matrix, fb.FBModelTransformationType.kModelTransformation, True)
                mesh_pos[m] = [matrix[12], matrix[13], matrix[14]]
            result["poses"].append({"label": label, "delta": delta,
                                    "T": T, "R": R, "S": S, "p": pos, "bs": bs, "meshP": mesh_pos})
            _log("  [mobu] %s" % label)
        reset_controls()

        result.update({"joints": joints,
                       "channels": [[m, c] for m, c, _p in bs_bindings],
                       "meshes": meshes, "rest": rest})

    save_json(out_path, result)
    return "%s/%s: %d poses -> %s" % (profile_name, "mobu", len(result["poses"]), out_path)


def run_side(profile_name, side, poses_path, out_path, names_path):
    if profile_name not in PROFILES:
        raise ValueError("profile 只能是 %s" % ", ".join(sorted(PROFILES)))
    if side == "maya":
        return run_maya(profile_name, poses_path, out_path, names_path)
    if side == "mobu":
        return run_mobu(profile_name, poses_path, out_path, names_path)
    raise ValueError("side 只能是 'maya' 或 'mobu'")
