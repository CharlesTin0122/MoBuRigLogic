# -*- coding: utf-8 -*-
"""MetaHuman 身体修形采样（Maya 2024 与 MotionBuilder 2019 共用）。

在目标应用内运行：按 poses.json 施加驱动骨骼的局部欧拉增量，采集 RigLogic
修形（输出）关节的世界矩阵与本地 TRS，写入 JSON 供 compare.py 对比。

本模块需要同时兼容：
  - Maya 2024 的 Python 3
  - MotionBuilder 2019 的 Python 2.7
因此不使用 f-string、类型注解与 keyword-only 参数。

采样约定
--------
* 姿态：以“当前静置姿态”为基准，叠加 poses.json 中的增量后采集；
* driver：RigLogic raw control 关节（Maya 侧 = 修形节点上游的 rotate 连接；
  MoBu 侧复用 Maya 侧导出的同名列表，保证两侧对比同一批骨骼）；
* output：RigLogic 修形输出关节（Maya 侧 = 修形节点的下游 rotate/translate/scale
  连接；MoBu 侧同样复用 Maya 侧名单）。
"""

import io
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))

#: Maya 侧身体修形 DG 节点（MetaHumanForMaya）
MAYA_RIGLOGIC_NODE = "body_rl4Embedded"

#: MoBu 侧身体修形约束名
MOBU_CONSTRAINT_NAME = "RigLogic Body Corrective"


# ----------------------------------------------------------------- 通用工具
def load_poses(path):
    """读取姿态表，返回 [(label, {joint: (dx, dy, dz)}), ...]。"""
    data = load_json(path)
    poses = []
    for item in data["poses"]:
        delta = {}
        for joint, value in item.get("delta", {}).items():
            delta[joint] = tuple(value)
        poses.append((item["label"], delta))
    return poses


def save_json(path, data):
    folder = os.path.dirname(path)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    # 二进制写入：py2 的 json.dumps 返回 str，py3 返回 str，统一 encode 后写 bytes
    text = json.dumps(data, ensure_ascii=True)
    with io.open(path, "wb") as f:
        f.write(text.encode("utf-8"))


def load_json(path):
    with io.open(path, "rb") as f:
        return json.loads(f.read().decode("utf-8"))


def _log(msg):
    # py2 的 print 语句在括号内只接受单个对象，统一用 % 拼接
    print(msg)


# ----------------------------------------------------------------- Maya 侧
def maya_drivers(cmds, node=MAYA_RIGLOGIC_NODE):
    """修形节点的输入来源关节（DNA raw control 骨骼）。"""
    drivers = set()
    for plug in cmds.listConnections(node, s=True, d=False, p=True) or []:
        upstream = cmds.listConnections(plug.split(".")[0], s=True, d=False, p=True) or []
        if upstream:
            drivers.add(upstream[0].split(".")[0])
    return sorted(drivers)


def maya_outputs(cmds, node=MAYA_RIGLOGIC_NODE):
    """修形节点的输出去向关节（RigLogic 修形骨骼）。"""
    plugs = cmds.listConnections(node, s=False, d=True, p=True) or []
    return sorted(set(plug.split(".")[0] for plug in plugs))


def _maya_snapshot(cmds, names):
    """世界矩阵 + 本地 TRS（translate/rotate/scale，均为 Maya 属性原始值）。"""
    mats = {}
    attrs = {}
    for name in names:
        mats[name] = [v for v in cmds.getAttr(name + ".worldMatrix[0]")]
        t = cmds.getAttr(name + ".translate")[0]
        r = cmds.getAttr(name + ".rotate")[0]
        s = cmds.getAttr(name + ".scale")[0]
        attrs[name] = [t[0], t[1], t[2], r[0], r[1], r[2], s[0], s[1], s[2]]
    return mats, attrs


def run_maya(poses, out_path, names_path, node=MAYA_RIGLOGIC_NODE):
    import maya.cmds as cmds

    if not cmds.objExists(node):
        raise RuntimeError("找不到 Maya 修形节点：%s" % node)

    drivers = maya_drivers(cmds, node)
    outputs = maya_outputs(cmds, node)
    if not drivers or not outputs:
        raise RuntimeError("修形节点 %s 没有连接驱动/输出关节" % node)

    rest = {}
    for joint in drivers:
        rest[joint] = list(cmds.getAttr(joint + ".rotate")[0])

    parents = {}
    for name in outputs:
        p = cmds.listRelatives(name, p=True)
        parents[name] = p[0] if p else None

    info = {
        "side": "maya",
        "app": "Maya " + cmds.about(version=True),
        "scene": cmds.file(q=True, sn=True),
        "riglogicNode": node,
    }
    result = {
        "app": info,
        "drivers": drivers,
        "outputs": outputs,
        "parent": parents,
        "rest": rest,
        "poses": [],
    }

    def apply(delta):
        for joint in drivers:
            base = rest[joint]
            d = delta.get(joint, (0.0, 0.0, 0.0))
            cmds.setAttr(joint + ".rotate", base[0] + d[0], base[1] + d[1], base[2] + d[2])

    for label, delta in poses:
        apply(delta)
        cmds.refresh(force=True)
        mats, attrs = _maya_snapshot(cmds, outputs + drivers)
        result["poses"].append({"label": label, "delta": delta, "mats": mats, "attrs": attrs})
        _log("  [maya] %s" % label)
    apply({})
    cmds.refresh(force=True)

    save_json(names_path, {"drivers": drivers, "outputs": outputs})
    save_json(out_path, result)
    return "maya: %d drivers / %d outputs / %d poses -> %s" % (
        len(drivers), len(outputs), len(poses), out_path)


# ----------------------------------------------------------------- MoBu 侧
def _mobu_components(fb):
    comps = {}
    for comp in fb.FBSystem().Scene.Components:
        if isinstance(comp, fb.FBModelSkeleton):
            comps.setdefault(comp.Name, comp)
    return comps


def _mobu_find_constraint(fb, name=MOBU_CONSTRAINT_NAME):
    for constraint in fb.FBSystem().Scene.Constraints:
        if constraint.Name == name:
            return constraint
    return None


def _mobu_snapshot(fb, comps, names):
    """世界矩阵 + 本地 TRS（Lcl Translation / Lcl Rotation / Lcl Scaling）。

    注意：插件的缩放输出写在 ``Lcl Scaling`` 上，``Scaling`` 属性恒为 1，
    读错属性会误判成“缩放不一致”。
    """
    mats = {}
    attrs = {}
    mat = fb.FBMatrix()
    for name in names:
        model = comps.get(name)
        if model is None:
            continue
        model.GetMatrix(mat, fb.FBModelTransformationType.kModelTransformation, True)
        mats[name] = [mat[i] for i in range(16)]
        t = model.PropertyList.Find("Lcl Translation").Data
        r = model.PropertyList.Find("Lcl Rotation").Data
        s = model.PropertyList.Find("Lcl Scaling").Data
        attrs[name] = [t[0], t[1], t[2], r[0], r[1], r[2], s[0], s[1], s[2]]
    return mats, attrs


def run_mobu(poses, out_path, names_path, constraint_name=MOBU_CONSTRAINT_NAME):
    import pyfbsdk as fb
    fb.FBSystem()

    names = load_json(names_path)
    drivers = names["drivers"]
    wanted_outputs = names["outputs"]

    comps = _mobu_components(fb)
    missing = [name for name in drivers if name not in comps]
    outputs = [name for name in wanted_outputs if name in comps]

    constraint = _mobu_find_constraint(fb, constraint_name)
    info = {
        "side": "mobu",
        "app": "MotionBuilder 2019",
        "constraint": constraint_name,
        "dna": str(constraint.PropertyList.Find("DNA Path").Data) if constraint else "",
        "lod": int(constraint.PropertyList.Find("LOD Level").Data) if constraint else -1,
    }
    if constraint is None:
        raise RuntimeError("找不到 MoBu 约束：%s（请先确认插件已加载并建好约束）" % constraint_name)

    rest = {}
    for joint in drivers:
        model = comps.get(joint)
        if model is None:
            continue
        rest[joint] = list(model.PropertyList.Find("Lcl Rotation").Data)

    parents = {}
    for name in outputs:
        parent = comps[name].Parent
        parents[name] = parent.Name if parent else None

    result = {
        "app": info,
        "drivers": drivers,
        "outputs": outputs,
        "parent": parents,
        "rest": rest,
        "missingDrivers": missing,
        "poses": [],
    }

    def apply(delta):
        for joint in drivers:
            model = comps.get(joint)
            if model is None:
                continue
            base = rest[joint]
            d = delta.get(joint, (0.0, 0.0, 0.0))
            model.PropertyList.Find("Lcl Rotation").Data = fb.FBVector3d(
                base[0] + d[0], base[1] + d[1], base[2] + d[2])
        fb.FBSystem().Scene.Evaluate()

    for label, delta in poses:
        apply(delta)
        fb.FBSystem().Scene.Evaluate()
        mats, attrs = _mobu_snapshot(fb, comps, outputs + drivers)
        result["poses"].append({"label": label, "delta": delta, "mats": mats, "attrs": attrs})
        _log("  [mobu] %s" % label)
    apply({})

    save_json(out_path, result)
    text = "mobu: %d drivers / %d outputs / %d poses -> %s" % (
        len(drivers), len(outputs), len(poses), out_path)
    if missing:
        text += "（缺失驱动骨骼：%s）" % ", ".join(missing)
    return text


def run_side(side, poses_path, out_path, names_path):
    poses = load_poses(poses_path)
    if side == "maya":
        return run_maya(poses, out_path, names_path)
    if side == "mobu":
        return run_mobu(poses, out_path, names_path)
    raise ValueError("side 只能是 'maya' 或 'mobu'")
