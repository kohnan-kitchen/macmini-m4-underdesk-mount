#!/usr/bin/env python3
"""Mac mini (M4) デスク下吊り下げマウント（一体成型）— STL 生成スクリプト

    .venv/bin/python macmini_underdesk_mount.py              # 印刷用の本体を出力
    .venv/bin/python macmini_underdesk_mount.py --assembly   # 組み付けイメージも出力
    .venv/bin/python macmini_underdesk_mount.py --section    # 断面（色分け用に3部品）も出力

要 trimesh + manifold3d。

■ 形
  左右の側板を底の前後で渡し材でつないだ、一体成型のクレードル。
  上面と底面中央、前後は開いていて、Mac mini は前から差し込む。

    ┌── 天板（木ビスで固定）
    ══╤═══════════════════════════╤══   ← フランジ（外向き）＋45°リブ
      │                           │
      │        Mac mini           │     ← 側板（肉抜き窓あり）
      │                           │
      └─┐                       ┌─┘
        └───────────────────────┘       ← 受け皿（左右）＋渡し材（前後端）

■ 設計方針
  - 設置姿勢のまま印刷する。底枠がプレートに接地し、片持ちの下向き面（サポートが
    無いと崩れる面）を作らない。残るのはフランジ下面のリブ間ブリッジ 22mm だけで、
    45°リブと肉抜き窓の切妻は自己支持できる。スクリプトがこれを毎回実測して検証する。
  - フランジは外向き。内向きにすると下面が幅16mmの宙浮きになって支えられない。
    外向きなら45°リブで支えられ、しかも本体を入れたままビスを締められる。
  - 左右を繋ぐ渡し材は底の前後端、本体の外側に置く。Mac mini M4 は底面のほぼ全体が
    円形の着脱式カバーで、その前半分が吸気・後半分が排気になっているため、
    底面には受け皿の縁 7.5mm 以外いっさいかからないようにしている。
  - 底が「側面の受け皿＋前後の渡し材」で閉じた枠になるので、左右の間隔が
    荷重やビスの緩みで開くことがない（＝本体が落ちない）。

■ 座標系（設置姿勢＝印刷姿勢＝この STL の姿勢）
    x : 前(-) → 後(+)   0 が Mac mini の中心
    y : 左(-) → 右(+)   0 が Mac mini の中心
    z : 下(0, 造形プレート) → 上（天板に当たる面）
"""

import sys

import numpy as np
import trimesh
from trimesh.creation import box, cylinder, extrude_triangulation, revolve
from trimesh.transformations import rotation_matrix

# ------------------------------------------------------- Mac mini 側の寸法
# 出典: Apple 技術仕様 https://support.apple.com/en-us/121555
#   5.0 x 5.0 x 2.0 インチ = 127.0 x 127.0 x 50.8 mm
#   Apple の cm 表記は「5.0cm」だが 2.0 インチは正確には 50.8mm。高い方を採る。
#   ChargerLAB の実測は 127.1 x 127.1 x 49.7 mm。
BODY_W = 127.0     # 幅（左右 = y）
BODY_D = 127.0     # 奥行（前後 = x）
BODY_H = 50.8      # 高さ
BODY_KG = 0.73     # M4 Pro の重量（重い方）

FIT_SIDE = 0.5     # 側板と本体側面の隙間（片側）
FIT_TOP = 1.5      # 本体上面とフランジ下面の隙間

# ---------------------------------------------------------- マウントの寸法
T_LIP = 5.0        # 受け皿と渡し材の厚み（底枠の厚み）
T_WALL = 3.2       # 側板の厚み
T_FLANGE = 6.0     # 天板に当たるフランジの厚み

LIP_LEN = 6.0      # 受け皿の内向き突出量（底面の吸排気を塞がない範囲）
LIP_INSET = 15.0   # 受け皿を本体の前後端からこれだけ引っ込める（電源ボタンの逃げ）
LIP_CHAMFER = 2.0  # 受け皿端の面取り（差し込みのガイド）

TIE_W = 8.0        # 前後の渡し材の幅（前後方向）
TIE_GAP = 1.0      # 本体の前後端と渡し材の隙間
STOP_H = 5.0       # 前後の渡し材を受け皿の上面より立ち上げる量。
                   # 本体上面の余裕 FIT_TOP(1.5mm) より高いので、天板に付けたあとでは
                   # 前から差し込めない。先に本体を入れてから天板に留める手順になる。

FLANGE_LEN = 16.0  # フランジの外向き突出量
PAD_L = 34.0       # フランジ1枚の長さ（前後方向）
PAD_X = (-48.0, 48.0)   # フランジの中心位置（左右それぞれに2枚 = 計4枚）
T_RIB = 6.0        # フランジを支える45°リブの厚み

# 木ビス（呼び径 4.2mm、皿頭/ラッパ頭）
SCREW_D = 4.2
HOLE_D = 4.8            # 下穴（ビスの軸が通る）
CS_HEAD_D = 9.0         # 皿面取りの開口径
CS_ANGLE_DEG = 90.0     # 皿面取りの頂角

# 側板のルーバー（縦スリットの列）
LOUVER = True
LV_W = 7.0         # スリットの幅
LV_PITCH = 12.0    # スリットのピッチ
LV_SPAN = 132.0    # スリット列の全長（前後方向）
LV_MARGIN = 8.0    # スリットと底枠の間に残す帯の幅
LV_TOP_GAP = 4.3   # スリット上端と45°リブの根元の間に残す帯の幅

# 外側の縦角を丸める
CORNER_R = 6.0     # 側板・渡し材でできる外形の四隅
PAD_R = 4.0        # フランジとリブの角

SECTIONS = 96      # 円筒・円錐の分割数
CUT_EPS = 0.5      # 切削ソリッドを面から出す量。同一平面を避けてブーリアンを安定させる

# ------------------------------------------------------------------ 導出値
POCKET_H = BODY_H + FIT_TOP                 # 本体が入る鉛直方向の空間
TOTAL_H = T_LIP + POCKET_H + T_FLANGE       # 全高
FLANGE_Z0 = TOTAL_H - T_FLANGE              # フランジ下面

WALL_IN = BODY_W / 2 + FIT_SIDE             # 側板の内面
WALL_OUT = WALL_IN + T_WALL                 # 側板の外面
LIP_IN = WALL_IN - LIP_LEN                  # 受け皿の内側の縁
LIP_X = BODY_D / 2 - LIP_INSET              # 受け皿の前後端

PART_D = BODY_D + 2 * (TIE_GAP + TIE_W)     # 全長（前後）
PART_W = 2 * (WALL_OUT + FLANGE_LEN)        # 全幅（左右、フランジ含む）
SCREW_Y = WALL_OUT + FLANGE_LEN / 2         # ビス穴の左右位置

CS_DEPTH = (CS_HEAD_D - HOLE_D) / 2.0 / np.tan(np.radians(CS_ANGLE_DEG / 2.0))
RIB_Z0 = FLANGE_Z0 - FLANGE_LEN             # 45°リブの根元（フランジ幅と同じだけ下がる）
LV_Z0 = T_LIP + LV_MARGIN                   # スリットの下端
LV_Z1 = RIB_Z0 - LV_TOP_GAP                 # スリットの上端


# --------------------------------------------------------------- 部品の生成
def aabox(x0, x1, y0, y1, z0, z1):
    """軸に平行な直方体を境界値で作る。"""
    m = box(extents=(x1 - x0, y1 - y0, z1 - z0))
    m.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return m


def prism_along(pts2d, a0, a1, axis):
    """2D三角形の断面を、指定軸方向に押し出した三角柱を作る。

    extrude_triangulation は 2D 断面を +Z に押し出すので、軸が x / y の場合は
    行列式 +1 の巡回置換で向きを変える（鏡像にすると裏返るため）。
    断面座標の取り方は軸ごとに違う:
        axis="x" → 断面は (y, z)
        axis="y" → 断面は (z, x)
    """
    v = np.array(pts2d, dtype=float)
    d1, d2 = v[1] - v[0], v[2] - v[0]
    if d1[0] * d2[1] - d1[1] * d2[0] < 0:            # 反時計回りに揃える
        v = v[::-1]
    m = extrude_triangulation(vertices=v, faces=np.array([[0, 1, 2]]), height=a1 - a0)
    if axis == "z":
        m.apply_translation((0, 0, a0))
        return m
    if axis == "x":      # (u,v,w) → (w+a0, u, v)
        t = np.array([[0, 0, 1, a0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
    elif axis == "y":    # (u,v,w) → (v, w+a0, u)
        t = np.array([[0, 1, 0, 0], [0, 0, 1, a0], [1, 0, 0, 0], [0, 0, 0, 1]], float)
    else:
        raise ValueError(axis)
    assert np.isclose(np.linalg.det(t[:3, :3]), 1.0)
    m.apply_transform(t)
    return m


def rounded_rect_prism_y(x0, x1, z0, z1, r, y0, y1):
    """角Rの付いた角柱を y 方向に押し出す。箱2つ＋四隅の円柱の和。肉抜き窓の切削用。"""
    parts = [
        aabox(x0 + r, x1 - r, y0, y1, z0, z1),
        aabox(x0, x1, y0, y1, z0 + r, z1 - r),
    ]
    for cx in (x0 + r, x1 - r):
        for cz in (z0 + r, z1 - r):
            cyl = cylinder(radius=r, height=y1 - y0, sections=SECTIONS)
            cyl.apply_transform(rotation_matrix(-np.pi / 2, (1, 0, 0)))
            cyl.apply_translation((cx, (y0 + y1) / 2, cz))
            parts.append(cyl)
    return trimesh.boolean.union(parts)


def rounded_rect_prism_z(x0, x1, y0, y1, r, z0, z1):
    """角Rの付いた角柱を z 方向に押し出す。外形の縦角を丸めるための交差用ソリッド。"""
    parts = [aabox(x0 + r, x1 - r, y0, y1, z0, z1)]
    if (y1 - y0) > 2 * r:
        parts.append(aabox(x0, x1, y0 + r, y1 - r, z0, z1))
    for cx in (x0 + r, x1 - r):
        for cy in {y0 + r, y1 - r}:
            cyl = cylinder(radius=r, height=z1 - z0, sections=SECTIONS)
            cyl.apply_translation((cx, cy, (z0 + z1) / 2))
            parts.append(cyl)
    return trimesh.boolean.union(parts)


def louver_cut_y(y0, y1):
    """側板に開ける縦スリットの列。

    上下を45°の山形にした縦長の六角形。矩形にすると天井が水平のブリッジになるが、
    山形なら下向きの面がちょうど45°になって自己支持でき、ブリッジが1つも出ない。
    幅が狭いので山の高さは幅の半分だけで済み、見た目もグリルらしくなる。
    """
    n = int(LV_SPAN // LV_PITCH)
    half = LV_W / 2
    slots = []
    for i in range(n):
        cx = -(n - 1) * LV_PITCH / 2 + i * LV_PITCH
        slots.append(aabox(cx - half, cx + half, y0, y1,
                           LV_Z0 + half - 0.3, LV_Z1 - half + 0.3))
        slots.append(prism_along(                                    # 上の山
            [(LV_Z1 - half, cx - half), (LV_Z1 - half, cx + half), (LV_Z1, cx)],
            y0, y1, axis="y"))
        slots.append(prism_along(                                    # 下の山
            [(LV_Z0 + half, cx - half), (LV_Z0 + half, cx + half), (LV_Z0, cx)],
            y0, y1, axis="y"))
    return trimesh.boolean.union(slots)


def screw_cut(x, y):
    """皿面取り付きのビス穴を、回転体ひとつとして作る。

    円錐と円筒を別々のソリッドとして重ねると、半径が一致する円で2つの面が交差し、
    しかも同じ角度分割なので頂点まで一致して、ブーリアンが重複面と退化三角形を作る。
    そこで (半径, 高さ) の断面を1つ描いて回すことで、自己交差のない切削ソリッドにする。

    皿面取りはフランジの下面側。下から差し込んで天板にねじ込む向き。
    """
    half = np.tan(np.radians(CS_ANGLE_DEG / 2.0))
    z_low = FLANGE_Z0 - CUT_EPS          # フランジ下面より少し下（同一平面を避ける）
    z_high = TOTAL_H + 2.0               # フランジ上面より上（貫通させる）
    profile = [
        (0.0, z_low),
        (CS_HEAD_D / 2 + CUT_EPS * half, z_low),   # 下面での開口径を CS_HEAD_D に保つ
        (HOLE_D / 2, FLANGE_Z0 + CS_DEPTH),
        (HOLE_D / 2, z_high),
        (0.0, z_high),
    ]
    cut = revolve(np.array(profile, dtype=float), sections=SECTIONS)
    cut.apply_translation((x, y, 0.0))
    return cut


def build_mount():
    parts = []

    for sy in (-1.0, 1.0):
        w_in, w_out = sy * WALL_IN, sy * WALL_OUT
        # 側板
        parts.append(aabox(-PART_D / 2, PART_D / 2,
                           min(w_in, w_out), max(w_in, w_out), 0.0, TOTAL_H))
        # 受け皿
        l_in = sy * LIP_IN
        parts.append(aabox(-LIP_X, LIP_X,
                           min(l_in, w_in), max(l_in, w_in), 0.0, T_LIP))
        # フランジと、それを支える45°リブ
        f_out = sy * (WALL_OUT + FLANGE_LEN)
        for px in PAD_X:
            parts.append(aabox(px - PAD_L / 2, px + PAD_L / 2,
                               min(w_out, f_out), max(w_out, f_out),
                               FLANGE_Z0, TOTAL_H))
            for rx in (px - PAD_L / 2, px + PAD_L / 2 - T_RIB):
                # 断面は (y, z) の直角三角形。斜辺が45°で下外を向く
                parts.append(prism_along(
                    [(w_out, RIB_Z0), (w_out, FLANGE_Z0), (f_out, FLANGE_Z0)],
                    rx, rx + T_RIB, axis="x"))

    # 前後の渡し材（本体の外側に置くので底面の吸排気にかからない）
    # 前後とも受け皿の上面より STOP_H 立ち上げて、前後方向に抜けないようにする。
    for sx in (-1.0, 1.0):
        t0 = sx * (BODY_D / 2 + TIE_GAP)
        t1 = sx * (BODY_D / 2 + TIE_GAP + TIE_W)
        parts.append(aabox(min(t0, t1), max(t0, t1), -WALL_OUT, WALL_OUT,
                           0.0, T_LIP + STOP_H))

    solid = trimesh.boolean.union(parts)

    cuts = []
    for px in PAD_X:
        for sy in (-1.0, 1.0):
            cuts.append(screw_cut(px, sy * SCREW_Y))

    # 受け皿端の面取り（差し込みのガイド）。断面は (z, x) の直角三角形を y 方向へ押し出す
    for x0, sign in ((-LIP_X, +1.0), (LIP_X, -1.0)):
        c = LIP_CHAMFER
        section = [(T_LIP, x0), (T_LIP, x0 + sign * c), (T_LIP - c, x0)]
        for sy in (-1.0, 1.0):
            y0, y1 = sorted((sy * LIP_IN, sy * WALL_IN))
            cuts.append(prism_along(section, y0 - 1.0, y1 + 1.0, axis="y"))

    if LOUVER:
        for sy in (-1.0, 1.0):
            y0, y1 = sorted((sy * (WALL_IN - 1.0), sy * (WALL_OUT + 1.0)))
            cuts.append(louver_cut_y(y0, y1))

    solid = trimesh.boolean.difference([solid] + cuts)

    # 外側の縦角を丸める。全体を覆う「角Rの付いた輪郭」と交差させて角だけ落とす。
    # フランジとリブは輪郭の外に出るので、その分の輪郭も足しておく。
    outline = [rounded_rect_prism_z(-PART_D / 2, PART_D / 2, -WALL_OUT, WALL_OUT,
                                    CORNER_R, -1.0, TOTAL_H + 1.0)]
    for sy in (-1.0, 1.0):
        f_out = sy * (WALL_OUT + FLANGE_LEN)
        w_in = sy * WALL_IN
        for px in PAD_X:
            outline.append(rounded_rect_prism_z(
                px - PAD_L / 2, px + PAD_L / 2,
                min(w_in, f_out), max(w_in, f_out),
                PAD_R, RIB_Z0 - 1.0, TOTAL_H + 1.0))
    return trimesh.boolean.intersection([solid, trimesh.boolean.union(outline)])


def body_box():
    """Mac mini の外形（干渉チェックと組み付けイメージ用）。"""
    return aabox(-BODY_D / 2, BODY_D / 2, -BODY_W / 2, BODY_W / 2,
                 T_LIP, T_LIP + BODY_H)


def build_assembly(mount):
    desk = aabox(-PART_D / 2 - 6, PART_D / 2 + 6, -PART_W / 2 - 6, PART_W / 2 + 6,
                 TOTAL_H, TOTAL_H + 12)
    return trimesh.util.concatenate([mount, body_box(), desk])


# ------------------------------------------------------------------ 検証・出力
def floating_patches(mesh, limit_deg=46.0):
    """宙に浮いた下向きの面を、連結した塊（パッチ）ごとに切り出す。

    造形方向を +Z とし、水平から limit_deg 未満の下向きの面を対象にする。
    しきい値をちょうど 45° にすると、45°の斜面が浮動小数の誤差で引っかかるので
    少し寝かせた 46° を既定にしている（45°の面は自己支持できるため除外したい）。
    プレートに接地している最下面も除く。
    """
    tri = mesh.vertices[mesh.faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        nz = np.where(ln > 0, n[:, 2] / np.where(ln > 0, ln, 1), 0.0)
    top_z = tri[:, :, 2].max(axis=1)
    sel = (ln > 0) & (nz < -np.sin(np.radians(limit_deg))) & (top_z > mesh.bounds[0][2] + 1e-6)
    idx = np.flatnonzero(sel)
    if len(idx) == 0:
        return []

    # 辺を共有する面をたどって塊にまとめる（Union-Find。graph エンジン不要）
    pos = {int(f): i for i, f in enumerate(idx)}
    parent = list(range(len(idx)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for fa, fb in mesh.face_adjacency:
        if int(fa) in pos and int(fb) in pos:
            ra, rb = find(pos[int(fa)]), find(pos[int(fb)])
            if ra != rb:
                parent[ra] = rb

    groups = {}
    for i, f in enumerate(idx):
        groups.setdefault(find(i), []).append(int(f))

    out = []
    for faces in groups.values():
        t = mesh.vertices[mesh.faces[faces]]
        a = 0.5 * np.linalg.norm(
            np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1).sum()
        out.append({
            "area": float(a),
            "z": float(t[:, :, 2].mean()),
            "x": (float(t[:, :, 0].min()), float(t[:, :, 0].max())),
            "y": (float(t[:, :, 1].min()), float(t[:, :, 1].max())),
        })
    return sorted(out, key=lambda p: -p["area"])


def classify_patch(mesh, patch, probe=1.5, drop=0.6):
    """宙浮きパッチが「ブリッジ」か「片持ち」かを判定する。

    パッチの少し下（drop mm）の高さで、XY のバウンディングボックスの外側 probe mm を
    4方向つついてみる。相対する2方向に肉があれば、その方向は両端支持＝ブリッジで、
    渡る距離はその辺の長さ。どちらの軸も片側しか支えが無ければ片持ちで、サポートが要る。
    """
    z = patch["z"] - drop
    (x0, x1), (y0, y1) = patch["x"], patch["y"]
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    probes = np.array([
        [x0 - probe, ym, z], [x1 + probe, ym, z],
        [xm, y0 - probe, z], [xm, y1 + probe, z],
    ])
    solid = mesh.contains(probes)
    spans = []
    if solid[0] and solid[1]:
        spans.append(("x", x1 - x0))
    if solid[2] and solid[3]:
        spans.append(("y", y1 - y0))
    if not spans:
        return "片持ち（要サポート）", None
    axis, span = min(spans, key=lambda s: s[1])
    return f"ブリッジ（{axis}方向 {span:.1f} mm）", span

def check(mesh, label):
    """メッシュ自体と、STL に書いて読み戻したものの両方を検証する。

    STL は座標を float32 で持つため、書き出しで頂点が丸められて融合し、
    メモリ上では気付かない非多様体（1辺に3面以上）が現れることがある。
    スライサーが読むのは STL なので、そちらで検証する。
    """
    import io
    from collections import Counter

    lo, hi = mesh.bounds
    back = trimesh.load(io.BytesIO(mesh.export(file_type="stl")), file_type="stl")
    _, cnt = np.unique(back.edges_sorted, axis=0, return_counts=True)
    shares = dict(Counter(cnt.tolist()))
    tri = back.vertices[back.faces]
    areas = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    degenerate = int((areas < 1e-9).sum())

    print(f"[{label}]")
    print(f"  外形             : {hi[0]-lo[0]:.1f} x {hi[1]-lo[1]:.1f} x {hi[2]-lo[2]:.1f} mm")
    print(f"  体積             : {mesh.volume / 1000:.2f} cm3")
    print(f"  水密             : {back.is_watertight}")
    print(f"  法線の向き一貫   : {back.is_winding_consistent}")
    print(f"  1辺の共有面数    : {shares}  (2 のみが正常)")
    print(f"  退化三角形       : {degenerate}")

    patches = floating_patches(back)
    total = sum(p["area"] for p in patches)
    print(f"  宙に浮いた下向き面: {total / 100:.2f} cm2 / {len(patches)} 箇所")
    cantilever = 0.0
    longest = 0.0
    for p in patches:
        if p["area"] < 5.0:                       # 0.05 cm2 未満は無視
            continue
        kind, span = classify_patch(back, p)
        print(f"    z={p['z']:5.1f} mm  {p['area']/100:5.2f} cm2  {kind}")
        if span is None:
            cantilever += p["area"]
        else:
            longest = max(longest, span)
    print(f"  → 片持ち {cantilever / 100:.2f} cm2 （0 ならサポート不要）"
          f" / 最長ブリッジ {longest:.1f} mm")

    return (back.is_watertight and back.is_winding_consistent
            and list(shares) == [2] and degenerate == 0
            and mesh.volume > 0 and cantilever < 5.0)


def main():
    mount = build_mount()
    if not check(mount, "マウント本体"):
        print("ERROR: メッシュまたはオーバーハングが不正です。STL は出力しません")
        return 1

    hit = trimesh.boolean.intersection([mount, body_box()])
    hit_vol = 0.0 if hit.is_empty or not np.isfinite(hit.volume) else hit.volume
    print(f"  本体との干渉体積 : {hit_vol / 1000:.4f} cm3  (0 なら干渉なし)")
    if hit_vol > 1e-6:
        print("ERROR: Mac mini と干渉しています。STL は出力しません")
        return 1

    mount.export("macmini_underdesk_mount.stl")
    print("  出力             : macmini_underdesk_mount.stl")

    print()
    print(f"全高              : {TOTAL_H:.1f} mm （受け皿 {T_LIP} + 内寸 {POCKET_H:.1f} + フランジ {T_FLANGE}）")
    print(f"全幅 x 全長        : {PART_W:.1f} x {PART_D:.1f} mm")
    print(f"側板内面の間隔    : {WALL_IN * 2:.1f} mm （本体 {BODY_W} + 片側 {FIT_SIDE}）")
    print(f"受け皿の受け幅    : {LIP_LEN - FIT_SIDE:.1f} mm （底面にかかるのはこの縁だけ）")
    print(f"受け皿端と本体端  : {LIP_INSET:.1f} mm （電源ボタンの逃げ）")
    print(f"前後のストッパー  : 受け皿の上面から {STOP_H:.1f} mm （本体上面の余裕は {FIT_TOP:.1f} mm）")
    print(f"ビス穴            : {len(PAD_X) * 2} 箇所 (x=±{abs(PAD_X[0]):.0f}, y=±{SCREW_Y:.1f})")
    print(f"皿面取り          : 開口 {CS_HEAD_D} mm / 深さ {CS_DEPTH:.2f} mm / 残り肉厚 {T_FLANGE - CS_DEPTH:.2f} mm")
    print(f"材料              : {mount.volume / 1000:.1f} cm3 ≒ PLA {mount.volume / 1000 * 1.24:.0f} g / PETG {mount.volume / 1000 * 1.27:.0f} g")
    print(f"必要個数          : 1個（一体成型）")

    if "--assembly" in sys.argv:
        build_assembly(mount).export("assembly_preview.stl")
        print()
        print("組み付けイメージ  : assembly_preview.stl （確認用・印刷不可）")

    if "--section" in sys.argv:
        # 半分だけ残した断面。preview.py に3つ渡すと色分けして描ける。
        # front = 前から見た断面（左右方向の収まり）、side = 横から見た断面（前後の
        # ストッパーの掛かり）。
        desk = aabox(-PART_D / 2 - 6, PART_D / 2 + 6, -PART_W / 2 - 6, PART_W / 2 + 6,
                     TOTAL_H, TOTAL_H + 12)
        halves = {
            "front": aabox(0.0, 300.0, -300.0, 300.0, -50.0, 200.0),
            "side": aabox(-300.0, 300.0, 0.0, 300.0, -50.0, 200.0),
        }
        for tag, half in halves.items():
            for part, solid in (("mount", mount), ("body", body_box()), ("desk", desk)):
                trimesh.boolean.intersection([solid, half]).export(
                    f"section_{tag}_{part}.stl")
        print()
        print("断面              : section_front_*.stl（前から）/ section_side_*.stl（横から）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
