#!/usr/bin/env python3
"""STL の確認用プレビュー PNG を書き出す（依存ライブラリなし）。

    python3 preview.py <入力.stl> [出力.png] [--dir=x,y,z]
    python3 preview.py a.stl b.stl c.stl 出力.png    # 部品ごとに色を変えて重ねる

--dir は視線方向（既定 -0.55,0.75,-0.60 ＝斜め上から見下ろす）。
z を正にすると下から見上げる図になる。
STL を複数渡すと、渡した順に PALETTE の色を割り当てて1枚に描く。

Zバッファ方式の簡易ソフトウェアレンダラ。印刷前に形状を目視確認するためのもの。
"""

import math
import os
import struct
import sys
import zlib

WIDTH, HEIGHT = 1200, 900
BG = (250, 249, 246)
PALETTE = [
    (214, 129, 74),          # PLA オレンジ
    (108, 142, 191),         # 青
    (150, 150, 150),         # グレー
    (168, 196, 124),         # 緑
]
LIGHT = (-0.39, -0.49, 0.78)  # 光源の向き（ワールド座標・正規化済み）
VIEW_DIR = (-0.55, 0.75, -0.60)  # 視線方向。main で上書きされる
FILL = 0.30                      # 反対側からの補助光。裏側から見ても形が読めるようにする


def load_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        tris = []
        for _ in range(n):
            d = struct.unpack("<12f", f.read(48))
            f.read(2)
            tris.append((d[3:6], d[6:9], d[9:12]))
    return tris


def normalize(v):
    ln = math.sqrt(sum(c * c for c in v))
    return tuple(c / ln for c in v)


def make_camera(tris):
    """モデルを見下ろす等角風のカメラ基底 (right, up, forward) と中心を作る。"""
    pts = [v for t in tris for v in t]
    center = tuple(sum(p[k] for p in pts) / len(pts) for k in range(3))
    fwd = normalize(VIEW_DIR)                       # 視線方向
    right = normalize((fwd[1], -fwd[0], 0.0))       # 水平を保つ
    up = (
        right[1] * fwd[2] - right[2] * fwd[1],
        right[2] * fwd[0] - right[0] * fwd[2],
        right[0] * fwd[1] - right[1] * fwd[0],
    )
    return center, right, up, fwd


def render(groups, path):
    """groups は [(三角形リスト, 色), ...]。渡した順に重ねて描く。"""
    all_tris = [t for tris, _ in groups for t in tris]
    center, right, up, fwd = make_camera(all_tris)

    def project(v):
        d = (v[0] - center[0], v[1] - center[1], v[2] - center[2])
        return (
            sum(d[k] * right[k] for k in range(3)),
            sum(d[k] * up[k] for k in range(3)),
            sum(d[k] * fwd[k] for k in range(3)),
        )

    cam, world, cols = [], [], []
    for tris, col in groups:
        for t in tris:
            cam.append(tuple(project(v) for v in t))
            world.append(t)
            cols.append(col)
    us = [v[0] for t in cam for v in t]
    vs = [v[1] for t in cam for v in t]
    scale = 0.86 * min(WIDTH / (max(us) - min(us)), HEIGHT / (max(vs) - min(vs)))
    ox = WIDTH / 2 - (max(us) + min(us)) / 2 * scale
    oy = HEIGHT / 2 + (max(vs) + min(vs)) / 2 * scale

    color = bytearray()
    for _ in range(WIDTH * HEIGHT):
        color += bytes(BG)
    depth = [float("inf")] * (WIDTH * HEIGHT)

    for tri, wtri, base in zip(cam, world, cols):
        # 法線・カリング・陰影はすべてワールド座標で扱う（カメラ基底の手系に依存しない）
        u = tuple(wtri[1][k] - wtri[0][k] for k in range(3))
        w = tuple(wtri[2][k] - wtri[0][k] for k in range(3))
        n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        ln = math.sqrt(sum(q * q for q in n))
        if ln == 0:
            continue
        n = tuple(q / ln for q in n)
        if sum(n[k] * fwd[k] for k in range(3)) > 0:   # 視線と同じ向き＝裏面
            continue
        d = sum(n[k] * LIGHT[k] for k in range(3))
        shade = 0.26 + 0.62 * max(0.0, d) + FILL * max(0.0, -d)
        col = tuple(min(255, int(ch * shade)) for ch in base)

        p = [(ox + v[0] * scale, oy - v[1] * scale, v[2]) for v in tri]
        minx = max(0, int(min(q[0] for q in p)))
        maxx = min(WIDTH - 1, int(max(q[0] for q in p)) + 1)
        miny = max(0, int(min(q[1] for q in p)))
        maxy = min(HEIGHT - 1, int(max(q[1] for q in p)) + 1)
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = p
        den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(den) < 1e-12:
            continue
        for py in range(miny, maxy + 1):
            for px in range(minx, maxx + 1):
                fx, fy = px + 0.5, py + 0.5
                l0 = ((y1 - y2) * (fx - x2) + (x2 - x1) * (fy - y2)) / den
                l1 = ((y2 - y0) * (fx - x2) + (x0 - x2) * (fy - y2)) / den
                l2 = 1.0 - l0 - l1
                if l0 < 0 or l1 < 0 or l2 < 0:
                    continue
                z = l0 * z0 + l1 * z1 + l2 * z2
                idx = py * WIDTH + px
                if z < depth[idx]:
                    depth[idx] = z
                    color[idx * 3:idx * 3 + 3] = bytes(col)

    raw = bytearray()
    for y in range(HEIGHT):
        raw.append(0)
        raw += color[y * WIDTH * 3:(y + 1) * WIDTH * 3]

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        f.write(chunk(b"IEND", b""))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for a in sys.argv[1:]:
        if a.startswith("--dir="):
            VIEW_DIR = tuple(float(v) for v in a.split("=", 1)[1].split(","))
    if not args:
        print(__doc__.strip())
        raise SystemExit(1)
    srcs = [a for a in args if a.lower().endswith(".stl")]
    pngs = [a for a in args if a.lower().endswith(".png")]
    if not srcs:
        print(__doc__.strip())
        raise SystemExit(1)
    dst = pngs[0] if pngs else os.path.join(
        os.path.dirname(os.path.abspath(srcs[0])), "preview.png")
    render([(load_stl(s), PALETTE[i % len(PALETTE)]) for i, s in enumerate(srcs)], dst)
    print(f"出力: {dst}")
