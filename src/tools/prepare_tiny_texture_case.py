#!/usr/bin/env python3
"""
Prepare a tiny synthetic Mesh -> Texture regression case.

This version does NOT download BOP or any external dataset.
It generates:
  - mesh.txt
  - cameras.txt
  - images/*.ppm
  - READY.txt

The generated case is deterministic, tiny (< 1 MiB), and keeps the same
command-line arguments used by the existing CMake test so it can be dropped
in as a replacement for prepare_bop_lm_texture_case.py.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil


def mat3_mul_vec(R, v):
    return (
        R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
        R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
        R[6] * v[0] + R[7] * v[1] + R[8] * v[2],
    )


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_norm(v):
    n = math.sqrt(vec_dot(v, v))
    if n <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def look_at_world_to_camera(camera_pos, target=(0.0, 0.0, 0.0)):
    """
    Build a BOP/OpenCV-style world->camera transform.
    Camera coordinates:
      +X right
      +Y down
      +Z forward
    """
    forward = vec_norm(vec_sub(target, camera_pos))
    world_up = (0.0, 1.0, 0.0)

    right = vec_norm(vec_cross(forward, world_up))
    if abs(vec_dot(right, right)) < 1e-8:
        world_up = (0.0, 0.0, 1.0)
        right = vec_norm(vec_cross(forward, world_up))

    up = vec_cross(right, forward)

    # OpenCV camera Y points down.
    down = (-up[0], -up[1], -up[2])

    R = [
        right[0], right[1], right[2],
        down[0], down[1], down[2],
        forward[0], forward[1], forward[2],
    ]

    rc = mat3_mul_vec(R, camera_pos)
    t = (-rc[0], -rc[1], -rc[2])
    return R, t


def write_mesh(path: Path):
    # 100 mm cube centered at origin.
    s = 50.0
    vertices = [
        (-s, -s, -s), ( s, -s, -s), ( s,  s, -s), (-s,  s, -s),
        (-s, -s,  s), ( s, -s,  s), ( s,  s,  s), (-s,  s,  s),
    ]

    # CCW winding seen from outside.
    faces = [
        (0, 2, 1), (0, 3, 2),  # -Z
        (4, 5, 6), (4, 6, 7),  # +Z
        (0, 4, 7), (0, 7, 3),  # -X
        (1, 2, 6), (1, 6, 5),  # +X
        (0, 1, 5), (0, 5, 4),  # -Y
        (3, 7, 6), (3, 6, 2),  # +Y
    ]

    with path.open("w", encoding="utf-8") as f:
        f.write("PCEDITOR_TEXTURE_MESH 1\n")
        f.write(f"vertices {len(vertices)}\n")
        for x, y, z in vertices:
            f.write(f"{x:.9g} {y:.9g} {z:.9g}\n")
        f.write(f"triangles {len(faces)}\n")
        for a, b, c in faces:
            f.write(f"{a} {b} {c}\n")

    return vertices, faces


def edge(ax, ay, bx, by, px, py):
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax)


def render_ppm(path: Path, vertices, faces, R, t, width, height, fx, fy, cx, cy, camera_index):
    # Background is intentionally non-black so missing texture coverage is obvious.
    pixels = bytearray([235, 235, 235] * (width * height))
    zbuf = [float("inf")] * (width * height)

    # Two triangles of each cube face share the same color.
    face_colors = [
        (220, 70, 70),    # -Z
        (70, 190, 90),    # +Z
        (70, 110, 220),   # -X
        (230, 180, 60),   # +X
        (190, 80, 200),   # -Y
        (70, 200, 200),   # +Y
    ]

    cam_vertices = []
    screen_vertices = []
    for v in vertices:
        q = mat3_mul_vec(R, v)
        q = (q[0] + t[0], q[1] + t[1], q[2] + t[2])
        cam_vertices.append(q)
        if q[2] <= 1e-6:
            screen_vertices.append(None)
        else:
            u = fx * q[0] / q[2] + cx
            vv = fy * q[1] / q[2] + cy
            screen_vertices.append((u, vv, q[2]))

    for tri_index, (ia, ib, ic) in enumerate(faces):
        sa = screen_vertices[ia]
        sb = screen_vertices[ib]
        sc = screen_vertices[ic]
        if sa is None or sb is None or sc is None:
            continue

        ax, ay, az = sa
        bx, by, bz = sb
        cxp, cyp, cz = sc

        area = edge(ax, ay, bx, by, cxp, cyp)
        if abs(area) < 1e-8:
            continue

        min_x = max(0, int(math.floor(min(ax, bx, cxp))))
        max_x = min(width - 1, int(math.ceil(max(ax, bx, cxp))))
        min_y = max(0, int(math.floor(min(ay, by, cyp))))
        max_y = min(height - 1, int(math.ceil(max(ay, by, cyp))))

        base = face_colors[tri_index // 2]

        for y in range(min_y, max_y + 1):
            py = y + 0.5
            for x in range(min_x, max_x + 1):
                px = x + 0.5
                w0 = edge(bx, by, cxp, cyp, px, py) / area
                w1 = edge(cxp, cyp, ax, ay, px, py) / area
                w2 = 1.0 - w0 - w1
                if w0 < -1e-7 or w1 < -1e-7 or w2 < -1e-7:
                    continue

                z = w0 * az + w1 * bz + w2 * cz
                idx = y * width + x
                if z >= zbuf[idx]:
                    continue
                zbuf[idx] = z

                # Add a deterministic image-space pattern so UV/image orientation
                # mistakes are easy to see.
                checker = ((x // 16) + (y // 16) + camera_index) & 1
                factor = 1.0 if checker == 0 else 0.72
                r = max(0, min(255, int(base[0] * factor)))
                g = max(0, min(255, int(base[1] * factor)))
                b = max(0, min(255, int(base[2] * factor)))

                off = idx * 3
                pixels[off:off + 3] = bytes((r, g, b))

    with path.open("wb") as f:
        f.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        f.write(pixels)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cache", type=Path, required=True)  # accepted for CMake compatibility
    ap.add_argument("--scene-id", type=int, default=1)   # ignored
    ap.add_argument("--object-id", type=int, default=1)  # ignored
    ap.add_argument("--camera-count", type=int, default=6)
    ap.add_argument("--max-image-width", type=int, default=256)
    ap.add_argument("--force-download", action="store_true")  # compatibility; means regenerate
    ap.add_argument("--no-auto-install", action="store_true") # compatibility
    ap.add_argument("--source-root", type=Path)                # compatibility
    args = ap.parse_args()

    out = args.out.resolve()
    marker = out / "READY.txt"

    if marker.is_file() and not args.force_download:
        print(f"[prepare] tiny texture case already ready: {out}")
        return 0

    if out.exists():
        shutil.rmtree(out)
    images_out = out / "images"
    images_out.mkdir(parents=True, exist_ok=True)

    vertices, faces = write_mesh(out / "mesh.txt")

    width = max(64, min(args.max_image_width if args.max_image_width > 0 else 256, 512))
    height = width
    fx = fy = width * 0.95
    cx = width * 0.5
    cy = height * 0.5

    count = max(2, min(args.camera_count, 12))
    radius = 260.0

    camera_lines = []
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        y = 70.0 * math.sin(angle * 0.5)
        pos = (radius * math.sin(angle), y, radius * math.cos(angle))
        R, t = look_at_world_to_camera(pos)

        name = f"frame_{i:02d}.ppm"
        render_ppm(
            images_out / name,
            vertices,
            faces,
            R,
            t,
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            i,
        )

        # Mat4f column-major, matching the original script.
        m = [
            R[0], R[3], R[6], 0.0,
            R[1], R[4], R[7], 0.0,
            R[2], R[5], R[8], 0.0,
            t[0], t[1], t[2], 1.0,
        ]
        camera_lines.append((i, name, width, height, fx, fy, cx, cy, m))

    with (out / "cameras.txt").open("w", encoding="utf-8") as f:
        f.write("PCEDITOR_TEXTURE_CAMERAS 1\n")
        f.write(f"cameras {len(camera_lines)}\n")
        for im_id, name, w, h, fx0, fy0, cx0, cy0, m in camera_lines:
            vals = " ".join(f"{x:.9g}" for x in m)
            f.write(
                f"{im_id} {name} {w} {h} "
                f"{fx0:.9g} {fy0:.9g} {cx0:.9g} {cy0:.9g} {vals}\n"
            )

    marker.write_text(
        "dataset=PCEDITOR synthetic tiny texture case\n"
        "mesh=cube_100mm\n"
        f"vertices={len(vertices)}\n"
        f"triangles={len(faces)}\n"
        f"cameras={len(camera_lines)}\n"
        f"image_size={width}x{height}\n",
        encoding="utf-8",
    )

    total_bytes = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    print(
        f"[prepare] READY tiny case: vertices={len(vertices)} "
        f"triangles={len(faces)} cameras={len(camera_lines)} "
        f"size={total_bytes / 1024.0:.1f} KiB -> {out}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
