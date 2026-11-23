#!/usr/bin/env python3
"""Build a bounded binary PCD for RViz without changing map coordinates."""

import argparse
import hashlib
import heapq
import json
import os
from pathlib import Path
import tempfile

import numpy as np


PCD_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("rgb", "<f4"), ("region_id", "<u4"),
])
MULTIPLIERS = np.array([
    0x9E3779B185EBCA87, 0xC2B2AE3D27D4EB4F,
    0x165667B19E3779F9,
], dtype=np.uint64)


def read_header(path):
    values = {}
    with path.open("rb") as stream:
        for _ in range(100):
            raw = stream.readline(4096)
            if not raw or len(raw) == 4096 and not raw.endswith(b"\n"):
                raise ValueError("无效的 PCD 文件头")
            if raw.startswith(b"#") or not raw.strip():
                continue
            parts = raw.decode("ascii").split()
            values[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                if values.get("FIELDS") != ["x", "y", "z", "rgb", "region_id"] or \
                   values.get("SIZE") != ["4"] * 5 or \
                   values.get("TYPE") != ["F", "F", "F", "F", "U"] or \
                   values.get("DATA") != ["binary"]:
                    raise ValueError("当前脚本需要 building_split_space 的二进制 XYZ/RGB/region_id 格式")
                count = int(values["POINTS"][0])
                return count, stream.tell()
    raise ValueError("缺少 PCD DATA 字段")


def voxel_hash(voxels):
    with np.errstate(over="ignore"):
        bits = voxels.astype(np.uint64)
        ranks = (bits * MULTIPLIERS).sum(axis=1, dtype=np.uint64)
        ranks ^= ranks >> np.uint64(33)
        ranks *= np.uint64(0xff51afd7ed558ccd)
        ranks ^= ranks >> np.uint64(33)
    return ranks


def build(source, output, voxel_size, max_points, ceiling_z, chunk_size):
    if source.resolve() == output.resolve():
        raise ValueError("输出不能覆盖定位和规划使用的原始地图")
    if not np.isfinite(voxel_size) or voxel_size <= 0 or max_points < 1 or chunk_size < 1:
        raise ValueError("体素大小、点数限制和块大小必须为正")
    if not np.isfinite(ceiling_z):
        raise ValueError("天花板高度必须是有限数值")
    count, offset = read_header(source)
    if source.stat().st_size < offset + count * PCD_DTYPE.itemsize:
        raise ValueError("原始地图数据不完整")
    points = np.memmap(source, dtype=PCD_DTYPE, mode="r", offset=offset, shape=(count,))
    chosen = {}
    heap = []
    removed_ceiling = 0
    for start in range(0, count, chunk_size):
        chunk = points[start:start + chunk_size]
        roof = (chunk["region_id"] == 10) & (chunk["z"] >= ceiling_z)
        removed_ceiling += int(np.count_nonzero(roof))
        finite = np.isfinite(chunk["x"]) & np.isfinite(chunk["y"]) & np.isfinite(chunk["z"])
        valid = chunk[finite & ~roof]
        if len(valid) == 0:
            continue
        xyz = np.column_stack((valid["x"], valid["y"], valid["z"]))
        coords = np.floor(xyz.astype(np.float64) / voxel_size).astype(np.int64)
        voxels, first = np.unique(coords, axis=0, return_index=True)
        ranks = voxel_hash(voxels)
        for voxel, index, rank in zip(voxels, first, ranks):
            key = tuple(int(value) for value in voxel)
            if key in chosen:
                continue
            rank = int(rank)
            if len(heap) >= max_points and rank >= -heap[0][0]:
                continue
            if len(heap) >= max_points:
                _, old = heapq.heappop(heap)
                del chosen[old]
            chosen[key] = valid[index].tobytes()
            heapq.heappush(heap, (-rank, key))
    if not chosen:
        raise ValueError("去顶和采样后没有可显示点")
    output.parent.mkdir(parents=True, exist_ok=True)
    header = ("# Visualization only; coordinates and point attributes copied from source\n"
              "VERSION 0.7\nFIELDS x y z rgb region_id\nSIZE 4 4 4 4 4\n"
              "TYPE F F F F U\nCOUNT 1 1 1 1 1\n"
              f"WIDTH {len(chosen)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(chosen)}\nDATA binary\n")
    with tempfile.NamedTemporaryFile(dir=output.parent, prefix=output.name + ".",
                                     suffix=".tmp", delete=False) as stream:
        temp = Path(stream.name)
        try:
            stream.write(header.encode("ascii"))
            for key in sorted(chosen):
                stream.write(chosen[key])
            stream.flush()
            os.fsync(stream.fileno())
            os.replace(temp, output)
            output.chmod(0o644)
        finally:
            temp.unlink(missing_ok=True)
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    manifest = {
        "source": str(source.resolve()), "source_sha256": digest.hexdigest(),
        "source_points": count, "visual_points": len(chosen),
        "ceiling_region_id": 10, "ceiling_z_m": ceiling_z,
        "removed_ceiling_points": removed_ceiling,
        "voxel_size_m": voxel_size, "max_points": max_points,
        "frame_id": "map", "coordinate_transform": "none",
    }
    output.with_suffix(".json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--voxel-size", type=float, default=0.20)
    parser.add_argument("--max-points", type=int, default=300000)
    parser.add_argument("--space10-ceiling-z", type=float, default=11.55)
    parser.add_argument("--chunk-size", type=int, default=1000000)
    args = parser.parse_args()
    build(args.source.expanduser(), args.output.expanduser(), args.voxel_size,
          args.max_points, args.space10_ceiling_z, args.chunk_size)


if __name__ == "__main__":
    main()
