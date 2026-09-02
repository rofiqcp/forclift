#!/usr/bin/python3
"""Prepare a planning-only occupancy map for robust Nav2 global planning.

The saved raw map remains untouched on /map for AMCL.  A second /nav_map is
created for global planning.  It:
  1) removes only tiny isolated occupied speckles in known free space; and
  2) clears the robot's *known initial footprint* plus a small configurable
     margin on the planning-only map.

Step (2) fixes a frequent mapping artefact where the robot/LiDAR support is
burned into the static map exactly under the pose from which autonomous mode is
started.  The raw localization map is never modified.

No third-party Python packages are required.
"""

import argparse
import math
import os
import re
import sys
from collections import deque


def _read_yaml_text(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _yaml_scalar(text, key, default=None):
    m = re.search(rf"^\s*{re.escape(key)}\s*:\s*(.+?)\s*$", text, flags=re.MULTILINE)
    if not m:
        return default
    return m.group(1).strip().strip("\"'")


def _yaml_list3(text, key, default=(0.0, 0.0, 0.0)):
    raw = _yaml_scalar(text, key, None)
    if raw is None:
        return tuple(default)
    m = re.match(r"^\[\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^\]]+)\s*\]$", raw)
    if not m:
        raise ValueError(f"Map YAML {key}: expected [x, y, yaw], got {raw!r}")
    return tuple(float(m.group(i)) for i in (1, 2, 3))


def _parse_pgm(path):
    data = open(path, "rb").read()
    n = len(data)
    i = 0

    def skip_ws_comments(idx):
        while idx < n:
            if data[idx] in b" \t\r\n":
                idx += 1
                continue
            if data[idx] == ord("#"):
                while idx < n and data[idx] not in b"\r\n":
                    idx += 1
                continue
            break
        return idx

    def token(idx):
        idx = skip_ws_comments(idx)
        if idx >= n:
            raise ValueError("Unexpected end of PGM header")
        start = idx
        while idx < n and data[idx] not in b" \t\r\n#":
            idx += 1
        return data[start:idx].decode("ascii"), idx

    magic, i = token(i)
    width_s, i = token(i)
    height_s, i = token(i)
    maxval_s, i = token(i)
    width = int(width_s)
    height = int(height_s)
    maxval = int(maxval_s)
    if width <= 0 or height <= 0 or not (1 <= maxval <= 255):
        raise ValueError("Unsupported PGM dimensions/maxval")

    if magic == "P5":
        if i >= n or data[i] not in b" \t\r\n":
            raise ValueError("Malformed P5 PGM header")
        if data[i] == 13 and i + 1 < n and data[i + 1] == 10:
            i += 2
        else:
            i += 1
        raster = data[i:i + width * height]
        if len(raster) != width * height:
            raise ValueError("P5 raster size does not match width*height")
        pixels = bytearray(raster)
    elif magic == "P2":
        pixels = bytearray()
        for _ in range(width * height):
            t, i = token(i)
            pixels.append(int(t))
        if len(pixels) != width * height:
            raise ValueError("P2 raster size does not match width*height")
    else:
        raise ValueError(f"Unsupported PGM magic {magic!r}; expected P5/P2")

    return width, height, maxval, pixels


def _write_pgm(path, width, height, maxval, pixels):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(
            f"P5\n# planning-only cleaned map; raw map remains unchanged\n"
            f"{width} {height}\n{maxval}\n".encode("ascii")
        )
        f.write(bytes(pixels))


def _neighbors8(idx, width, height):
    y, x = divmod(idx, width)
    for dy in (-1, 0, 1):
        ny = y + dy
        if ny < 0 or ny >= height:
            continue
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx = x + dx
            if 0 <= nx < width:
                yield ny * width + nx


def _component_has_unknown_halo(component, unknown, width, height, halo_cells):
    if halo_cells <= 0:
        return False
    for idx in component:
        y, x = divmod(idx, width)
        y0 = max(0, y - halo_cells)
        y1 = min(height - 1, y + halo_cells)
        x0 = max(0, x - halo_cells)
        x1 = min(width - 1, x + halo_cells)
        for yy in range(y0, y1 + 1):
            base = yy * width
            for xx in range(x0, x1 + 1):
                if unknown[base + xx]:
                    return True
    return False


def _clean_components(pixels, width, height, maxval, negate, occupied_thresh,
                      free_thresh, max_cells, unknown_halo_cells):
    occ = [False] * (width * height)
    unknown = [False] * (width * height)
    for idx, raw in enumerate(pixels):
        norm = raw / float(maxval)
        occ_prob = norm if negate else (1.0 - norm)
        if occ_prob > occupied_thresh:
            occ[idx] = True
        elif occ_prob >= free_thresh:
            unknown[idx] = True

    visited = [False] * len(occ)
    remove = []
    removed_components = 0
    for start in range(len(occ)):
        if not occ[start] or visited[start]:
            continue
        q = deque([start])
        visited[start] = True
        comp = []
        touches_border = False
        while q:
            cur = q.popleft()
            comp.append(cur)
            y, x = divmod(cur, width)
            if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                touches_border = True
            for nb in _neighbors8(cur, width, height):
                if occ[nb] and not visited[nb]:
                    visited[nb] = True
                    q.append(nb)
        if len(comp) > max_cells or touches_border:
            continue
        if _component_has_unknown_halo(comp, unknown, width, height, unknown_halo_cells):
            continue
        remove.extend(comp)
        removed_components += 1

    free_value = 0 if negate else maxval
    for idx in remove:
        pixels[idx] = free_value
    return len(remove), removed_components


def _clear_start_footprint(pixels, width, height, maxval, negate, free_thresh, resolution,
                           map_origin, start_x, start_y, start_yaw,
                           half_length, half_width, margin):
    """Clear cells beneath the known robot start pose on /nav_map only.

    PGM raster row 0 is the top of the map while OccupancyGrid y=0 is the
    bottom.  Each cell center is transformed map-local -> world -> robot-start.
    """
    ox, oy, oyaw = map_origin
    co = math.cos(oyaw)
    so = math.sin(oyaw)
    cs = math.cos(start_yaw)
    ss = math.sin(start_yaw)
    hx = max(0.0, float(half_length)) + max(0.0, float(margin))
    hy = max(0.0, float(half_width)) + max(0.0, float(margin))
    free_value = 0 if negate else maxval
    changed = 0
    covered = 0

    for raster_y in range(height):
        map_y = height - 1 - raster_y
        local_y = (map_y + 0.5) * resolution
        base = raster_y * width
        for map_x in range(width):
            local_x = (map_x + 0.5) * resolution
            wx = ox + co * local_x - so * local_y
            wy = oy + so * local_x + co * local_y
            dx = wx - start_x
            dy = wy - start_y
            rx = cs * dx + ss * dy
            ry = -ss * dx + cs * dy
            if abs(rx) <= hx and abs(ry) <= hy:
                covered += 1
                idx = base + map_x
                raw = pixels[idx]
                norm = raw / float(maxval)
                occ_prob = norm if negate else (1.0 - norm)
                # Leave cells that map_server already classifies as free untouched.
                # Only unknown/occupied start cells are converted to free on /nav_map.
                if occ_prob >= free_thresh:
                    pixels[idx] = free_value
                    changed += 1
    return changed, covered


def prepare(raw_yaml, output_dir, max_cells=3, unknown_halo_cells=2,
            clear_start=False, start_x=0.0, start_y=0.0, start_yaw=0.0,
            start_half_length=0.65, start_half_width=0.40, start_margin=0.10):
    raw_yaml = os.path.abspath(os.path.expanduser(raw_yaml))
    text = _read_yaml_text(raw_yaml)
    image_value = _yaml_scalar(text, "image")
    if not image_value:
        raise RuntimeError("Map YAML has no image: entry")
    image_path = image_value if os.path.isabs(image_value) else os.path.join(
        os.path.dirname(raw_yaml), image_value)
    image_path = os.path.abspath(os.path.expanduser(image_path))
    if os.path.splitext(image_path)[1].lower() != ".pgm":
        raise RuntimeError("Planning-map filter currently supports PGM map images only")

    negate = int(float(_yaml_scalar(text, "negate", "0"))) != 0
    occupied_thresh = float(_yaml_scalar(text, "occupied_thresh", "0.65"))
    free_thresh = float(_yaml_scalar(text, "free_thresh", "0.25"))
    resolution = float(_yaml_scalar(text, "resolution", "0.05"))
    map_origin = _yaml_list3(text, "origin", (0.0, 0.0, 0.0))
    if resolution <= 0.0:
        raise RuntimeError("Map resolution must be > 0")

    width, height, maxval, pixels = _parse_pgm(image_path)
    occupied_before = 0
    for raw in pixels:
        norm = raw / float(maxval)
        p = norm if negate else (1.0 - norm)
        if p > occupied_thresh:
            occupied_before += 1

    removed_cells, removed_components = _clean_components(
        pixels, width, height, maxval, negate, occupied_thresh, free_thresh,
        max_cells=max_cells, unknown_halo_cells=unknown_halo_cells)

    start_cleared_cells = 0
    start_covered_cells = 0
    if clear_start:
        start_cleared_cells, start_covered_cells = _clear_start_footprint(
            pixels, width, height, maxval, negate, free_thresh, resolution, map_origin,
            start_x, start_y, start_yaw,
            start_half_length, start_half_width, start_margin)

    base = os.path.splitext(os.path.basename(raw_yaml))[0]
    os.makedirs(output_dir, exist_ok=True)
    out_pgm = os.path.join(output_dir, base + "_navclean.pgm")
    out_yaml = os.path.join(output_dir, base + "_navclean.yaml")
    _write_pgm(out_pgm, width, height, maxval, pixels)

    replaced = re.sub(
        r"^\s*image\s*:\s*.+?$", "image: " + out_pgm, text,
        count=1, flags=re.MULTILINE)
    with open(out_yaml, "w", encoding="utf-8") as f:
        f.write(replaced)

    print(
        f"NAV_MAP={out_yaml}\n"
        f"RAW_MAP={raw_yaml}\n"
        f"OCCUPIED_BEFORE={occupied_before}\n"
        f"REMOVED_CELLS={removed_cells}\n"
        f"REMOVED_COMPONENTS={removed_components}\n"
        f"MAX_COMPONENT_CELLS={max_cells}\n"
        f"UNKNOWN_HALO_CELLS={unknown_halo_cells}\n"
        f"START_CLEAR_ENABLED={int(bool(clear_start))}\n"
        f"START_CLEARED_CELLS={start_cleared_cells}\n"
        f"START_COVERED_CELLS={start_covered_cells}\n"
        f"START_POSE={start_x:.6f},{start_y:.6f},{start_yaw:.6f}\n"
        f"START_CLEAR_HALF_EXTENT={start_half_length + start_margin:.3f},"
        f"{start_half_width + start_margin:.3f}"
    )
    return out_yaml, removed_cells, removed_components, start_cleared_cells


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-yaml", required=True)
    ap.add_argument("--output-dir", default="/tmp/navigation_autonomous_navmap")
    ap.add_argument("--max-cells", type=int, default=3)
    ap.add_argument("--unknown-halo-cells", type=int, default=2)
    ap.add_argument("--clear-start", action="store_true")
    ap.add_argument("--start-x", type=float, default=0.0)
    ap.add_argument("--start-y", type=float, default=0.0)
    ap.add_argument("--start-yaw", type=float, default=0.0)
    ap.add_argument("--start-half-length", type=float, default=0.65)
    ap.add_argument("--start-half-width", type=float, default=0.40)
    ap.add_argument("--start-margin", type=float, default=0.10)
    args = ap.parse_args()

    if args.max_cells < 0 or args.unknown_halo_cells < 0:
        ap.error("max-cells and unknown-halo-cells must be >= 0")
    if args.start_half_length <= 0 or args.start_half_width <= 0 or args.start_margin < 0:
        ap.error("start half extents must be > 0 and margin must be >= 0")

    try:
        prepare(
            args.map_yaml, args.output_dir,
            max_cells=args.max_cells,
            unknown_halo_cells=args.unknown_halo_cells,
            clear_start=args.clear_start,
            start_x=args.start_x, start_y=args.start_y, start_yaw=args.start_yaw,
            start_half_length=args.start_half_length,
            start_half_width=args.start_half_width,
            start_margin=args.start_margin,
        )
    except Exception as exc:
        print(f"[prepare_nav_map] ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
