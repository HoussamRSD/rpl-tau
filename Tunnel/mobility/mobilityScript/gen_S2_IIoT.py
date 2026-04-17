#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

"""
S2 - IIoT (factory) mobility scenario for Cooja/Contiki
Output format per line: <mote_id> <time> <x> <y>

IDs: 0..N-1  (so "mote 1" corresponds to id 0)

Scenario:
- Sink: fixed (controller / workshop server)
- Senders (N-1):
  - Fixed (65%): machine sensors (static, spread in workshop)
  - Mobile (35%): AGVs/robots moving along corridors (grid) with pauses at stations

Mobility (AGVs):
- speed: 1.2 m/s
- pause: 5..60 s (loading/unloading)
- model: constrained corridors (Manhattan-like grid) -> semi predictable

New parameters supported:
- --sink_id (default 0), --sink_pos, --sink_x, --sink_y
- strict anti-overlap each instant: --min_dist + --precision (prevents same printed coords too)
- corridor grid density: --lane_lines (aka corridor lines per axis)
- multi-lane corridors for high N: --lanes_per_corridor + --lane_width

Example:
python gen_S2_IIoT.py \
  --nodes 30 --duration 3600 --dt 2 --w 200 --h 200 \
  --out S2.dat --seed 42 \
  --sink_id 0 --sink_pos center \
  --fixed_senders_ratio 0.65 \
  --lane_lines 6 \
  --lanes_per_corridor 2 --lane_width 2.0 \
  --agv_speed 1.2 --pause_min 5 --pause_max 60 \
  --turn_prob 0.30 \
  --min_dist 2.0 --precision 3
"""

import argparse
import random
import math

# -----------------------------
# Utils
# -----------------------------
def clamp(v, vmin, vmax):
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v

def dist2(ax, ay, bx, by):
    dx = ax - bx
    dy = ay - by
    return dx*dx + dy*dy

def qkey(x, y, prec):
    return (round(x, prec), round(y, prec))

def ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
    # Avoid same printed coordinates AND enforce minimum distance
    if qkey(x, y, prec) in occ_q:
        return False
    for (ox, oy) in occ_xy:
        if dist2(x, y, ox, oy) < min_dist2:
            return False
    return True

def add_occ(x, y, occ_xy, occ_q, prec):
    occ_xy.append((x, y))
    occ_q.add(qkey(x, y, prec))

def fmt_line(mid, t, x, y, prec):
    return ("%d %d %." + str(prec) + "f %." + str(prec) + "f\n") % (mid, t, x, y)

def make_lines(n_lines, max_coord, margin):
    # evenly spaced in [margin, max_coord - margin]
    if n_lines < 2:
        return [margin, max_coord - margin]
    span = float(max_coord - 2.0*margin)
    if span <= 0:
        return [0.0, float(max_coord)]
    step = span / float(n_lines - 1)
    lines = []
    i = 0
    while i < n_lines:
        lines.append(margin + i * step)
        i += 1
    return lines

def nearest_line(v, lines):
    best = lines[0]
    bestd = abs(v - best)
    for g in lines[1:]:
        d = abs(v - g)
        if d < bestd:
            bestd = d
            best = g
    return best

def near_line(v, lines, eps):
    best = nearest_line(v, lines)
    return (abs(v - best) <= eps), best

def lane_offsets(lanes_per_corridor, lane_width):
    if lanes_per_corridor <= 1:
        return [0.0]
    offs = []
    mid = (lanes_per_corridor - 1) / 2.0
    i = 0
    while i < lanes_per_corridor:
        offs.append((i - mid) * lane_width)
        i += 1
    return offs

def sink_xy(pos, W, H, x_lines, y_lines, custom_x, custom_y):
    if pos == "custom":
        sx = clamp(custom_x, 0.0, W)
        sy = clamp(custom_y, 0.0, H)
        return (nearest_line(sx, x_lines), nearest_line(sy, y_lines))
    if pos == "center":
        return (nearest_line(W/2.0, x_lines), nearest_line(H/2.0, y_lines))
    if pos == "left":
        return (x_lines[0], nearest_line(H/2.0, y_lines))
    if pos == "right":
        return (x_lines[-1], nearest_line(H/2.0, y_lines))
    if pos == "top":
        return (nearest_line(W/2.0, x_lines), y_lines[-1])
    if pos == "bottom":
        return (nearest_line(W/2.0, x_lines), y_lines[0])
    if pos == "topleft":
        return (x_lines[0], y_lines[-1])
    if pos == "topright":
        return (x_lines[-1], y_lines[-1])
    if pos == "bottomleft":
        return (x_lines[0], y_lines[0])
    if pos == "bottomright":
        return (x_lines[-1], y_lines[0])
    return (nearest_line(W/2.0, x_lines), nearest_line(H/2.0, y_lines))

def sample_free_rect(xmin, xmax, ymin, ymax, occ_xy, occ_q, min_dist2, prec, max_tries=500000):
    for _ in range(max_tries):
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        if ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
            return (x, y)
    return (x, y)

def sample_free_corridor_point(W, H, x_lines, y_lines, off, occ_xy, occ_q, min_dist2, prec, max_tries=500000):
    # pick a point on corridors (either horizontal or vertical)
    for _ in range(max_tries):
        if random.random() < 0.5:
            y0 = random.choice(y_lines)
            y = clamp(y0 + off, 0.0, H)
            x = random.uniform(0.0, W)
        else:
            x0 = random.choice(x_lines)
            x = clamp(x0 + off, 0.0, W)
            y = random.uniform(0.0, H)
        if ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
            return (x, y)
    return (x, y)

def build_stations(x_lines, y_lines, off, W, H):
    # stations = intersections on this offset lane
    st = []
    for x in x_lines:
        for y in y_lines:
            px = x + off
            py = y + off
            if 0.0 <= px <= W and 0.0 <= py <= H:
                st.append((px, py))
    # add extra stations along lines to increase variety
    xs = [0.2*W, 0.5*W, 0.8*W]
    ys = [0.2*H, 0.5*H, 0.8*H]
    for y in y_lines:
        py = clamp(y + off, 0.0, H)
        for px in xs:
            st.append((clamp(px, 0.0, W), py))
    for x in x_lines:
        px = clamp(x + off, 0.0, W)
        for py in ys:
            st.append((px, clamp(py, 0.0, H)))
    return st

# -----------------------------
# Main
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nodes", type=int, required=True, help="Total nodes (IDs 0..N-1)")
    ap.add_argument("--duration", type=int, default=3600)
    ap.add_argument("--dt", type=int, default=2)
    ap.add_argument("--w", type=float, default=300.0)
    ap.add_argument("--h", type=float, default=300.0)
    ap.add_argument("--out", type=str, default="S2.dat")
    ap.add_argument("--seed", type=int, default=42)

    # Sink
    ap.add_argument("--sink_id", type=int, default=0)
    ap.add_argument("--sink_pos", type=str, default="center",
                    choices=["center","left","right","top","bottom",
                             "topleft","topright","bottomleft","bottomright","custom"])
    ap.add_argument("--sink_x", type=float, default=150.0)
    ap.add_argument("--sink_y", type=float, default=150.0)

    # Mix
    ap.add_argument("--fixed_senders_ratio", type=float, default=0.65)

    # Corridors (grid lines per axis)
    ap.add_argument("--lane_lines", "--corridor_lines", dest="lane_lines", type=int, default=10)

    # Multi-lane corridors (helps for large N)
    ap.add_argument("--lanes_per_corridor", type=int, default=2)
    ap.add_argument("--lane_width", type=float, default=2.0)

    # AGV mobility
    ap.add_argument("--agv_speed", type=float, default=1.2)
    ap.add_argument("--pause_min", type=int, default=5)
    ap.add_argument("--pause_max", type=int, default=60)
    ap.add_argument("--turn_prob", type=float, default=0.30)

    # Anti-overlap
    ap.add_argument("--min_dist", type=float, default=2.0)
    ap.add_argument("--precision", type=int, default=3)

    args = ap.parse_args()
    random.seed(args.seed)

    N = args.nodes
    T = args.duration
    dt = args.dt
    W = args.w
    H = args.h
    prec = args.precision

    if N < 2:
        raise SystemExit("Error: --nodes must be >= 2")
    if args.sink_id < 0 or args.sink_id > (N - 1):
        raise SystemExit("Error: --sink_id must be in [0..N-1]")
    if dt <= 0:
        raise SystemExit("Error: --dt must be > 0")
    if args.lane_lines < 2:
        raise SystemExit("Error: --lane_lines must be >= 2")
    if args.lanes_per_corridor < 1:
        args.lanes_per_corridor = 1
    if args.min_dist < 0:
        raise SystemExit("Error: --min_dist must be >= 0")

    # Anti-overlap
    min_dist2 = args.min_dist * args.min_dist

    # Lanes offsets
    lane_offs = lane_offsets(args.lanes_per_corridor, args.lane_width)
    max_off = 0.0
    for o in lane_offs:
        if abs(o) > max_off:
            max_off = abs(o)

    # Margin so lanes + min_dist remain inside
    margin = max_off + args.min_dist + 1.0

    # Corridor center lines inside margin
    x_lines = make_lines(args.lane_lines, W, margin)
    y_lines = make_lines(args.lane_lines, H, margin)

    # Deterministic sink (snap to intersection)
    sx, sy = sink_xy(args.sink_pos, W, H, x_lines, y_lines, args.sink_x, args.sink_y)

    # Split roles (exclude sink)
    ids = list(range(0, N))
    ids.remove(args.sink_id)
    random.shuffle(ids)

    senders = N - 1
    n_fixed = int(round(senders * clamp(args.fixed_senders_ratio, 0.0, 1.0)))
    n_fixed = max(0, min(senders, n_fixed))
    n_agv = senders - n_fixed

    fixed_ids = set(ids[:n_fixed])
    agv_ids = set(ids[n_fixed:])

    nodes = {}
    nodes[args.sink_id] = {"type": "sink", "x": sx, "y": sy}

    # Initial occupancy
    occ0_xy = []
    occ0_q = set()
    add_occ(sx, sy, occ0_xy, occ0_q, prec)

    # -----------------------------
    # Place fixed machine sensors (spread in workshop)
    # Use grid+jitter placement to avoid clustering
    # -----------------------------
    fixed_list = list(fixed_ids)
    random.shuffle(fixed_list)

    if n_fixed > 0:
        gx = int(math.sqrt(n_fixed))
        if gx < 2:
            gx = 2
        gy = int(math.ceil(float(n_fixed) / float(gx)))
        if gy < 2:
            gy = 2

        stepx = (W - 2.0*margin) / float(gx - 1) if gx > 1 else 0.0
        stepy = (H - 2.0*margin) / float(gy - 1) if gy > 1 else 0.0

        grid_points = []
        i = 0
        while i < gx:
            j = 0
            while j < gy:
                px = margin + i * stepx
                py = margin + j * stepy
                grid_points.append((px, py))
                j += 1
            i += 1

        random.shuffle(grid_points)
        k = 0

        for mid in fixed_list:
            placed = False
            tries = 0
            while (not placed) and tries < 500000:
                tries += 1
                if k < len(grid_points):
                    px, py = grid_points[k]
                    k += 1
                else:
                    px = random.uniform(margin, W - margin)
                    py = random.uniform(margin, H - margin)

                # jitter
                jx = clamp(px + random.uniform(-1.5, 1.5), margin, W - margin)
                jy = clamp(py + random.uniform(-1.5, 1.5), margin, H - margin)

                if ok_pos(jx, jy, occ0_xy, occ0_q, min_dist2, prec):
                    nodes[mid] = {"type": "fixed", "x": jx, "y": jy}
                    add_occ(jx, jy, occ0_xy, occ0_q, prec)
                    placed = True

            if not placed:
                raise SystemExit("Placement failed for FIXED nodes. Increase area (--w/--h) or reduce --min_dist.")

    # -----------------------------
    # Prepare stations per lane offset (AGV waypoints)
    # -----------------------------
    stations_by_off = {}
    for off in lane_offs:
        stations_by_off[off] = build_stations(x_lines, y_lines, off, W, H)

    # -----------------------------
    # Place AGVs on corridors + set targets
    # -----------------------------
    for mid in agv_ids:
        off = random.choice(lane_offs)
        ax, ay = sample_free_corridor_point(W, H, x_lines, y_lines, off, occ0_xy, occ0_q, min_dist2, prec)
        add_occ(ax, ay, occ0_xy, occ0_q, prec)

        st = stations_by_off[off]
        tx, ty = random.choice(st)
        # ensure different target
        tries = 0
        while dist2(tx, ty, ax, ay) < 1e-6 and tries < 1000:
            tx, ty = random.choice(st)
            tries += 1

        nodes[mid] = {
            "type": "agv",
            "x": ax, "y": ay,
            "tx": tx, "ty": ty,
            "off": off,
            "pause": random.randint(0, max(0, args.pause_min)) // max(1, dt),
            "axis": "x" if random.random() < 0.5 else "y"
        }

    # Movement per step
    step = max(0.0, args.agv_speed * dt)
    eps = max(0.8, step * 0.35)

    def pick_new_target(n):
        off = n["off"]
        st = stations_by_off[off]
        tx, ty = random.choice(st)
        tries = 0
        while dist2(tx, ty, n["x"], n["y"]) < 1e-6 and tries < 1000:
            tx, ty = random.choice(st)
            tries += 1
        n["tx"], n["ty"] = tx, ty
        n["axis"] = "x" if random.random() < 0.5 else "y"

    def propose_agv_move(n, factor):
        """
        Move on corridors with offset-lanes.
        - if moving along x: keep y on nearest (y_line + off), x moves continuous and snaps at x intersections
        - if moving along y: keep x on nearest (x_line + off), y moves continuous and snaps at y intersections
        """
        ox, oy = n["x"], n["y"]
        tx, ty = n["tx"], n["ty"]
        off = n["off"]

        # reached target?
        if abs(ox - tx) <= eps and abs(oy - ty) <= eps:
            return (tx, ty, True)

        # decide axis (semi-predictable)
        need_x = abs(ox - tx) > eps
        need_y = abs(oy - ty) > eps

        if need_x and need_y:
            # at "intersection" -> maybe turn choice
            if random.random() < args.turn_prob:
                n["axis"] = "y" if n["axis"] == "x" else "x"
        else:
            if need_x:
                n["axis"] = "x"
            elif need_y:
                n["axis"] = "y"

        nx, ny = ox, oy
        dstep = step * factor

        if n["axis"] == "x" and need_x:
            # keep y on corridor lane: y = nearest y_line + off
            y0 = nearest_line(oy - off, y_lines) + off
            ny = clamp(y0, 0.0, H)

            if ox < tx:
                nx = clamp(ox + dstep, 0.0, W)
                if nx > tx:
                    nx = tx
            else:
                nx = clamp(ox - dstep, 0.0, W)
                if nx < tx:
                    nx = tx

            # snap to x intersections if close
            hit, xhit = near_line(nx - off, x_lines, eps)
            if hit:
                nx = clamp(xhit + off, 0.0, W)

        elif n["axis"] == "y" and need_y:
            # keep x on corridor lane: x = nearest x_line + off
            x0 = nearest_line(ox - off, x_lines) + off
            nx = clamp(x0, 0.0, W)

            if oy < ty:
                ny = clamp(oy + dstep, 0.0, H)
                if ny > ty:
                    ny = ty
            else:
                ny = clamp(oy - dstep, 0.0, H)
                if ny < ty:
                    ny = ty

            # snap to y intersections if close
            hit, yhit = near_line(ny - off, y_lines, eps)
            if hit:
                ny = clamp(yhit + off, 0.0, H)

        else:
            # cannot progress (already aligned) -> stay
            nx, ny = ox, oy

        arrived = (abs(nx - tx) <= eps and abs(ny - ty) <= eps)
        if arrived:
            nx, ny = tx, ty
        return (nx, ny, arrived)

    # -----------------------------
    # Simulation loop with strict no-overlap + fixed preservation
    # -----------------------------
    with open(args.out, "w") as f:
        t = 0
        while t <= T:
            # write all nodes 0..N-1
            for mid in range(0, N):
                n = nodes[mid]
                f.write(fmt_line(mid, t, n["x"], n["y"], prec))

            if t == T:
                break

            # reserve sink + fixed
            occ_xy = []
            occ_q = set()
            for mid in range(0, N):
                if nodes[mid]["type"] in ("sink", "fixed"):
                    add_occ(nodes[mid]["x"], nodes[mid]["y"], occ_xy, occ_q, prec)

            # update AGVs in random order to reduce bias
            agv_list = list(agv_ids)
            random.shuffle(agv_list)

            for mid in agv_list:
                n = nodes[mid]
                ox, oy = n["x"], n["y"]

                if n["pause"] > 0:
                    n["pause"] -= 1
                    add_occ(ox, oy, occ_xy, occ_q, prec)
                    continue

                # propose move with collision-safe fallback
                candidates = []
                nx, ny, arrived = propose_agv_move(n, 1.0)
                candidates.append((nx, ny, arrived))
                nx2, ny2, arrived2 = propose_agv_move(n, 0.5)
                candidates.append((nx2, ny2, arrived2))
                nx3, ny3, arrived3 = propose_agv_move(n, 0.25)
                candidates.append((nx3, ny3, arrived3))
                candidates.append((ox, oy, False))

                chosen = (ox, oy, False)
                for (cx, cy, carr) in candidates:
                    cx = clamp(cx, 0.0, W)
                    cy = clamp(cy, 0.0, H)
                    if ok_pos(cx, cy, occ_xy, occ_q, min_dist2, prec):
                        chosen = (cx, cy, carr)
                        break

                n["x"], n["y"] = chosen[0], chosen[1]
                add_occ(n["x"], n["y"], occ_xy, occ_q, prec)

                if chosen[2]:
                    # reached station -> pause then new target
                    ps = random.randint(args.pause_min, args.pause_max)
                    n["pause"] = ps // max(1, dt)
                    pick_new_target(n)

            t += dt

    print("[OK] Generated:", args.out)
    print("  IDs: 0..%d (sink_id=%d)" % (N-1, args.sink_id))
    print("  fixed=%d, agv=%d" % (n_fixed, n_agv))
    print("  sink_pos=%s, min_dist=%.2f, precision=%d" % (args.sink_pos, args.min_dist, prec))
    print("  corridors=%d lines/axis, lanes=%d, lane_width=%.2f" % (args.lane_lines, args.lanes_per_corridor, args.lane_width))
    print("  agv_speed=%.2f m/s, pause=%d..%ds" % (args.agv_speed, args.pause_min, args.pause_max))
    print("  area=%.0fx%.0f, dt=%ds, T=%ds" % (W, H, dt, T))

if __name__ == "__main__":
    main()

