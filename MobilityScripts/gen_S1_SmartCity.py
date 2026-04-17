#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

"""
S1 - Smart City mobility scenario for Cooja/Contiki
Output format per line: <mote_id> <time> <x> <y>

IDs: 0..N-1 (so "mote 1" corresponds to id 0 in this file)

Scenario:
- Sink: fixed (urban gateway / edge)
- Senders mix among (N-1):
  - Fixed devices (30%): lamp posts, parking, traffic lights
  - Mobile (70%): pedestrians (50%) + urban vehicles (20%)

Mobility:
- Pedestrians: Random Walk
  speed 1 m/s, pause 10..60s
- Vehicles: constrained Manhattan/Grid (roads) with stops (traffic lights / bus stops)
  speed 8 m/s, pause 5..20s, turn at intersections

New parameters supported:
- --sink_id (default 0), --sink_pos, --sink_x/y
- strict anti-overlap: --min_dist + --precision (prevents same printed coords too)
- vehicles multi-lane: --lanes_per_road + --lane_width
- --street_lines controls road grid density

Example (100 nodes):
python gen_S1_SmartCity.py \
  --nodes 15 --duration 3600 --dt 2 --w 300 --h 300 \
  --out S1_100.dat --seed 42 \
  --sink_id 0 --sink_pos center \
  --fixed_ratio 0.30 --ped_ratio 0.50 --veh_ratio 0.20 \
  --ped_speed 1.0 --ped_pause_min 10 --ped_pause_max 60 --ped_pause_prob 0.20 \
  --veh_speed 8.0 --turn_prob 0.35 --pause_prob 0.35 --light_pause_min 5 --light_pause_max 20 \
  --street_lines 16 --lanes_per_road 3 --lane_width 3.0 \
  --min_dist 8 --precision 3
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
    # dynamic precision output
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

def lane_offsets(lanes_per_road, lane_width):
    if lanes_per_road <= 1:
        return [0.0]
    offs = []
    mid = (lanes_per_road - 1) / 2.0
    i = 0
    while i < lanes_per_road:
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

def sample_free_rect(xmin, xmax, ymin, ymax, occ_xy, occ_q, min_dist2, prec, max_tries=300000):
    for _ in range(max_tries):
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        if ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
            return (x, y)
    return (x, y)

def sample_free_intersection(x_lines, y_lines, occ_xy, occ_q, min_dist2, prec, max_tries=300000):
    for _ in range(max_tries):
        x = random.choice(x_lines)
        y = random.choice(y_lines)
        if ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
            return (x, y)
    return (x, y)

def sample_free_lane_point(W, H, x_lines, y_lines, lane_offs, occ_xy, occ_q, min_dist2, prec, max_tries=300000):
    for _ in range(max_tries):
        off = random.choice(lane_offs)
        if random.random() < 0.5:
            # horizontal road: y = street + off, x continuous
            y0 = random.choice(y_lines)
            y = clamp(y0 + off, 0.0, H)
            x = random.uniform(0.0, W)
        else:
            # vertical road: x = street + off, y continuous
            x0 = random.choice(x_lines)
            x = clamp(x0 + off, 0.0, W)
            y = random.uniform(0.0, H)

        if ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
            return (x, y, off)
    return (x, y, off)

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
    ap.add_argument("--out", type=str, default="S1.dat")
    ap.add_argument("--seed", type=int, default=42)

    # Sink
    ap.add_argument("--sink_id", type=int, default=0)
    ap.add_argument("--sink_pos", type=str, default="center",
                    choices=["center","left","right","top","bottom",
                             "topleft","topright","bottomleft","bottomright","custom"])
    ap.add_argument("--sink_x", type=float, default=150.0)
    ap.add_argument("--sink_y", type=float, default=150.0)

    # Ratios among senders (N-1)
    ap.add_argument("--fixed_ratio", type=float, default=0.30)
    ap.add_argument("--ped_ratio", type=float, default=0.50)
    ap.add_argument("--veh_ratio", type=float, default=0.20)

    # Pedestrians (Random Walk)
    ap.add_argument("--ped_speed", type=float, default=1.0)
    ap.add_argument("--ped_pause_min", type=int, default=10)
    ap.add_argument("--ped_pause_max", type=int, default=60)
    ap.add_argument("--ped_pause_prob", type=float, default=0.20)

    # Vehicles (Manhattan/Grid)
    ap.add_argument("--veh_speed", type=float, default=8.0)
    ap.add_argument("--turn_prob", type=float, default=0.35)
    ap.add_argument("--pause_prob", type=float, default=0.35)
    ap.add_argument("--light_pause_min", type=int, default=5)
    ap.add_argument("--light_pause_max", type=int, default=20)

    ap.add_argument("--street_lines", type=int, default=16)
    ap.add_argument("--lanes_per_road", type=int, default=3)
    ap.add_argument("--lane_width", type=float, default=3.0)

    # Anti-overlap
    ap.add_argument("--min_dist", type=float, default=8.0)
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
    if args.street_lines < 2:
        raise SystemExit("Error: --street_lines must be >= 2")
    if args.lanes_per_road < 1:
        args.lanes_per_road = 1

    # Normalize ratios
    s = args.fixed_ratio + args.ped_ratio + args.veh_ratio
    if s <= 0:
        raise SystemExit("Error: ratios sum must be > 0")
    fixed_ratio = args.fixed_ratio / s
    ped_ratio = args.ped_ratio / s
    # veh implied

    min_dist2 = args.min_dist * args.min_dist

    # Lanes offsets
    lane_offs = lane_offsets(args.lanes_per_road, args.lane_width)
    max_off = 0.0
    for o in lane_offs:
        if abs(o) > max_off:
            max_off = abs(o)

    # Margin so lanes + min_dist stay inside area
    margin = max_off + args.min_dist + 1.0

    # Build streets (inside margin)
    x_lines = make_lines(args.street_lines, W, margin)
    y_lines = make_lines(args.street_lines, H, margin)

    # If lane width too large compared to street spacing, auto-scale down
    span = float(W - 2.0*margin)
    if args.street_lines >= 2 and span > 0 and args.lanes_per_road > 1:
        street_step = span / float(args.street_lines - 1)
        if max_off > 0.45 * street_step:
            half = (args.lanes_per_road - 1) / 2.0
            if half > 0:
                args.lane_width = (0.45 * street_step) / float(half)
                lane_offs = lane_offsets(args.lanes_per_road, args.lane_width)
                max_off = 0.0
                for o in lane_offs:
                    if abs(o) > max_off:
                        max_off = abs(o)
                margin = max_off + args.min_dist + 1.0
                x_lines = make_lines(args.street_lines, W, margin)
                y_lines = make_lines(args.street_lines, H, margin)

    # Sink fixed deterministic (snap to intersection)
    sx, sy = sink_xy(args.sink_pos, W, H, x_lines, y_lines, args.sink_x, args.sink_y)

    # Assign IDs (exclude sink)
    ids = list(range(0, N))
    ids.remove(args.sink_id)
    random.shuffle(ids)

    senders = N - 1
    n_fixed = int(round(senders * fixed_ratio))
    n_ped = int(round(senders * ped_ratio))
    n_fixed = max(0, min(senders, n_fixed))
    n_ped = max(0, min(senders - n_fixed, n_ped))
    n_veh = senders - n_fixed - n_ped

    fixed_ids = set(ids[:n_fixed])
    ped_ids = set(ids[n_fixed:n_fixed + n_ped])
    veh_ids = set(ids[n_fixed + n_ped:])

    nodes = {}
    nodes[args.sink_id] = {"type": "sink", "x": sx, "y": sy}

    # Initial occupancy
    occ0_xy = []
    occ0_q = set()
    add_occ(sx, sy, occ0_xy, occ0_q, prec)

    # ---- Place fixed devices (grid-ish + jitter) to avoid stacking
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
            while (not placed) and tries < 200000:
                if k >= len(grid_points):
                    px = random.uniform(margin, W - margin)
                    py = random.uniform(margin, H - margin)
                else:
                    px, py = grid_points[k]
                k += 1
                tries += 1

                # small jitter
                jx = clamp(px + random.uniform(-1.0, 1.0), margin, W - margin)
                jy = clamp(py + random.uniform(-1.0, 1.0), margin, H - margin)

                if ok_pos(jx, jy, occ0_xy, occ0_q, min_dist2, prec):
                    nodes[mid] = {"type": "fixed", "x": jx, "y": jy}
                    add_occ(jx, jy, occ0_xy, occ0_q, prec)
                    placed = True

            if not placed:
                # fallback (very dense case)
                nodes[mid] = {"type": "fixed", "x": px, "y": py}
                add_occ(px, py, occ0_xy, occ0_q, prec)

    # ---- Place pedestrians (free plane) with anti-overlap
    for mid in ped_ids:
        px, py = sample_free_rect(margin, W - margin, margin, H - margin,
                                  occ0_xy, occ0_q, min_dist2, prec)
        add_occ(px, py, occ0_xy, occ0_q, prec)
        nodes[mid] = {
            "type": "ped",
            "x": px, "y": py,
            "pause": random.randint(args.ped_pause_min, args.ped_pause_max) // max(1, dt)
        }

    # ---- Place vehicles on road lanes with anti-overlap
    dirs = [(1,0), (-1,0), (0,1), (0,-1)]
    for mid in veh_ids:
        vx, vy, off = sample_free_lane_point(W, H, x_lines, y_lines, lane_offs,
                                             occ0_xy, occ0_q, min_dist2, prec)
        add_occ(vx, vy, occ0_xy, occ0_q, prec)

        dx, dy = random.choice(dirs)

        # enforce road constraint at start
        if dx != 0:
            y0 = nearest_line(vy, y_lines)
            vy = clamp(y0 + off, 0.0, H)
        else:
            x0 = nearest_line(vx, x_lines)
            vx = clamp(x0 + off, 0.0, W)

        nodes[mid] = {
            "type": "veh",
            "x": vx, "y": vy,
            "dir": (dx, dy),
            "lane_off": off,
            "pause": random.randint(0, max(1, 5 // max(1, dt)))
        }

    # Movement per step
    ped_step = max(0.0, args.ped_speed * dt)
    veh_step = max(0.0, args.veh_speed * dt)

    # intersection detection tolerance
    eps = max(1.0, veh_step * 0.35)

    def traffic_pause(n):
        if random.random() < args.pause_prob:
            ps = random.randint(args.light_pause_min, args.light_pause_max)
            n["pause"] = ps // max(1, dt)

    def maybe_turn(n):
        dx, dy = n["dir"]
        if random.random() < args.turn_prob:
            if dx != 0:
                n["dir"] = random.choice([(dx, 0), (0, 1), (0, -1)])
            else:
                n["dir"] = random.choice([(0, dy), (1, 0), (-1, 0)])

    # -----------------------------
    # Simulation loop with strict no-overlap
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

            # update mobiles in mixed random order
            mobiles = list(ped_ids) + list(veh_ids)
            random.shuffle(mobiles)

            for mid in mobiles:
                n = nodes[mid]
                ox, oy = n["x"], n["y"]

                if n["pause"] > 0:
                    n["pause"] -= 1
                    add_occ(ox, oy, occ_xy, occ_q, prec)
                    continue

                # propose move
                nx, ny = ox, oy
                at_inter = False

                if n["type"] == "ped":
                    ang = random.uniform(0.0, 2.0 * math.pi)
                    nx = clamp(ox + ped_step * math.cos(ang), margin, W - margin)
                    ny = clamp(oy + ped_step * math.sin(ang), margin, H - margin)

                    # sometimes pause
                    if random.random() < clamp(args.ped_pause_prob, 0.0, 1.0):
                        n["pause"] = random.randint(args.ped_pause_min, args.ped_pause_max) // max(1, dt)

                elif n["type"] == "veh":
                    dx, dy = n["dir"]
                    off = n["lane_off"]

                    nx = clamp(ox + dx * veh_step, 0.0, W)
                    ny = clamp(oy + dy * veh_step, 0.0, H)

                    # road constraint + detect intersections
                    if dx != 0:
                        y0 = nearest_line(ny, y_lines)
                        ny = clamp(y0 + off, 0.0, H)
                        hit, xhit = near_line(nx, x_lines, eps)
                        if hit:
                            nx = xhit
                            at_inter = True
                    else:
                        x0 = nearest_line(nx, x_lines)
                        nx = clamp(x0 + off, 0.0, W)
                        hit, yhit = near_line(ny, y_lines, eps)
                        if hit:
                            ny = yhit
                            at_inter = True

                # collision avoidance: full, half, quarter, else stay
                candidates = [
                    (nx, ny),
                    (ox + 0.5*(nx-ox), oy + 0.5*(ny-oy)),
                    (ox + 0.25*(nx-ox), oy + 0.25*(ny-oy)),
                    (ox, oy),
                ]

                chosen = (ox, oy)
                for (cx, cy) in candidates:
                    cx = clamp(cx, 0.0, W)
                    cy = clamp(cy, 0.0, H)

                    # re-apply constraints for vehicles
                    if n["type"] == "veh":
                        dx, dy = n["dir"]
                        off = n["lane_off"]
                        if dx != 0:
                            y0 = nearest_line(cy, y_lines)
                            cy = clamp(y0 + off, 0.0, H)
                        else:
                            x0 = nearest_line(cx, x_lines)
                            cx = clamp(x0 + off, 0.0, W)
                    else:
                        # pedestrians stay inside margin
                        cx = clamp(cx, margin, W - margin)
                        cy = clamp(cy, margin, H - margin)

                    if ok_pos(cx, cy, occ_xy, occ_q, min_dist2, prec):
                        chosen = (cx, cy)
                        break

                n["x"], n["y"] = chosen
                add_occ(n["x"], n["y"], occ_xy, occ_q, prec)

                # vehicle intersection behavior
                if n["type"] == "veh" and at_inter and chosen != (ox, oy):
                    traffic_pause(n)
                    maybe_turn(n)

                # border bounce for vehicles
                if n["type"] == "veh":
                    dx, dy = n["dir"]
                    if n["x"] <= 0.0 or n["x"] >= W or n["y"] <= 0.0 or n["y"] >= H:
                        n["dir"] = (-dx, -dy)

            t += dt

    print("[OK] Generated:", args.out)
    print("  IDs: 0..%d (sink_id=%d)" % (N-1, args.sink_id))
    print("  fixed=%d, pedestrians=%d, vehicles=%d" % (n_fixed, n_ped, n_veh))
    print("  sink_pos=%s, min_dist=%.2f, precision=%d" % (args.sink_pos, args.min_dist, prec))
    print("  streets=%d, lanes=%d, lane_width=%.2f" % (args.street_lines, args.lanes_per_road, args.lane_width))
    print("  area=%.0fx%.0f, dt=%ds, T=%ds" % (W, H, dt, T))

if __name__ == "__main__":
    main()

