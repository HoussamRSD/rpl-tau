#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

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
    # prevent same printed coordinates + keep min distance
    if qkey(x, y, prec) in occ_q:
        return False
    for (ox, oy) in occ_xy:
        if dist2(x, y, ox, oy) < min_dist2:
            return False
    return True

def add_occ(x, y, occ_xy, occ_q, prec):
    occ_xy.append((x, y))
    occ_q.add(qkey(x, y, prec))

def make_lines(n_lines, max_coord, margin):
    # evenly spaced in [margin, max_coord - margin]
    if n_lines < 2:
        return [margin, max_coord - margin]
    span = float(max_coord - 2.0*margin)
    if span <= 0:
        # fallback
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
    # lanes=3 => [-w, 0, +w], lanes=2 => [-w/2, +w/2]
    if lanes_per_road <= 1:
        return [0.0]
    offs = []
    mid = (lanes_per_road - 1) / 2.0
    i = 0
    while i < lanes_per_road:
        offs.append((i - mid) * lane_width)
        i += 1
    return offs

def sample_free_intersection(x_lines, y_lines, occ_xy, occ_q, min_dist2, prec, max_tries=200000):
    for _ in range(max_tries):
        x = random.choice(x_lines)
        y = random.choice(y_lines)
        if ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
            return (x, y)
    return (x, y)

def sample_free_lane_point(W, H, x_lines, y_lines, lane_offs, occ_xy, occ_q, min_dist2, prec, max_tries=200000):
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
# Sink deterministic position
# -----------------------------
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
    ap.add_argument("--out", type=str, default="S4.dat")
    ap.add_argument("--seed", type=int, default=42)

    ap.add_argument("--sink_id", type=int, default=0, help="Sink node id (default 0)")
    ap.add_argument("--sink_pos", type=str, default="center",
                    choices=["center","left","right","top","bottom",
                             "topleft","topright","bottomleft","bottomright","custom"])
    ap.add_argument("--sink_x", type=float, default=150.0)
    ap.add_argument("--sink_y", type=float, default=150.0)

    # Scenario
    ap.add_argument("--rsu_ratio", type=float, default=0.20, help="Fixed RSUs ratio among senders")

    # Manhattan grid
    ap.add_argument("--street_lines", type=int, default=16, help="Street lines per axis (>=2)")

    # Vehicles
    ap.add_argument("--veh_speed", type=float, default=12.0, help="m/s (≈40 km/h)")
    ap.add_argument("--turn_prob", type=float, default=0.35)
    ap.add_argument("--light_pause_min", type=int, default=0)
    ap.add_argument("--light_pause_max", type=int, default=30)
    ap.add_argument("--pause_prob", type=float, default=0.35)

    # No-overlap
    ap.add_argument("--min_dist", type=float, default=10.0, help="Increase for large N")
    ap.add_argument("--precision", type=int, default=3, help="Decimals in output file")

    # Lanes (multi-voies)
    ap.add_argument("--lanes_per_road", type=int, default=3, help="1..")
    ap.add_argument("--lane_width", type=float, default=3.0, help="meters")

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
    if args.street_lines < 2:
        raise SystemExit("Error: --street_lines must be >= 2")
    if dt <= 0:
        raise SystemExit("Error: --dt must be > 0")
    if args.lanes_per_road < 1:
        args.lanes_per_road = 1
    args.rsu_ratio = clamp(args.rsu_ratio, 0.0, 1.0)

    # Lanes offsets
    lane_offs = lane_offsets(args.lanes_per_road, args.lane_width)
    max_off = 0.0
    for o in lane_offs:
        if abs(o) > max_off:
            max_off = abs(o)

    # Anti-overlap
    min_dist2 = args.min_dist * args.min_dist

    # Margin so lanes + min_dist stay inside the area
    margin = max_off + args.min_dist + 1.0

    # Streets inside margin
    x_lines = make_lines(args.street_lines, W, margin)
    y_lines = make_lines(args.street_lines, H, margin)

    # Auto-fix lane_width if it is too large vs street spacing (avoid "lanes crossing roads")
    if args.street_lines >= 2:
        span = float(W - 2.0*margin)
        if span > 0:
            street_step = span / float(args.street_lines - 1)
            # require max_off <= 0.45 * street_step
            if max_off > 0.45 * street_step and args.lanes_per_road > 1:
                # scale down lane_width
                half = (args.lanes_per_road - 1) / 2.0
                if half > 0:
                    new_lane_width = (0.45 * street_step) / float(half)
                    args.lane_width = new_lane_width
                    lane_offs = lane_offsets(args.lanes_per_road, args.lane_width)
                    max_off = 0.0
                    for o in lane_offs:
                        if abs(o) > max_off:
                            max_off = abs(o)
                    margin = max_off + args.min_dist + 1.0
                    x_lines = make_lines(args.street_lines, W, margin)
                    y_lines = make_lines(args.street_lines, H, margin)

    # Sink deterministic (snap to intersection)
    sx, sy = sink_xy(args.sink_pos, W, H, x_lines, y_lines, args.sink_x, args.sink_y)

    # Split roles (exclude sink)
    ids = list(range(0, N))
    ids.remove(args.sink_id)
    random.shuffle(ids)

    senders = N - 1
    n_rsu = int(round(senders * args.rsu_ratio))
    n_rsu = max(0, min(senders, n_rsu))
    n_veh = senders - n_rsu

    rsu_ids = set(ids[:n_rsu])
    veh_ids = set(ids[n_rsu:])

    nodes = {}
    nodes[args.sink_id] = {"type": "sink", "x": sx, "y": sy}

    # Initial occupancy: reserve sink + rsu + vehicles start
    occ0_xy = []
    occ0_q = set()
    add_occ(sx, sy, occ0_xy, occ0_q, prec)

    # Fixed RSUs on intersections (unique)
    for mid in rsu_ids:
        rx, ry = sample_free_intersection(x_lines, y_lines, occ0_xy, occ0_q, min_dist2, prec)
        add_occ(rx, ry, occ0_xy, occ0_q, prec)
        nodes[mid] = {"type": "rsu", "x": rx, "y": ry}

    # Vehicles
    dirs = [(1,0), (-1,0), (0,1), (0,-1)]
    for mid in veh_ids:
        vx, vy, off = sample_free_lane_point(W, H, x_lines, y_lines, lane_offs, occ0_xy, occ0_q, min_dist2, prec)
        add_occ(vx, vy, occ0_xy, occ0_q, prec)

        dx, dy = random.choice(dirs)

        # enforce road constraint for initial placement
        if dx != 0:
            y0 = nearest_line(vy, y_lines)
            vy = clamp(y0 + off, 0.0, H)
            vx = clamp(vx, 0.0, W)
        else:
            x0 = nearest_line(vx, x_lines)
            vx = clamp(x0 + off, 0.0, W)
            vy = clamp(vy, 0.0, H)

        nodes[mid] = {
            "type": "veh",
            "x": vx, "y": vy,
            "dir": (dx, dy),
            "lane_off": off,
            "pause": random.randint(0, max(1, 5 // max(1, dt)))
        }

    veh_step = max(0.0, args.veh_speed * dt)
    eps = max(0.8, veh_step * 0.35)

    def maybe_turn(n):
        dx, dy = n["dir"]
        if random.random() < args.turn_prob:
            if dx != 0:
                n["dir"] = random.choice([(dx, 0), (0, 1), (0, -1)])
            else:
                n["dir"] = random.choice([(0, dy), (1, 0), (-1, 0)])

    def traffic_light_pause(n):
        if random.random() < args.pause_prob:
            if args.light_pause_max <= 0:
                n["pause"] = 0
            else:
                ps = random.randint(args.light_pause_min, args.light_pause_max)
                n["pause"] = ps // max(1, dt)

    line_fmt = "%%d %%d %%.%df %%.%df\n" % (prec, prec)

    with open(args.out, "w") as f:
        t = 0
        while t <= T:
            # write positions for all nodes 0..N-1
            for mid in range(0, N):
                n = nodes[mid]
                f.write(line_fmt % (mid, t, n["x"], n["y"]))

            if t == T:
                break

            # reserve sink + rsu (fixed positions cannot be overwritten)
            occ_xy = []
            occ_q = set()
            for mid in range(0, N):
                if nodes[mid]["type"] in ("sink", "rsu"):
                    add_occ(nodes[mid]["x"], nodes[mid]["y"], occ_xy, occ_q, prec)

            veh_list = list(veh_ids)
            random.shuffle(veh_list)

            for mid in veh_list:
                n = nodes[mid]

                if n["pause"] > 0:
                    n["pause"] -= 1
                    add_occ(n["x"], n["y"], occ_xy, occ_q, prec)
                    continue

                dx, dy = n["dir"]
                off = n["lane_off"]
                ox, oy = n["x"], n["y"]

                # propose full move
                nx = clamp(ox + dx * veh_step, 0.0, W)
                ny = clamp(oy + dy * veh_step, 0.0, H)

                # Manhattan constraint with lanes:
                # - horizontal motion: y = y_line + off, x continuous (snap to x_line when crossing)
                # - vertical motion: x = x_line + off, y continuous (snap to y_line when crossing)
                at_inter = False
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

                    # re-apply lane constraint to candidate
                    if dx != 0:
                        y0 = nearest_line(cy, y_lines)
                        cy = clamp(y0 + off, 0.0, H)
                    else:
                        x0 = nearest_line(cx, x_lines)
                        cx = clamp(x0 + off, 0.0, W)

                    if ok_pos(cx, cy, occ_xy, occ_q, min_dist2, prec):
                        chosen = (cx, cy)
                        break

                n["x"], n["y"] = chosen
                add_occ(n["x"], n["y"], occ_xy, occ_q, prec)

                # intersection behavior
                if at_inter and chosen != (ox, oy):
                    traffic_light_pause(n)
                    maybe_turn(n)

                # border bounce
                if n["x"] <= 0.0 or n["x"] >= W or n["y"] <= 0.0 or n["y"] >= H:
                    n["dir"] = (-dx, -dy)

            t += dt

    print("[OK] Generated:", args.out)
    print("  IDs: 0..%d (sink_id=%d)" % (N-1, args.sink_id))
    print("  fixed_RSU=%d, vehicles=%d" % (n_rsu, n_veh))
    print("  streets=%d, lanes=%d, lane_width=%.2f" % (args.street_lines, args.lanes_per_road, args.lane_width))
    print("  veh_speed=%.2f m/s, min_dist=%.2f, precision=%d" % (args.veh_speed, args.min_dist, prec))
    print("  area=%.0fx%.0f, margin=%.2f" % (W, H, margin))

if __name__ == "__main__":
    main()

