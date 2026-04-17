#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
python gen_LAB.py \
  --nodes 200 --duration 3600 --dt 2 --w 400 --h 400 \
  --out LAB200.dat --seed 42 \
  --sink_id 0 --sink_pos center \
  --stable_ratio 0.5 \
  --rw_speed 1.0 --pause_min 10 --pause_max 60 \
  --churn_period 120 --burst_fraction 0.35 --burst_speed 6 --burst_duration 30 \
  --min_dist 12 --precision 3

'''
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

def sample_free_rect(xmin, xmax, ymin, ymax, occ_xy, occ_q, min_dist2, prec, max_tries=200000):
    for _ in range(max_tries):
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        if ok_pos(x, y, occ_xy, occ_q, min_dist2, prec):
            return (x, y)
    return (x, y)

def fmt_line(mid, t, x, y, prec):
    # dynamic precision output
    return ("%d %d %." + str(prec) + "f %." + str(prec) + "f\n") % (mid, t, x, y)

# -----------------------------
# Deterministic sink position
# -----------------------------
def sink_xy(pos, W, H, custom_x, custom_y, margin):
    if pos == "custom":
        return (clamp(custom_x, margin, W - margin), clamp(custom_y, margin, H - margin))
    if pos == "center":
        return (W/2.0, H/2.0)
    if pos == "left":
        return (margin, H/2.0)
    if pos == "right":
        return (W - margin, H/2.0)
    if pos == "top":
        return (W/2.0, H - margin)
    if pos == "bottom":
        return (W/2.0, margin)
    if pos == "topleft":
        return (margin, H - margin)
    if pos == "topright":
        return (W - margin, H - margin)
    if pos == "bottomleft":
        return (margin, margin)
    if pos == "bottomright":
        return (W - margin, margin)
    return (W/2.0, H/2.0)

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
    ap.add_argument("--out", type=str, default="LAB.dat")
    ap.add_argument("--seed", type=int, default=42)

    # IDs
    ap.add_argument("--sink_id", type=int, default=0)
    ap.add_argument("--sink_pos", type=str, default="center",
                    choices=["center","left","right","top","bottom",
                             "topleft","topright","bottomleft","bottomright","custom"])
    ap.add_argument("--sink_x", type=float, default=150.0)
    ap.add_argument("--sink_y", type=float, default=150.0)

    # Mix
    ap.add_argument("--stable_ratio", type=float, default=0.50, help="Fraction of fixed stable nodes among senders")

    # Mobility (normal)
    ap.add_argument("--rw_speed", type=float, default=1.0, help="m/s (normal)")
    ap.add_argument("--pause_min", type=int, default=10)
    ap.add_argument("--pause_max", type=int, default=60)

    # Synthetic churn (test-only)
    ap.add_argument("--churn_period", type=int, default=120, help="seconds between churn events")
    ap.add_argument("--burst_fraction", type=float, default=0.35, help="fraction of mobiles entering burst each event")
    ap.add_argument("--burst_speed", type=float, default=6.0, help="m/s during burst")
    ap.add_argument("--burst_duration", type=int, default=30, help="seconds burst lasts")

    # Zones (left/right) + corridor width
    ap.add_argument("--corridor_ratio", type=float, default=0.10, help="middle band width ratio (0..0.3)")

    # No overlap + output
    ap.add_argument("--min_dist", type=float, default=10.0, help="increase for large N to avoid visual overlap")
    ap.add_argument("--precision", type=int, default=3, help="decimals in file (avoid rounding collisions)")

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
    if args.dt <= 0:
        raise SystemExit("Error: --dt must be > 0")

    stable_ratio = clamp(args.stable_ratio, 0.0, 1.0)
    min_dist2 = args.min_dist * args.min_dist

    # margins to reduce border issues
    margin = max(2.0, args.min_dist)

    # Zones: left / corridor / right
    c = clamp(args.corridor_ratio, 0.02, 0.30)
    x0 = margin
    x1 = W - margin

    cor_w = (x1 - x0) * c
    left_w = ((x1 - x0) - cor_w) / 2.0

    left_xmin = x0
    left_xmax = x0 + left_w
    cor_xmin  = left_xmax
    cor_xmax  = cor_xmin + cor_w
    right_xmin = cor_xmax
    right_xmax = x1

    ymin = margin
    ymax = H - margin

    # Sink fixed
    sx, sy = sink_xy(args.sink_pos, W, H, args.sink_x, args.sink_y, margin)

    # Role split (exclude sink)
    sender_ids = list(range(0, N))
    sender_ids.remove(args.sink_id)
    random.shuffle(sender_ids)

    senders = N - 1
    n_stable = int(round(senders * stable_ratio))
    n_stable = max(0, min(senders, n_stable))
    n_mobile = senders - n_stable

    stable_ids = set(sender_ids[:n_stable])
    mobile_ids = set(sender_ids[n_stable:])

    # Node states
    nodes = {}
    nodes[args.sink_id] = {"type": "sink", "x": sx, "y": sy}

    # Initial occupancy
    occ_xy = []
    occ_q = set()
    add_occ(sx, sy, occ_xy, occ_q, prec)

    # Place stable anchors on a grid (deterministic-ish) with slight jitter
    # Create a grid count roughly matching n_stable
    if n_stable > 0:
        gx = int(math.sqrt(n_stable))
        gy = int(math.ceil(float(n_stable) / float(max(1, gx))))
        gx = max(2, gx)
        gy = max(2, gy)
        stepx = (W - 2.0*margin) / float(gx - 1)
        stepy = (H - 2.0*margin) / float(gy - 1)

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
        for mid in stable_ids:
            # pick next grid point and add small jitter (but keep min_dist)
            placed = False
            tries = 0
            while (not placed) and tries < 50000:
                if k >= len(grid_points):
                    # fallback random
                    px, py = random.uniform(margin, W-margin), random.uniform(margin, H-margin)
                else:
                    px, py = grid_points[k]
                k += 1
                tries += 1
                # jitter
                jx = clamp(px + random.uniform(-1.0, 1.0), margin, W-margin)
                jy = clamp(py + random.uniform(-1.0, 1.0), margin, H-margin)
                if ok_pos(jx, jy, occ_xy, occ_q, min_dist2, prec):
                    nodes[mid] = {"type": "stable", "x": jx, "y": jy}
                    add_occ(jx, jy, occ_xy, occ_q, prec)
                    placed = True
            if not placed:
                # last resort: put far (may be dense)
                nodes[mid] = {"type": "stable", "x": px, "y": py}
                add_occ(px, py, occ_xy, occ_q, prec)

    # Place mobiles: half in left, half in right (test pattern)
    mobile_list = list(mobile_ids)
    random.shuffle(mobile_list)
    half = len(mobile_list) // 2
    left_m = set(mobile_list[:half])
    right_m = set(mobile_list[half:])

    def new_target(zone):
        if zone == "L":
            return (random.uniform(left_xmin, left_xmax), random.uniform(ymin, ymax))
        if zone == "R":
            return (random.uniform(right_xmin, right_xmax), random.uniform(ymin, ymax))
        # corridor
        return (random.uniform(cor_xmin, cor_xmax), random.uniform(ymin, ymax))

    for mid in mobile_list:
        zone = "L" if mid in left_m else "R"
        # start in zone
        px, py = sample_free_rect(left_xmin, left_xmax, ymin, ymax, occ_xy, occ_q, min_dist2, prec) if zone == "L" \
                 else sample_free_rect(right_xmin, right_xmax, ymin, ymax, occ_xy, occ_q, min_dist2, prec)
        add_occ(px, py, occ_xy, occ_q, prec)

        tx, ty = new_target(zone)
        pause_steps = random.randint(args.pause_min, args.pause_max) // max(1, dt)

        nodes[mid] = {
            "type": "mobile",
            "x": px, "y": py,
            "tx": tx, "ty": ty,
            "zone": zone,
            "pause": pause_steps,
            "burst_left": 0  # steps remaining in burst
        }

    # Steps per tick
    rw_step = max(0.0, args.rw_speed * dt)
    burst_step = max(0.0, args.burst_speed * dt)
    burst_steps = max(1, args.burst_duration // max(1, dt))

    churn_steps = max(1, args.churn_period // max(1, dt))

    def pick_new_waypoint(n):
        # normal target inside current zone
        tx, ty = new_target(n["zone"])
        n["tx"], n["ty"] = tx, ty

    def start_burst(n):
        # switch zone and create far target in opposite zone
        n["zone"] = "R" if n["zone"] == "L" else "L"
        tx, ty = new_target(n["zone"])
        n["tx"], n["ty"] = tx, ty
        n["pause"] = 0
        n["burst_left"] = burst_steps

    def propose_move(n, step):
        ox, oy = n["x"], n["y"]
        tx, ty = n["tx"], n["ty"]
        dx = tx - ox
        dy = ty - oy
        d = math.sqrt(dx*dx + dy*dy)
        if d < 1e-9:
            return (ox, oy, True)
        ux = dx / d
        uy = dy / d
        nx = ox + ux * step
        ny = oy + uy * step

        # clamp
        nx = clamp(nx, margin, W - margin)
        ny = clamp(ny, margin, H - margin)

        # arrival check (avoid oscillation)
        if dist2(nx, ny, tx, ty) <= (step * step):
            return (tx, ty, True)
        return (nx, ny, False)

    # -----------------------------
    # Simulation loop with strict no-overlap
    # -----------------------------
    with open(args.out, "w") as f:
        t = 0
        tick = 0
        while t <= T:
            # write
            for mid in range(0, N):
                n = nodes[mid]
                f.write(fmt_line(mid, t, n["x"], n["y"], prec))

            if t == T:
                break

            # churn event: every churn_steps, select some mobiles and burst-switch zone
            if (tick % churn_steps) == 0 and len(mobile_list) > 0:
                k = int(round(len(mobile_list) * clamp(args.burst_fraction, 0.0, 1.0)))
                k = max(0, min(len(mobile_list), k))
                burst_set = random.sample(mobile_list, k) if k > 0 else []
                for mid in burst_set:
                    start_burst(nodes[mid])

            # reserve fixed (sink + stable)
            occ_xy2 = []
            occ_q2 = set()
            for mid in range(0, N):
                if nodes[mid]["type"] in ("sink", "stable"):
                    add_occ(nodes[mid]["x"], nodes[mid]["y"], occ_xy2, occ_q2, prec)

            # update mobiles in random order
            random.shuffle(mobile_list)
            for mid in mobile_list:
                n = nodes[mid]

                # keep current occupied if paused / cannot move
                if n["pause"] > 0:
                    n["pause"] -= 1
                    add_occ(n["x"], n["y"], occ_xy2, occ_q2, prec)
                    continue

                step = burst_step if n["burst_left"] > 0 else rw_step
                nx, ny, arrived = propose_move(n, step)

                # collision avoidance: try full, half, quarter, else stay
                ox, oy = n["x"], n["y"]
                candidates = [
                    (nx, ny),
                    (ox + 0.5*(nx-ox), oy + 0.5*(ny-oy)),
                    (ox + 0.25*(nx-ox), oy + 0.25*(ny-oy)),
                    (ox, oy),
                ]

                chosen = (ox, oy)
                chosen_arrived = False
                for (cx, cy) in candidates:
                    cx = clamp(cx, margin, W - margin)
                    cy = clamp(cy, margin, H - margin)
                    if ok_pos(cx, cy, occ_xy2, occ_q2, min_dist2, prec):
                        chosen = (cx, cy)
                        # recompute arrival based on chosen
                        chosen_arrived = (dist2(cx, cy, n["tx"], n["ty"]) < 1e-6)
                        break

                n["x"], n["y"] = chosen
                add_occ(n["x"], n["y"], occ_xy2, occ_q2, prec)

                # burst countdown
                if n["burst_left"] > 0:
                    n["burst_left"] -= 1

                # if arrived: pause and choose next waypoint (normal behavior)
                if arrived or chosen_arrived:
                    if n["burst_left"] > 0:
                        # during burst: keep moving quickly (no long pauses)
                        pick_new_waypoint(n)
                    else:
                        n["pause"] = random.randint(args.pause_min, args.pause_max) // max(1, dt)
                        pick_new_waypoint(n)

            tick += 1
            t += dt

    print("[OK] Generated:", args.out)
    print("  IDs: 0..%d (sink_id=%d)" % (N-1, args.sink_id))
    print("  stable=%d, mobile=%d (stable_ratio=%.2f)" % (n_stable, n_mobile, stable_ratio))
    print("  churn_period=%ds, burst_fraction=%.2f, burst_duration=%ds" %
          (args.churn_period, args.burst_fraction, args.burst_duration))
    print("  area=%.0fx%.0f, min_dist=%.2f, precision=%d" % (W, H, args.min_dist, prec))

if __name__ == "__main__":
    main()

