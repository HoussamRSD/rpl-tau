#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

"""
S3 - Smart Hospital mobility scenario for Cooja/Contiki
Output per line: <mote_id> <time> <x> <y>

IDs: 0..N-1 (so "mote 1" corresponds to id 0)

Scenario:
- Sink: fixed (nurse station / server)
- Senders (N-1):
  - Fixed 30%: corridor routers / anchors (static on corridors)
  - Mobile 70%: patients (45%) + staff (25%)

Mobility:
- Patients: slow Random Waypoint + long pauses (60..600s), inside "rooms" areas
- Staff: corridor trips (grid) + medium pauses (10..120s), semi-directional

New parameters (same style as S1/S2):
- --sink_id (default 0), --sink_pos, --sink_x/y
- strict anti-overlap: --min_dist + --precision (also avoids same printed coordinates)
- scalable for 100..200 nodes:
  - room points generated per corridor cell: --rooms_per_cell
  - staff corridors multi-lane: --lanes_per_corridor + --lane_width
  - corridor density: --corridor_lines

Example:
python gen_S3_Hospital.py \
  --nodes 100 --duration 3600 --dt 2 --w 400 --h 400 \
  --out S3_100.dat --seed 42 \
  --sink_id 0 --sink_pos center \
  --fixed_ratio 0.30 --patient_ratio 0.45 --staff_ratio 0.25 \
  --corridor_lines 16 --rooms_per_cell 3 \
  --lanes_per_corridor 2 --lane_width 2.0 \
  --patient_speed 0.6 --patient_pause_min 60 --patient_pause_max 600 \
  --staff_speed 1.3 --staff_pause_min 10 --staff_pause_max 120 --turn_prob 0.35 \
  --min_dist 6 --precision 3
"""

import argparse
import random
import math

# -----------------------------
# Utils + fast occupancy (spatial hashing)
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

def cell_key(x, y, cell):
    return (int(x // cell), int(y // cell))

def ok_pos(x, y, occ_q, occ_grid, min_dist2, prec, cell):
    # avoid same printed coordinate
    if qkey(x, y, prec) in occ_q:
        return False

    ck = cell_key(x, y, cell)
    # check neighbor cells only
    for gx in (ck[0]-1, ck[0], ck[0]+1):
        for gy in (ck[1]-1, ck[1], ck[1]+1):
            pts = occ_grid.get((gx, gy))
            if pts:
                for (ox, oy) in pts:
                    if dist2(x, y, ox, oy) < min_dist2:
                        return False
    return True

def add_occ(x, y, occ_q, occ_grid, prec, cell):
    occ_q.add(qkey(x, y, prec))
    ck = cell_key(x, y, cell)
    if ck not in occ_grid:
        occ_grid[ck] = []
    occ_grid[ck].append((x, y))

def fmt_line(mid, t, x, y, prec):
    return ("%d %d %." + str(prec) + "f %." + str(prec) + "f\n") % (mid, t, x, y)

def make_lines(n_lines, max_coord, margin):
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
    b = nearest_line(v, lines)
    return (abs(v - b) <= eps), b

def lane_offsets(lanes_per, lane_width):
    if lanes_per <= 1:
        return [0.0]
    offs = []
    mid = (lanes_per - 1) / 2.0
    i = 0
    while i < lanes_per:
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

def sample_free_rect(xmin, xmax, ymin, ymax, occ_q, occ_grid, min_dist2, prec, cell, max_tries=800000):
    for _ in range(max_tries):
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        if ok_pos(x, y, occ_q, occ_grid, min_dist2, prec, cell):
            return (x, y)
    return (x, y)

def sample_free_corridor_point(W, H, x_lines, y_lines, off, occ_q, occ_grid, min_dist2, prec, cell, max_tries=800000):
    # pick a point on corridors (either horizontal or vertical), with lane offset
    for _ in range(max_tries):
        if random.random() < 0.5:
            y0 = random.choice(y_lines)
            y = clamp(y0 + off, 0.0, H)
            x = random.uniform(0.0, W)
        else:
            x0 = random.choice(x_lines)
            x = clamp(x0 + off, 0.0, W)
            y = random.uniform(0.0, H)
        if ok_pos(x, y, occ_q, occ_grid, min_dist2, prec, cell):
            return (x, y)
    return (x, y)

def build_room_points(W, H, x_lines, y_lines, rooms_per_cell, margin_inner):
    # create many unique room points inside each corridor cell rectangle
    rooms = []
    if len(x_lines) < 2 or len(y_lines) < 2:
        return rooms

    xi = 0
    while xi < len(x_lines) - 1:
        x1 = x_lines[xi]
        x2 = x_lines[xi + 1]
        yi = 0
        while yi < len(y_lines) - 1:
            y1 = y_lines[yi]
            y2 = y_lines[yi + 1]

            xmin = min(x1, x2) + margin_inner
            xmax = max(x1, x2) - margin_inner
            ymin = min(y1, y2) + margin_inner
            ymax = max(y1, y2) - margin_inner

            if xmax > xmin and ymax > ymin:
                k = 0
                while k < rooms_per_cell:
                    rx = random.uniform(xmin, xmax)
                    ry = random.uniform(ymin, ymax)
                    rooms.append((rx, ry))
                    k += 1

            yi += 1
        xi += 1

    # fallback if too few
    if len(rooms) < 10:
        for _ in range(200):
            rooms.append((random.uniform(0.0, W), random.uniform(0.0, H)))
    return rooms

def pick_different_point(points, x, y):
    tx, ty = random.choice(points)
    tries = 0
    while dist2(tx, ty, x, y) < 1e-8 and tries < 2000 and len(points) > 1:
        tx, ty = random.choice(points)
        tries += 1
    return tx, ty

# -----------------------------
# Main
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nodes", type=int, required=True, help="Total nodes (IDs 0..N-1)")
    ap.add_argument("--duration", type=int, default=3600)
    ap.add_argument("--dt", type=int, default=2)
    ap.add_argument("--w", type=float, default=400.0)
    ap.add_argument("--h", type=float, default=400.0)
    ap.add_argument("--out", type=str, default="S3.dat")
    ap.add_argument("--seed", type=int, default=42)

    # Sink
    ap.add_argument("--sink_id", type=int, default=0)
    ap.add_argument("--sink_pos", type=str, default="center",
                    choices=["center","left","right","top","bottom",
                             "topleft","topright","bottomleft","bottomright","custom"])
    ap.add_argument("--sink_x", type=float, default=200.0)
    ap.add_argument("--sink_y", type=float, default=200.0)

    # Ratios among senders (N-1)
    ap.add_argument("--fixed_ratio", type=float, default=0.30)
    ap.add_argument("--patient_ratio", type=float, default=0.45)
    ap.add_argument("--staff_ratio", type=float, default=0.25)

    # Hospital corridors
    ap.add_argument("--corridor_lines", type=int, default=16)

    # Rooms density
    ap.add_argument("--rooms_per_cell", type=int, default=3)

    # Staff corridor lanes (helps a lot for high N)
    ap.add_argument("--lanes_per_corridor", type=int, default=2)
    ap.add_argument("--lane_width", type=float, default=2.0)

    # Patients
    ap.add_argument("--patient_speed", type=float, default=0.6)
    ap.add_argument("--patient_pause_min", type=int, default=60)
    ap.add_argument("--patient_pause_max", type=int, default=600)

    # Staff
    ap.add_argument("--staff_speed", type=float, default=1.3)
    ap.add_argument("--staff_pause_min", type=int, default=10)
    ap.add_argument("--staff_pause_max", type=int, default=120)
    ap.add_argument("--turn_prob", type=float, default=0.35)

    # Anti-overlap
    ap.add_argument("--min_dist", type=float, default=6.0)
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
    if args.corridor_lines < 2:
        raise SystemExit("Error: --corridor_lines must be >= 2")
    if args.lanes_per_corridor < 1:
        args.lanes_per_corridor = 1
    if args.min_dist < 0:
        raise SystemExit("Error: --min_dist must be >= 0")

    # normalize ratios
    s = args.fixed_ratio + args.patient_ratio + args.staff_ratio
    if s <= 0:
        raise SystemExit("Error: ratios sum must be > 0")
    fixed_ratio = args.fixed_ratio / s
    patient_ratio = args.patient_ratio / s

    senders = N - 1
    n_fixed = int(round(senders * fixed_ratio))
    n_patient = int(round(senders * patient_ratio))
    n_fixed = max(0, min(senders, n_fixed))
    n_patient = max(0, min(senders - n_fixed, n_patient))
    n_staff = senders - n_fixed - n_patient

    # anti-overlap parameters
    min_dist2 = args.min_dist * args.min_dist
    cell = max(0.5, args.min_dist)  # spatial hash cell size

    # staff lane offsets
    staff_lane_offs = lane_offsets(args.lanes_per_corridor, args.lane_width)
    max_off = 0.0
    for o in staff_lane_offs:
        if abs(o) > max_off:
            max_off = abs(o)

    # margins
    margin = max_off + args.min_dist + 2.0
    margin_inner = max(2.0, 0.15 * (min(W, H) / float(args.corridor_lines)))

    # build corridor center lines inside margin
    x_lines = make_lines(args.corridor_lines, W, margin)
    y_lines = make_lines(args.corridor_lines, H, margin)

    # sink deterministic on corridor grid
    sx, sy = sink_xy(args.sink_pos, W, H, x_lines, y_lines, args.sink_x, args.sink_y)

    # assign roles (exclude sink)
    ids = list(range(0, N))
    ids.remove(args.sink_id)
    random.shuffle(ids)

    fixed_ids = set(ids[:n_fixed])
    patient_ids = set(ids[n_fixed:n_fixed + n_patient])
    staff_ids = set(ids[n_fixed + n_patient:])

    nodes = {}
    nodes[args.sink_id] = {"type": "sink", "x": sx, "y": sy}

    # initial occupancy
    occ_q = set()
    occ_grid = {}
    add_occ(sx, sy, occ_q, occ_grid, prec, cell)

    # build room points (lots of them) for patient RWP
    room_points = build_room_points(W, H, x_lines, y_lines, max(1, args.rooms_per_cell), margin_inner)

    # place fixed corridor routers (on corridors, continuous positions, offset 0)
    fixed_list = list(fixed_ids)
    random.shuffle(fixed_list)
    for mid in fixed_list:
        fx, fy = sample_free_corridor_point(W, H, x_lines, y_lines, 0.0, occ_q, occ_grid, min_dist2, prec, cell)
        nodes[mid] = {"type": "fixed", "x": fx, "y": fy}
        add_occ(fx, fy, occ_q, occ_grid, prec, cell)

    # place patients in rooms (free plane)
    for mid in patient_ids:
        px, py = sample_free_rect(margin, W - margin, margin, H - margin, occ_q, occ_grid, min_dist2, prec, cell)
        tx, ty = pick_different_point(room_points, px, py)
        nodes[mid] = {
            "type": "patient",
            "x": px, "y": py,
            "tx": tx, "ty": ty,
            "pause": random.randint(args.patient_pause_min, args.patient_pause_max) // max(1, dt)
        }
        add_occ(px, py, occ_q, occ_grid, prec, cell)

    # staff starts on corridor lanes
    for mid in staff_ids:
        off = random.choice(staff_lane_offs)
        sx0, sy0 = sample_free_corridor_point(W, H, x_lines, y_lines, off, occ_q, occ_grid, min_dist2, prec, cell)

        # choose a staff target near corridor intersections (snap-ish)
        tx = random.choice(x_lines)
        ty = random.choice(y_lines)
        tx = clamp(tx + off, 0.0, W)
        ty = clamp(ty + off, 0.0, H)

        nodes[mid] = {
            "type": "staff",
            "x": sx0, "y": sy0,
            "tx": tx, "ty": ty,
            "off": off,
            "axis": "x" if random.random() < 0.5 else "y",
            "pause": random.randint(args.staff_pause_min, args.staff_pause_max) // max(1, dt)
        }
        add_occ(sx0, sy0, occ_q, occ_grid, prec, cell)

    # movement steps
    patient_step = max(0.0, args.patient_speed * dt)
    staff_step = max(0.0, args.staff_speed * dt)
    eps_p = max(0.8, patient_step * 0.6)
    eps_s = max(1.0, staff_step * 0.35)

    def pick_new_patient_target(n):
        tx, ty = pick_different_point(room_points, n["x"], n["y"])
        n["tx"], n["ty"] = tx, ty

    def pick_new_staff_target(n):
        off = n["off"]
        tx = random.choice(x_lines)
        ty = random.choice(y_lines)
        n["tx"] = clamp(tx + off, 0.0, W)
        n["ty"] = clamp(ty + off, 0.0, H)
        n["axis"] = "x" if random.random() < 0.5 else "y"

    def propose_patient_move(n, factor):
        ox, oy = n["x"], n["y"]
        tx, ty = n["tx"], n["ty"]
        dx = tx - ox
        dy = ty - oy
        d = math.sqrt(dx*dx + dy*dy)

        if d <= eps_p:
            return (tx, ty, True)

        step = patient_step * factor
        if step <= 0.0:
            return (ox, oy, False)

        ux = dx / d
        uy = dy / d
        nx = clamp(ox + ux * step, margin, W - margin)
        ny = clamp(oy + uy * step, margin, H - margin)

        # close enough -> snap
        if dist2(nx, ny, tx, ty) <= (step*step):
            return (tx, ty, True)
        return (nx, ny, False)

    def propose_staff_move(n, factor):
        ox, oy = n["x"], n["y"]
        tx, ty = n["tx"], n["ty"]
        off = n["off"]
        step = staff_step * factor
        if step <= 0.0:
            return (ox, oy, False)

        # reached
        if abs(ox - tx) <= eps_s and abs(oy - ty) <= eps_s:
            return (tx, ty, True)

        # sometimes turn
        if random.random() < args.turn_prob:
            n["axis"] = "y" if n["axis"] == "x" else "x"

        nx, ny = ox, oy

        if n["axis"] == "x":
            # keep y on nearest corridor lane: y = nearest y_line + off
            y0 = nearest_line(oy - off, y_lines) + off
            ny = clamp(y0, 0.0, H)

            if ox < tx:
                nx = clamp(ox + step, 0.0, W)
            else:
                nx = clamp(ox - step, 0.0, W)

            hit, xhit = near_line(nx - off, x_lines, eps_s)
            if hit:
                nx = clamp(xhit + off, 0.0, W)

        else:
            # keep x on nearest corridor lane: x = nearest x_line + off
            x0 = nearest_line(ox - off, x_lines) + off
            nx = clamp(x0, 0.0, W)

            if oy < ty:
                ny = clamp(oy + step, 0.0, H)
            else:
                ny = clamp(oy - step, 0.0, H)

            hit, yhit = near_line(ny - off, y_lines, eps_s)
            if hit:
                ny = clamp(yhit + off, 0.0, H)

        arrived = (abs(nx - tx) <= eps_s and abs(ny - ty) <= eps_s)
        if arrived:
            nx, ny = tx, ty
        return (nx, ny, arrived)

    # -----------------------------
    # Simulation loop (strict no-overlap)
    # -----------------------------
    with open(args.out, "w") as f:
        t = 0
        while t <= T:
            for mid in range(0, N):
                n = nodes[mid]
                f.write(fmt_line(mid, t, n["x"], n["y"], prec))

            if t == T:
                break

            # reset occupancy for next instant (reserve sink+fixed first)
            occ_q2 = set()
            occ_grid2 = {}

            for mid in range(0, N):
                if nodes[mid]["type"] in ("sink", "fixed"):
                    add_occ(nodes[mid]["x"], nodes[mid]["y"], occ_q2, occ_grid2, prec, cell)

            # update mobiles in random order
            mobile_ids = list(patient_ids) + list(staff_ids)
            random.shuffle(mobile_ids)

            for mid in mobile_ids:
                n = nodes[mid]
                ox, oy = n["x"], n["y"]

                if n["pause"] > 0:
                    n["pause"] -= 1
                    add_occ(ox, oy, occ_q2, occ_grid2, prec, cell)
                    continue

                if n["type"] == "patient":
                    # try full/half/quarter + small side dodge + stay
                    best = (ox, oy, False)

                    candidates = []
                    candidates.append(propose_patient_move(n, 1.0))
                    candidates.append(propose_patient_move(n, 0.5))
                    candidates.append(propose_patient_move(n, 0.25))
                    # side dodge (perpendicular small)
                    nx, ny, arr = propose_patient_move(n, 0.5)
                    px = -(n["ty"] - oy)
                    py =  (n["tx"] - ox)
                    pd = math.sqrt(px*px + py*py)
                    if pd > 1e-9:
                        px /= pd
                        py /= pd
                        j = max(0.5, args.min_dist * 0.5)
                        candidates.append((clamp(nx + px*j, margin, W-margin), clamp(ny + py*j, margin, H-margin), False))
                        candidates.append((clamp(nx - px*j, margin, W-margin), clamp(ny - py*j, margin, H-margin), False))
                    candidates.append((ox, oy, False))

                    chosen = (ox, oy, False)
                    for (cx, cy, carr) in candidates:
                        if ok_pos(cx, cy, occ_q2, occ_grid2, min_dist2, prec, cell):
                            chosen = (cx, cy, carr)
                            break

                    n["x"], n["y"] = chosen[0], chosen[1]
                    add_occ(n["x"], n["y"], occ_q2, occ_grid2, prec, cell)

                    if chosen[2]:
                        ps = random.randint(args.patient_pause_min, args.patient_pause_max)
                        n["pause"] = ps // max(1, dt)
                        pick_new_patient_target(n)

                elif n["type"] == "staff":
                    candidates = []
                    candidates.append(propose_staff_move(n, 1.0))
                    candidates.append(propose_staff_move(n, 0.5))
                    candidates.append(propose_staff_move(n, 0.25))
                    candidates.append((ox, oy, False))

                    chosen = (ox, oy, False)
                    for (cx, cy, carr) in candidates:
                        if ok_pos(cx, cy, occ_q2, occ_grid2, min_dist2, prec, cell):
                            chosen = (cx, cy, carr)
                            break

                    # if blocked, try changing axis/target once
                    if chosen[0] == ox and chosen[1] == oy:
                        if random.random() < 0.6:
                            n["axis"] = "y" if n["axis"] == "x" else "x"
                        if random.random() < 0.25:
                            pick_new_staff_target(n)
                        cx, cy, carr = propose_staff_move(n, 0.5)
                        if ok_pos(cx, cy, occ_q2, occ_grid2, min_dist2, prec, cell):
                            chosen = (cx, cy, carr)

                    n["x"], n["y"] = chosen[0], chosen[1]
                    add_occ(n["x"], n["y"], occ_q2, occ_grid2, prec, cell)

                    if chosen[2]:
                        ps = random.randint(args.staff_pause_min, args.staff_pause_max)
                        n["pause"] = ps // max(1, dt)
                        pick_new_staff_target(n)

            t += dt

    print("[OK] Generated:", args.out)
    print("  IDs: 0..%d (sink_id=%d)" % (N-1, args.sink_id))
    print("  fixed=%d, patients=%d, staff=%d" % (n_fixed, n_patient, n_staff))
    print("  sink_pos=%s, min_dist=%.2f, precision=%d" % (args.sink_pos, args.min_dist, prec))
    print("  corridors=%d lines/axis, lanes=%d, lane_width=%.2f" % (args.corridor_lines, args.lanes_per_corridor, args.lane_width))
    print("  rooms_per_cell=%d" % (args.rooms_per_cell))
    print("  area=%.0fx%.0f, dt=%ds, T=%ds" % (W, H, dt, T))

if __name__ == "__main__":
    main()

