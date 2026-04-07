#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import random

# =========================
# Paramètres
# =========================
OUT_FILE = "20.dat"

N_NODES   = 20
SINK_ID   = 0
FIXED_IDS = [0, 1, 2, 3, 4]      # 5 fixes (inclut le sink)

WIDTH  = 200.0
HEIGHT = 200.0
MARGIN = 14.0                   # clamp dans [14 .. 386] si WIDTH=400

DURATION = 3600                 # instant_t max
DT       = 1                    # pas de temps (instant_t += DT)

STEP_MAX = 3.0                  # mobilité faible
SEED     = 42

SINK_POS      = (WIDTH/2.0, HEIGHT/2.0)
FIXED_RADIUS  = 25.0            # distance des fixes autour du sink


def clamp(v, vmin, vmax):
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v


def generate():
    random.seed(SEED)

    # Vérifs
    if SINK_ID not in FIXED_IDS:
        raise ValueError("SINK_ID must be inside FIXED_IDS")
    if len(set(FIXED_IDS)) != 5:
        raise ValueError("FIXED_IDS must contain exactly 5 unique IDs")
    if N_NODES <= max(FIXED_IDS):
        raise ValueError("N_NODES must be > max(FIXED_IDS)")

    xmin, xmax = MARGIN, WIDTH - MARGIN
    ymin, ymax = MARGIN, HEIGHT - MARGIN

    positions = {}

    # 1) Placer le sink (fixe)
    positions[SINK_ID] = [clamp(SINK_POS[0], xmin, xmax),
                          clamp(SINK_POS[1], ymin, ymax)]

    # 2) Placer les autres fixes autour du sink (croix)
    cross = [
        (positions[SINK_ID][0] + FIXED_RADIUS, positions[SINK_ID][1]),  # right
        (positions[SINK_ID][0] - FIXED_RADIUS, positions[SINK_ID][1]),  # left
        (positions[SINK_ID][0], positions[SINK_ID][1] + FIXED_RADIUS),  # up
        (positions[SINK_ID][0], positions[SINK_ID][1] - FIXED_RADIUS),  # down
    ]
    ci = 0
    for nid in FIXED_IDS:
        if nid == SINK_ID:
            continue
        x, y = cross[ci]
        ci += 1
        positions[nid] = [clamp(x, xmin, xmax), clamp(y, ymin, ymax)]

    # 3) Initialiser les mobiles aléatoirement
    for nid in range(N_NODES):
        if nid in positions:
            continue
        positions[nid] = [random.uniform(xmin, xmax),
                          random.uniform(ymin, ymax)]

    # 4) Écriture du .dat au format: <mote_id> <instant_t> <X> <Y>
    with open(OUT_FILE, "w") as f:
        t = 0
        while t <= DURATION:
            for nid in range(N_NODES):
                if nid not in FIXED_IDS:
                    dx = random.uniform(-STEP_MAX, STEP_MAX)
                    dy = random.uniform(-STEP_MAX, STEP_MAX)
                    x, y = positions[nid]
                    x = clamp(x + dx, xmin, xmax)
                    y = clamp(y + dy, ymin, ymax)
                    positions[nid] = [x, y]

                x, y = positions[nid]
                f.write("%d %d %.3f %.3f\n" % (nid, t, x, y))

            t += DT

    print("[OK] Generated:", OUT_FILE)


if __name__ == "__main__":
    generate()
