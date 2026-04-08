#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Générateur Polyvalent de Mobilité pour Cooja (.dat)
Format généré : <node_id> <time> <X> <Y>
"""

import math
import random
import argparse
import os

# ==============================================================================
# 🛠️ CONFIGURATION FACILE (Éditez ces variables si vous n'utilisez pas le Terminal)
# ==============================================================================

OUT_FILE         = "ma_topologie.dat"   # Nom du fichier généré

N_NODES          = 30                   # Nombre total de nœuds
SINK_ID          = 0                    # L'ID du Sink (ATTENTION: Cooja commence souvent par 1)

NB_FIXED_NODES   = 5                    # Nombre de nœuds STATIQUES (inclus le Sink)
FIXED_RADIUS     = 25.0                 # Distance des nœuds fixes par rapport au Sink

WIDTH            = 200.0                # Largeur de la carte (mètres)
HEIGHT           = 200.0                # Hauteur de la carte (mètres)
MARGIN           = 10.0                 # Marge de sécurité sur les bords de carte

DURATION         = 3600                 # Durée totale de la simulation (secondes)
TIME_STEP        = 1.0                  # Pas de calcul (met à jour position chaque 1s)

MOBILITY_POWER   = 3.0                  # Vitesse max des nœuds mobiles (mètres par seconde / pas)
SEED             = 42                   # Seed (pour pouvoir reproduire la même génération)

SINK_POS_X       = WIDTH / 2.0          # Position X du Sink (au centre par défaut)
SINK_POS_Y       = HEIGHT / 2.0         # Position Y du Sink (au centre par défaut)

# ==============================================================================
# LOGIQUE DE GÉNÉRATION (Ne pas toucher généralement)
# ==============================================================================

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def generate_mobility(args):
    random.seed(args.seed)

    print(f"--- Génération de mobilité: {args.out_file} ---")
    print(f"Total Nodes  : {args.n_nodes}")
    print(f"Fixed Nodes  : {args.nb_fixed}")
    print(f"Mobile Nodes : {args.n_nodes - args.nb_fixed}")
    print(f"Duration     : {args.duration}s (Step: {args.step}s)")
    print(f"Speed        : {args.power} m/s")

    xmin, xmax = args.margin, args.width - args.margin
    ymin, ymax = args.margin, args.height - args.margin

    positions = {}
    fixed_ids = []

    # 1. Assigner les IDs fixes (SINK en premier, puis d'autres)
    fixed_ids.append(args.sink_id)
    if args.nb_fixed > args.n_nodes:
        args.nb_fixed = args.n_nodes

    # Sélectionner les autres nœuds fixes (s'il y en a)
    available_ids = [i for i in range(1, args.n_nodes + 1) if i != args.sink_id]
    random.shuffle(available_ids)
    
    while len(fixed_ids) < args.nb_fixed:
        fixed_ids.append(available_ids.pop())

    # 2. Positionner les nœuds Fixes
    # Le Sink est au centre spécifié
    positions[args.sink_id] = [clamp(args.sink_x, xmin, xmax), clamp(args.sink_y, ymin, ymax)]

    # Placement circulaire pour les autres fixes autour du Sink
    other_fixed = [fid for fid in fixed_ids if fid != args.sink_id]
    nb_other_fixed = len(other_fixed)
    
    for i, nid in enumerate(other_fixed):
        angle = (2 * math.pi * i) / nb_other_fixed if nb_other_fixed > 0 else 0
        px = args.sink_x + args.radius * math.cos(angle)
        py = args.sink_y + args.radius * math.sin(angle)
        positions[nid] = [clamp(px, xmin, xmax), clamp(py, ymin, ymax)]

    # 3. Positionner les nœuds Mobiles aléatoirement
    for nid in range(1, args.n_nodes + 1):
        if nid not in positions:
            positions[nid] = [random.uniform(xmin, xmax), random.uniform(ymin, ymax)]

    # 4. Écrire le fichier .dat
    output_path = os.path.abspath(args.out_file)
    with open(output_path, "w") as f:
        t = 0.0
        while t <= args.duration:
            for nid in range(1, args.n_nodes + 1):
                # Déplacer si ce n'est pas un nœud fixe
                if nid not in fixed_ids:
                    dx = random.uniform(-args.power, args.power) * args.step
                    dy = random.uniform(-args.power, args.power) * args.step
                    nx = positions[nid][0] + dx
                    ny = positions[nid][1] + dy
                    # Random Waypoint 'Bounce' sur les bords
                    positions[nid][0] = clamp(nx, xmin, xmax)
                    positions[nid][1] = clamp(ny, ymin, ymax)

                # Écrire : <id> <time> <X> <Y>
                x, y = positions[nid]
                f.write(f"{nid} {t:g} {x:.3f} {y:.3f}\n")
            
            t += args.step

    print(f"\n[Succès] Fichier généré dans : {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Générateur de Fichier de Mobilité Cooja (.dat)")
    
    parser.add_argument("-o", "--out_file", type=str, default=OUT_FILE, help="Fichier de sortie (.dat)")
    parser.add_argument("-n", "--n_nodes", type=int, default=N_NODES, help="Nombre total de nœuds")
    parser.add_argument("--sink_id", type=int, default=SINK_ID, help="ID du Sink")
    parser.add_argument("--nb_fixed", type=int, default=NB_FIXED_NODES, help="Nombre de nœuds fixes (inclut le sink)")
    parser.add_argument("--radius", type=float, default=FIXED_RADIUS, help="Rayon du cercle des nœuds fixes")
    parser.add_argument("--width", type=float, default=WIDTH, help="Largeur de la zone")
    parser.add_argument("--height", type=float, default=HEIGHT, help="Hauteur de la zone")
    parser.add_argument("--margin", type=float, default=MARGIN, help="Marge des bords")
    parser.add_argument("-d", "--duration", type=float, default=DURATION, help="Durée de la simulation (secondes)")
    parser.add_argument("--step", type=float, default=TIME_STEP, help="Interval (pas) de tempe en secondes")
    parser.add_argument("-p", "--power", type=float, default=MOBILITY_POWER, help="Vitesse max de mobilité m/s")
    parser.add_argument("--sink_x", type=float, default=SINK_POS_X, help="Position X du Sink")
    parser.add_argument("--sink_y", type=float, default=SINK_POS_Y, help="Position Y du Sink")
    parser.add_argument("--seed", type=int, default=SEED, help="Seed pour la génération aléatoire")

    args = parser.parse_args()
    generate_mobility(args)
