#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
Générateur de Fichiers de Mobilité pour Cooja (Format Random Waypoint)
=============================================================================
Ce script génère un fichier de traces de mobilité au format attendu par
les plugins de mobilité de Cooja (.dat). Le fichier contient ligne par ligne
les coordonnées successives de chaque nœud dans le réseau.

Format du fichier de sortie :
<id_du_noeud> <temps_en_secondes> <coordonnee_X> <coordonnee_Y>

Auteur : [Rédigé pour Projet de Fin d'Études]
"""

import math
import random
import argparse
import os

# ==============================================================================
# PARAMÈTRES PAR DÉFAUT (Modifiables directement ici sans console)
# ==============================================================================
OUT_FILE         = "ma_topologie.dat"   # Nom du fichier généré

N_NODES          = 30                   # Nombre total de nœuds
SINK_ID          = 0                    # L'ID du nœud Puits (Sink)
NB_FIXED_NODES   = 5                    # Nombre de nœuds STATIQUES (inclus le Sink)
FIXED_RADIUS     = 25.0                 # Distance des nœuds fixes par rapport au Sink

WIDTH            = 200.0                # Largeur de la carte (mètres)
HEIGHT           = 200.0                # Hauteur de la carte (mètres)
MARGIN           = 10.0                 # Marge pour empêcher les nœuds de toucher les bords

DURATION         = 3600                 # Durée totale de la simulation (en secondes)
TIME_STEP        = 1.0                  # Résolution temporelle (1 position toutes les X s)

MOBILITY_POWER   = 3.0                  # Vitesse maximale en mètres par seconde
SEED             = 42                   # Graine de hasard (pour réplicabilité)

SINK_POS_X       = WIDTH / 2.0          # Position centrale du Sink sur l'axe X
SINK_POS_Y       = HEIGHT / 2.0         # Position centrale du Sink sur l'axe Y

# ==============================================================================
# FONCTIONS UTILITAIRES
# ==============================================================================
def clamp(val, min_val, max_val):
    """
    Empêche une valeur (coordonnée) de dépasser les limites de la zone autorisée.
    :param val: Valeur d'origine
    :param min_val: Valeur minimale
    :param max_val: Valeur maximale
    :return: Valeur bridée
    """
    return max(min_val, min(val, max_val))

# ==============================================================================
# FONCTION PRINCIPALE DE GÉNÉRATION
# ==============================================================================
def generate_mobility(args):
    """
    Génère le fichier de traces de mobilité basé sur les paramètres donnés.
    Logique :
      1. Placement du nœud Sink au centre (ou position configurée).
      2. Placement radial des nœuds statiques autour du Sink.
      3. Déplacement aléatoire (Random Waypoint) limité pour les nœuds mobiles.
    """
    # Fixer la graine aléatoire pour obtenir exactement le même fichier à chaque exécution
    random.seed(args.seed)

    print(f"--- Génération de mobilité: {args.out_file} ---")
    print(f"Total Nœuds  : {args.n_nodes}")
    print(f"Nœuds Fixes  : {args.nb_fixed}")
    print(f"Nœuds Mobiles: {args.n_nodes - args.nb_fixed}")
    print(f"Durée totale : {args.duration}s (Pas/Step de {args.step}s)")
    print(f"Vitesse max  : {args.power} m/s")

    # Limites opérationnelles de la zone (surface en retranchant les marges)
    xmin, xmax = args.margin, args.width - args.margin
    ymin, ymax = args.margin, args.height - args.margin

    positions = {}
    fixed_ids = []

    # Étape 1 : Assigner les IDs des nœuds fixes. Le Sink est toujours inclus en premier.
    fixed_ids.append(args.sink_id)
    if args.nb_fixed > args.n_nodes:
        args.nb_fixed = args.n_nodes

    # Trouver quels autres nœuds seront statiques, choisis aléatoirement
    available_ids = [i for i in range(1, args.n_nodes + 1) if i != args.sink_id]
    random.shuffle(available_ids)
    
    while len(fixed_ids) < args.nb_fixed and available_ids:
        fixed_ids.append(available_ids.pop())

    # Étape 2 : Positionner mathématiquement les nœuds Fixes
    # Le Sink prend sa place spécifique
    positions[args.sink_id] = [clamp(args.sink_x, xmin, xmax), clamp(args.sink_y, ymin, ymax)]

    # Placement des autres nœuds fixes en formation circulaire autour du Sink
    other_fixed = [fid for fid in fixed_ids if fid != args.sink_id]
    nb_other_fixed = len(other_fixed)
    
    for i, nid in enumerate(other_fixed):
        # Répartition équitable des angles (360° divisé par le nombre de nœuds restants)
        angle = (2 * math.pi * i) / nb_other_fixed if nb_other_fixed > 0 else 0
        px = args.sink_x + args.radius * math.cos(angle)
        py = args.sink_y + args.radius * math.sin(angle)
        positions[nid] = [clamp(px, xmin, xmax), clamp(py, ymin, ymax)]

    # Étape 3 : Placement initial aléatoire des nœuds mobiles
    for nid in range(1, args.n_nodes + 1):
        if nid not in positions:
            positions[nid] = [random.uniform(xmin, xmax), random.uniform(ymin, ymax)]

    # Étape 4 : Calculer les translations et écrire la sortie
    output_path = os.path.abspath(args.out_file)
    with open(output_path, "w") as f:
        t = 0.0
        while t <= args.duration:
            for nid in range(1, args.n_nodes + 1):
                
                # Si le nœud est mobile, réaliser le vecteur de déplacement
                if nid not in fixed_ids:
                    dx = random.uniform(-args.power, args.power) * args.step
                    dy = random.uniform(-args.power, args.power) * args.step
                    nx = positions[nid][0] + dx
                    ny = positions[nid][1] + dy
                    
                    # Empêcher le nœud de traverser les "murs" virtuels de la zone
                    positions[nid][0] = clamp(nx, xmin, xmax)
                    positions[nid][1] = clamp(ny, ymin, ymax)

                # Sérialisation dans le fichier log
                x, y = positions[nid]
                f.write(f"{nid} {t:g} {x:.3f} {y:.3f}\n")
            
            # Incrémenter le temps pour la prochaine simulation state
            t += args.step

    print(f"\n[Succès] Fichier généré dans : {output_path}")

# ==============================================================================
# POINT D'ENTRÉE DU SCRIPT
# ==============================================================================
if __name__ == "__main__":
    # Outil de traitement des arguments en ligne de commande pour lancer le script facilement 
    parser = argparse.ArgumentParser(description="Générateur de Fichier de Mobilité Cooja (.dat)")
    
    parser.add_argument("-o", "--out_file", type=str, default=OUT_FILE, help="Fichier de sortie (.dat)")
    parser.add_argument("-n", "--n_nodes", type=int, default=N_NODES, help="Nombre total de nœuds")
    parser.add_argument("--sink_id", type=int, default=SINK_ID, help="ID du Sink")
    parser.add_argument("--nb_fixed", type=int, default=NB_FIXED_NODES, help="Nombre de nœuds fixes (inclut le sink)")
    parser.add_argument("--radius", type=float, default=FIXED_RADIUS, help="Rayon du cercle des nœuds fixes")
    parser.add_argument("--width", type=float, default=WIDTH, help="Largeur de la zone géographique")
    parser.add_argument("--height", type=float, default=HEIGHT, help="Hauteur de la zone géographique")
    parser.add_argument("--margin", type=float, default=MARGIN, help="Marge des bords virtuels")
    parser.add_argument("-d", "--duration", type=float, default=DURATION, help="Durée de la simulation (secondes)")
    parser.add_argument("--step", type=float, default=TIME_STEP, help="Interval (pas) de temps en secondes")
    parser.add_argument("-p", "--power", type=float, default=MOBILITY_POWER, help="Vitesse max de mobilité m/s")
    parser.add_argument("--sink_x", type=float, default=SINK_POS_X, help="Position X initiale du Sink")
    parser.add_argument("--sink_y", type=float, default=SINK_POS_Y, help="Position Y initiale du Sink")
    parser.add_argument("--seed", type=int, default=SEED, help="Seed pour répliquer le même comportement aléatoire")

    args = parser.parse_args()
    generate_mobility(args)
