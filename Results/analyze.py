import os
import re
import glob
import statistics

def analyze_logs(directory):
    """
    Analyse les fichiers journaux (.log) d'une simulation Cooja.
    Extrait les statistiques vitales de performance du réseau : PDR (Packet Delivery Ratio),
    Latence moyenne (End-to-End Delay), Overhead de Contrôle (RPL DIO/DAO/DIS), et instabilité (Parent Changes).
    Génère un rapport consolidé avec les moyennes et écart-types par topologie.
    """
    log_files = glob.glob(os.path.join(directory, "*.log"))
    if not log_files:
        print("No log files found in directory:", directory)
        return

    # ==============================================================================
    # DÉCLARATION DES EXPRESSIONS RÉGULIÈRES (REGEX)
    # Ces filtres extraient les événements du simulateur "texte"
    # ==============================================================================
    
    # Traitement robuste des logs Cooja (timestamp:node_id:payload)
    # On autorise du texte optionnel (debug) entre le node_id et l'action
    send_pattern = re.compile(r'^\s*(\d+)\s*:\s*(\d+)\s*:.*Client sending\s+(\d+)', re.IGNORECASE)
    recv_pattern = re.compile(r'^\s*(\d+)\s*:\s*(\d+)\s*:.*Server received\s+(\d+)\s+from\s+(\d+)', re.IGNORECASE)



    # Trafic de Contrôle (Couche Routage RPL)
    dio_pattern = re.compile(r'Sending a multicast-DIO|Sending unicast-DIO')
    dao_pattern = re.compile(r'Sending a DAO|Sending a No-Path DAO')
    dis_pattern = re.compile(r'Sending a DIS')
    
    # Exact pattern for parent switch deduction
    switch_pattern = re.compile(r'^(\d+):(\d+):#A Parent Switch!')
    
    # Energest pattern: timestamp : node_id : Energest: CPU LPM TX RX ...
    energest_pattern = re.compile(r'^\s*(\d+)\s*:\s*(\d+)\s*:.*Energest:\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)', re.IGNORECASE)
    
    # Filename matching pattern
    fname_pattern = re.compile(r'TOPO-(\d+)-RS-(\d+)\.log')

    # ==========================================================================
    # REGEX Q-LEARNING SUPERVISOR (tags emis par rpl-of-tau.c)
    # ==========================================================================

    # [RL-STATE] URGENCY (0): tau_pref=180 tau_cand=520 delta=+340
    rl_state_pattern = re.compile(
        r'\[RL-STATE\]\s+(\w+)\s+\((\d+)\).*delta=([+-]?\d+)')

    # [RL-UPDATE] REWARD TOTAL: +42  (etx=+12  stability=+30)
    rl_reward_pattern = re.compile(
        r'\[RL-UPDATE\]\s+REWARD TOTAL:\s+([+-]?\d+).*etx=([+-]?\d+).*stability=([+-]?\d+)')

    # [RL-UPDATE] REWARD: RUPTURE (-50)
    rl_rupture_pattern = re.compile(r'\[RL-UPDATE\]\s+REWARD:\s+RUPTURE')

    # [RL-TABLE] #12 | URG[stay=-3 sw=28] MIN[stay=15 sw=-5] MAJ[stay=-2 sw=41]
    rl_table_pattern = re.compile(
        r'\[RL-TABLE\]\s+#(\d+)\s+\|\s+'
        r'URG\[stay=([+-]?\d+)\s+sw=([+-]?\d+)\]\s+'
        r'MIN\[stay=([+-]?\d+)\s+sw=([+-]?\d+)\]\s+'
        r'MAJ\[stay=([+-]?\d+)\s+sw=([+-]?\d+)\]')

    # [LAYER1] Classic decision => STAY / SWITCH
    layer1_stay_pattern   = re.compile(r'\[LAYER1\]\s+Classic decision\s*=>\s*STAY')
    layer1_switch_pattern = re.compile(r'\[LAYER1\]\s+Classic decision\s*=>\s*SWITCH')

    # [LAYER2] *** PROACTIVE SWITCH OVERRIDE ***
    ql_proactive_switch_pattern  = re.compile(r'\[LAYER2\]\s+\*\*\*\s+PROACTIVE SWITCH')
    # [LAYER2] *** PROTECTIVE STAY OVERRIDE ***
    ql_protective_stay_pattern   = re.compile(r'\[LAYER2\]\s+\*\*\*\s+PROTECTIVE STAY')
    # [LAYER2] QL UNCERTAIN
    ql_uncertain_pattern         = re.compile(r'\[LAYER2\]\s+QL UNCERTAIN')
    # [LAYER2] BOOTSTRAP MODE
    ql_bootstrap_pattern         = re.compile(r'\[LAYER2\]\s+BOOTSTRAP MODE')

    # [OF-TAU] PANIC!
    panic_pattern = re.compile(r'OF-TAU PANIC!')

    report = []
    def log_print(msg):
        print(msg)
        report.append(msg)

    log_print(f"Directory: {directory}")
    log_print(f"Log files found: {len(log_files)}")
    log_print("-" * 50)

    # Group by topology: topo_data[X] = list of results for each Y
    topo_data = {}

    for file_path in log_files:
        file_name = os.path.basename(file_path)
        m_fname = fname_pattern.search(file_name)
        if m_fname:
            topo_id = int(m_fname.group(1))
            run_id = int(m_fname.group(2))
        else:
            topo_id = 1
            run_id = 0
            
        if topo_id not in topo_data:
            topo_data[topo_id] = []
        
        sent_dict = {} # (node_id, seq) -> time_sent
        recv_set = set()  # set to avoid duplicate reception counting
        node_energy = {}  # node_id -> dict of energest values
        
        # ----------------------------------------------------------------------
        # Variables de comptage pour chaque fichier (File/Run = 'f_')
        # ----------------------------------------------------------------------
        f_sent = 0
        f_recv = 0
        f_latency_sum = 0.0
        f_latency_count = 0
        f_dio = 0
        f_dao = 0
        f_dis = 0
        f_parent_changes = 0

        # ----------------------------------------------------------------------
        # Variables Q-Learning (prefix 'rl_')
        # ----------------------------------------------------------------------
        rl_state_counts     = {'URGENCY': 0, 'MINOR': 0, 'MAJOR': 0}
        rl_delta_tau_list   = []          # liste des delta_tau observes
        rl_reward_list      = []          # recompenses totales par cycle
        rl_etx_reward_list  = []          # composante ETX de la recompense
        rl_stab_reward_list = []          # composante stabilite de la recompense
        rl_rupture_count    = 0           # nombre de ruptures detectees (malus -50)
        rl_proactive_switches = 0         # overrides SWITCH par QL
        rl_protective_stays   = 0         # overrides STAY par QL
        rl_ql_uncertain_count = 0         # fois ou QL etait incertain
        rl_bootstrap_count    = 0         # fois ou QL en mode bootstrap
        rl_layer1_stay_count  = 0         # decisions STAY de la couche classique
        rl_layer1_switch_count= 0         # decisions SWITCH de la couche classique
        rl_panic_count        = 0         # declenchements du Panic Monitor
        rl_last_qtable        = None      # dernier snapshot Q-table
        rl_qtable_snapshots   = []        # historique Q-tables (convergence)

        # Moteur de parsing Ligne-par-Ligne (Mémoire efficiente)
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                
                # --- 1. DÉTECTION DE L'INSTABILITÉ (CHANGEMENTS DE PARENT) ---
                m_switch = switch_pattern.search(line)
                if m_switch:
                    f_parent_changes += 1
                    continue
                
                # --- 2. TRANSMISSION APPLICATION (UDP SENDER) ---
                m_send = send_pattern.search(line)
                if m_send:
                    time_us = int(m_send.group(1))
                    node_id = int(m_send.group(2))
                    seq = int(m_send.group(3))
                    
                    # On mémorise le Timestamp (Microsecondes) pour calculer la Latence plus tard
                    # Note: f_sent compte les messages uniques (node, seq)
                    if (node_id, seq) not in sent_dict:
                        sent_dict[(node_id, seq)] = time_us
                        f_sent += 1
                    continue
                
                # --- 3. RECEPTION APPLICATION (UDP RECEIVER / SINK) ---
                m_recv = recv_pattern.search(line)
                if m_recv:
                    time_us = int(m_recv.group(1))
                    # group(2) est le node_id du serveur, group(3) est seq, group(4) est la source
                    seq = int(m_recv.group(3))
                    node_id = int(m_recv.group(4))
                    
                    # VÉRIFICATION UNIQUE : Ignorer les paquets dupliqués
                    if (node_id, seq) not in recv_set:
                        recv_set.add((node_id, seq))
                        
                        # VÉRIFICATION : Un paquet ne peut être "reçu" que s'il a été "envoyé"
                        # Cela évite les PDR > 100% dus à des erreurs de parsing ou logs tronqués
                        if (node_id, seq) in sent_dict:
                            f_recv += 1
                            time_sent = sent_dict[(node_id, seq)]
                            
                            # Calcul du Délai de Bout-en-Bout (End-to-end Latency)
                            latency_ms = (time_us - time_sent) / 1000.0
                            if latency_ms >= 0:
                                f_latency_sum += latency_ms
                                f_latency_count += 1
                    
                    continue
                
                # --- 4. SURCOÛT DE CONTRÔLE (CONTROL OVERHEAD RPL) ---
                if "Sending a multicast-DIO" in line or "Sending unicast-DIO" in line:
                    f_dio += 1
                elif "Sending a DAO" in line or "Sending a No-Path DAO" in line:
                    f_dao += 1
                elif "Sending a DIS" in line:
                    f_dis += 1
                    
                # --- 5. ENERGEST LOGS ---
                m_energest = energest_pattern.search(line)
                if m_energest:
                    e_node = int(m_energest.group(2))
                    node_energy[e_node] = {
                        'cpu': int(m_energest.group(3)),
                        'lpm': int(m_energest.group(4)),
                        'tx': int(m_energest.group(5)),
                        'rx': int(m_energest.group(6))
                    }

                # --- 6. Q-LEARNING SUPERVISOR METRICS ---

                # [RL-STATE] : distribution des etats + delta_tau
                m_rl_state = rl_state_pattern.search(line)
                if m_rl_state:
                    state_name = m_rl_state.group(1).upper()
                    if state_name in rl_state_counts:
                        rl_state_counts[state_name] += 1
                    try:
                        rl_delta_tau_list.append(int(m_rl_state.group(3)))
                    except ValueError:
                        pass
                    continue

                # [RL-UPDATE] REWARD TOTAL : recompense hybride par cycle
                m_reward = rl_reward_pattern.search(line)
                if m_reward:
                    rl_reward_list.append(int(m_reward.group(1)))
                    rl_etx_reward_list.append(int(m_reward.group(2)))
                    rl_stab_reward_list.append(int(m_reward.group(3)))
                    continue

                # [RL-UPDATE] REWARD: RUPTURE
                if rl_rupture_pattern.search(line):
                    rl_rupture_count += 1
                    continue

                # [RL-TABLE] : snapshot Q-table (convergence)
                m_qtable = rl_table_pattern.search(line)
                if m_qtable:
                    snapshot = {
                        'update': int(m_qtable.group(1)),
                        'urg_stay':  int(m_qtable.group(2)),
                        'urg_sw':    int(m_qtable.group(3)),
                        'min_stay':  int(m_qtable.group(4)),
                        'min_sw':    int(m_qtable.group(5)),
                        'maj_stay':  int(m_qtable.group(6)),
                        'maj_sw':    int(m_qtable.group(7)),
                    }
                    rl_qtable_snapshots.append(snapshot)
                    rl_last_qtable = snapshot
                    continue

                # [LAYER1] Classic decision
                if layer1_stay_pattern.search(line):
                    rl_layer1_stay_count += 1
                    continue
                if layer1_switch_pattern.search(line):
                    rl_layer1_switch_count += 1
                    continue

                # [LAYER2] QL override events
                if ql_proactive_switch_pattern.search(line):
                    rl_proactive_switches += 1
                    continue
                if ql_protective_stay_pattern.search(line):
                    rl_protective_stays += 1
                    continue
                if ql_uncertain_pattern.search(line):
                    rl_ql_uncertain_count += 1
                    continue
                if ql_bootstrap_pattern.search(line):
                    rl_bootstrap_count += 1
                    continue

                # OF-TAU PANIC events
                if panic_pattern.search(line):
                    rl_panic_count += 1
                    continue

        f_pdr = (f_recv / f_sent * 100) if f_sent > 0 else 0.0
        f_avg_latency = (f_latency_sum / f_latency_count) if f_latency_count > 0 else 0.0
        f_overhead = f_dio + f_dao + f_dis
        
        # Calculate Average Energy Consumption (CC2420 / Z1 Mote Params, 3V)
        # RTIMER_SECOND = 32768
        total_energy_mJ = 0.0
        for nid, consts in node_energy.items():
            cpu_mJ = (consts['cpu'] * 3.0 * 1.8) / 32768.0
            lpm_mJ = (consts['lpm'] * 3.0 * 0.0545) / 32768.0
            tx_mJ  = (consts['tx'] * 3.0 * 17.4) / 32768.0
            rx_mJ  = (consts['rx'] * 3.0 * 18.8) / 32768.0
            total_energy_mJ += (cpu_mJ + lpm_mJ + tx_mJ + rx_mJ)
            
        f_avg_energy = (total_energy_mJ / len(node_energy)) if len(node_energy) > 0 else 0.0
        
        # RL aggregated metrics for this file
        rl_total_decisions = rl_layer1_stay_count + rl_layer1_switch_count
        rl_total_updates   = len(rl_reward_list)

        rl_avg_reward      = (sum(rl_reward_list) / rl_total_updates)\
                              if rl_total_updates > 0 else 0.0
        rl_avg_etx_r       = (sum(rl_etx_reward_list) / rl_total_updates)\
                              if rl_total_updates > 0 else 0.0
        rl_avg_stab_r      = (sum(rl_stab_reward_list) / rl_total_updates)\
                              if rl_total_updates > 0 else 0.0
        rl_avg_delta_tau   = (sum(rl_delta_tau_list) / len(rl_delta_tau_list))\
                              if rl_delta_tau_list else 0.0

        # Override rate = QL overrides / total LAYER1 decisions where QL was active
        rl_total_overrides = rl_proactive_switches + rl_protective_stays
        rl_override_rate   = (rl_total_overrides / rl_total_decisions * 100)\
                              if rl_total_decisions > 0 else 0.0

        # State visit distribution (%)
        rl_total_states = sum(rl_state_counts.values())
        rl_state_pct = {
            k: (v / rl_total_states * 100) if rl_total_states > 0 else 0.0
            for k, v in rl_state_counts.items()
        }

        # Q-table convergence: delta between first and last snapshot
        rl_convergence_delta = None
        if len(rl_qtable_snapshots) >= 2:
            first = rl_qtable_snapshots[0]
            last  = rl_qtable_snapshots[-1]
            # Sum of absolute changes across all cells
            keys = ['urg_stay','urg_sw','min_stay','min_sw','maj_stay','maj_sw']
            rl_convergence_delta = sum(
                abs(last[k] - first[k]) for k in keys
            )

        topo_data[topo_id].append({
            'file': file_name,
            'run': run_id,
            'sent': f_sent,
            'recv': f_recv,
            'pdr': f_pdr,
            'avg_latency': f_avg_latency,
            'parent_changes': f_parent_changes,
            'overhead': f_overhead,
            'dio': f_dio,
            'dao': f_dao,
            'dis': f_dis,
            'avg_energy': f_avg_energy,
            # --- RL Metrics ---
            'rl_total_updates':       rl_total_updates,
            'rl_total_decisions':     rl_total_decisions,
            'rl_layer1_stay':         rl_layer1_stay_count,
            'rl_layer1_switch':       rl_layer1_switch_count,
            'rl_proactive_switches':  rl_proactive_switches,
            'rl_protective_stays':    rl_protective_stays,
            'rl_override_rate':       rl_override_rate,
            'rl_uncertain_count':     rl_ql_uncertain_count,
            'rl_bootstrap_count':     rl_bootstrap_count,
            'rl_rupture_count':       rl_rupture_count,
            'rl_panic_count':         rl_panic_count,
            'rl_avg_reward':          rl_avg_reward,
            'rl_avg_etx_reward':      rl_avg_etx_r,
            'rl_avg_stab_reward':     rl_avg_stab_r,
            'rl_avg_delta_tau':       rl_avg_delta_tau,
            'rl_state_pct':           rl_state_pct,
            'rl_last_qtable':         rl_last_qtable,
            'rl_convergence_delta':   rl_convergence_delta,
        })



    # Sort and evaluate per Topology
    sorted_topos = sorted(topo_data.keys())
    
    for topo_id in sorted_topos:
        log_print(f"\n==================================================")
        log_print(f"      EVALUATION FOR TOPOLOGY {topo_id}")
        log_print(f"==================================================")
        
        runs = topo_data[topo_id]
        runs = sorted(runs, key=lambda x: x['run'])
        
        pdr_list = []
        latency_list = []
        pc_list = []
        overhead_list = []
        dio_list = []
        dao_list = []
        dis_list = []
        energy_list = []

        # RL metric lists (per run)
        rl_override_rate_list   = []
        rl_avg_reward_list      = []
        rl_rupture_list         = []
        rl_panic_list           = []
        rl_proactive_sw_list    = []
        rl_protective_st_list   = []
        rl_avg_delta_tau_list   = []
        rl_has_ql = False  # True if at least one file has QL data
        
        for r in runs:
            log_print(f"File: {r['file']}")
            log_print(f"  Sent: {r['sent']}, Received: {r['recv']} -> PDR: {r['pdr']:.2f}%")
            log_print(f"  Avg Latency: {r['avg_latency']:.2f} ms")
            log_print(f"  NPC (Parent Changes): {r['parent_changes']}")
            log_print(f"  Total Overhead: {r['overhead']} (DIO: {r['dio']}, DAO: {r['dao']}, DIS: {r['dis']})")
            log_print(f"  Avg Energy/Node: {r['avg_energy']:.2f} mJ")

            # RL metrics per-file display
            rl_upd = r.get('rl_total_updates', 0)
            if rl_upd > 0:
                rl_has_ql = True
                sp   = r['rl_state_pct']
                log_print(f"  [RL] Updates: {rl_upd}  Decisions: {r['rl_total_decisions']}")
                log_print(f"  [RL] LAYER1: STAY={r['rl_layer1_stay']} SWITCH={r['rl_layer1_switch']}")
                log_print(f"  [RL] QL Overrides: proactive_switch={r['rl_proactive_switches']}  "
                          f"protective_stay={r['rl_protective_stays']}  "
                          f"override_rate={r['rl_override_rate']:.1f}%")
                log_print(f"  [RL] QL Uncertain: {r['rl_uncertain_count']}  "
                          f"Bootstrap skips: {r['rl_bootstrap_count']}")
                log_print(f"  [RL] Ruptures: {r['rl_rupture_count']}  "
                          f"Panic triggers: {r['rl_panic_count']}")
                log_print(f"  [RL] Avg Reward: {r['rl_avg_reward']:.2f}  "
                          f"(ETX component: {r['rl_avg_etx_reward']:.2f}  "
                          f"Stability bonus: {r['rl_avg_stab_reward']:.2f})")
                log_print(f"  [RL] Avg Delta-Tau: {r['rl_avg_delta_tau']:.1f}")
                log_print(f"  [RL] State visits: URGENCY={sp['URGENCY']:.1f}%  "
                          f"MINOR={sp['MINOR']:.1f}%  MAJOR={sp['MAJOR']:.1f}%")
                if r['rl_last_qtable']:
                    qt = r['rl_last_qtable']
                    log_print(f"  [RL] Final Q-Table (update #{qt['update']}):")
                    log_print(f"         URGENCY : STAY={qt['urg_stay']:+4d}  SWITCH={qt['urg_sw']:+4d}  "
                              f"=> {'SWITCH' if qt['urg_sw'] > qt['urg_stay'] else 'STAY'} preferred")
                    log_print(f"         MINOR   : STAY={qt['min_stay']:+4d}  SWITCH={qt['min_sw']:+4d}  "
                              f"=> {'SWITCH' if qt['min_sw'] > qt['min_stay'] else 'STAY'} preferred")
                    log_print(f"         MAJOR   : STAY={qt['maj_stay']:+4d}  SWITCH={qt['maj_sw']:+4d}  "
                              f"=> {'SWITCH' if qt['maj_sw'] > qt['maj_stay'] else 'STAY'} preferred")
                if r['rl_convergence_delta'] is not None:
                    log_print(f"  [RL] Q-Table total drift (first->last): {r['rl_convergence_delta']}")
            else:
                log_print(f"  [RL] No Q-Learning data found (QL not active or V2 without RL)")

            log_print("-" * 50)
            
            pdr_list.append(r['pdr'])
            latency_list.append(r['avg_latency'])
            pc_list.append(r['parent_changes'])
            overhead_list.append(r['overhead'])
            dio_list.append(r['dio'])
            dao_list.append(r['dao'])
            dis_list.append(r['dis'])
            energy_list.append(r['avg_energy'])

            if r.get('rl_total_updates', 0) > 0:
                rl_override_rate_list.append(r['rl_override_rate'])
                rl_avg_reward_list.append(r['rl_avg_reward'])
                rl_rupture_list.append(r['rl_rupture_count'])
                rl_panic_list.append(r['rl_panic_count'])
                rl_proactive_sw_list.append(r['rl_proactive_switches'])
                rl_protective_st_list.append(r['rl_protective_stays'])
                rl_avg_delta_tau_list.append(r['rl_avg_delta_tau'])
            
        # Calculate Averages and STD
        log_print(f"=== AVERAGE FOR TOPOLOGY {topo_id} ({len(runs)} simulations) ===")
        
        avg_pdr = statistics.mean(pdr_list) if pdr_list else 0
        std_pdr = statistics.stdev(pdr_list) if len(pdr_list) > 1 else 0
        
        avg_lat = statistics.mean(latency_list) if latency_list else 0
        std_lat = statistics.stdev(latency_list) if len(latency_list) > 1 else 0
        
        avg_pc = statistics.mean(pc_list) if pc_list else 0
        std_pc = statistics.stdev(pc_list) if len(pc_list) > 1 else 0
        
        avg_ovh = statistics.mean(overhead_list) if overhead_list else 0
        std_ovh = statistics.stdev(overhead_list) if len(overhead_list) > 1 else 0

        avg_dio = statistics.mean(dio_list) if dio_list else 0
        avg_dao = statistics.mean(dao_list) if dao_list else 0
        avg_dis = statistics.mean(dis_list) if dis_list else 0
        
        avg_egy = statistics.mean(energy_list) if energy_list else 0
        std_egy = statistics.stdev(energy_list) if len(energy_list) > 1 else 0
        
        log_print(f"Average PDR            : {avg_pdr:.2f}% ± {std_pdr:.2f}")
        log_print(f"Average Latency        : {avg_lat:.2f} ms ± {std_lat:.2f}")
        log_print(f"Average NPC            : {avg_pc:.1f} ± {std_pc:.1f}")
        log_print(f"Average Control Ovhd   : {avg_ovh:.1f} ± {std_ovh:.1f} (DIO: {avg_dio:.1f}, DAO: {avg_dao:.1f}, DIS: {avg_dis:.1f})")
        log_print(f"Average Energy/Node    : {avg_egy:.2f} mJ ± {std_egy:.2f}")

        # RL Summary (only if QL data was present)
        if rl_has_ql and rl_override_rate_list:
            log_print(f"")
            log_print(f"=== Q-LEARNING SUPERVISOR METRICS (TOPO {topo_id}) ===")

            def _avg(lst): return statistics.mean(lst) if lst else 0.0
            def _std(lst): return statistics.stdev(lst) if len(lst) > 1 else 0.0

            log_print(f"Avg QL Override Rate   : {_avg(rl_override_rate_list):.1f}% ± {_std(rl_override_rate_list):.1f}")
            log_print(f"  Proactive SWITCHes   : {_avg(rl_proactive_sw_list):.1f} ± {_std(rl_proactive_sw_list):.1f}")
            log_print(f"  Protective STAYs     : {_avg(rl_protective_st_list):.1f} ± {_std(rl_protective_st_list):.1f}")
            log_print(f"Avg Reward/cycle       : {_avg(rl_avg_reward_list):.2f} ± {_std(rl_avg_reward_list):.2f}")
            log_print(f"  (positive = agent improving, near +50 = fully stable links)")
            log_print(f"Rupture Events         : {_avg(rl_rupture_list):.1f} ± {_std(rl_rupture_list):.1f}")
            log_print(f"Panic Monitor Triggers : {_avg(rl_panic_list):.1f} ± {_std(rl_panic_list):.1f}")
            log_print(f"Avg Delta-Tau          : {_avg(rl_avg_delta_tau_list):.1f}")
            log_print(f"  (positive = agent consistently sees better candidates)")

    # Save to .txt file
    txt_path = os.path.join(directory, "evaluation_results.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(report))
    print(f"\n[+] Results successfully saved to: {txt_path}")

if __name__ == '__main__':
    analyze_logs(r"E:\3emeAnneeEMP\PFE\Implémentation\Results\203040SmartCity\OnlyTauDIO-RL-Lakhlef\5-stableLastVersion")
