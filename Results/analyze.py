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
    
    # Trafic de Données Utilisateur (Couche Application)
    # Match: "60000000:2:Client sending 1" -> t=60000000us, noeud=2, seq=1
    send_pattern = re.compile(r'^(\d+):(\d+):Client sending (\d+)')
    # Match: "61500000:1:Server received 1 from 2" -> t=61500000us, seq=1, source=2
    recv_pattern = re.compile(r'^(\d+):1:Server received (\d+) from (\d+)')
    
    # Trafic de Contrôle (Couche Routage RPL)
    dio_pattern = re.compile(r'Sending a multicast-DIO|Sending unicast-DIO')
    dao_pattern = re.compile(r'Sending a DAO|Sending a No-Path DAO')
    dis_pattern = re.compile(r'Sending a DIS')
    
    # TAU debug pattern for parent switch deduction
    tau_pattern = re.compile(r'^(\d+):(\d+):\[TAU\].*NPC=(\d+)')
    
    # Filename matching pattern
    fname_pattern = re.compile(r'TOPO-(\d+)-RS-(\d+)\.log')

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
        npc_dict = {}  # node_id -> last_npc
        
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

        # Moteur de parsing Ligne-par-Ligne (Mémoire efficiente)
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                
                # --- 1. DÉTECTION DE L'INSTABILITÉ (CHANGEMENTS DE PARENT) ---
                # On utilise la trace customisée [TAU] ajoutée dans le code C
                m_tau = tau_pattern.search(line)
                if m_tau:
                    node_id = int(m_tau.group(2))
                    npc_val = int(m_tau.group(3))  # Valeur brute pénalisée (accumulée)
                    last_npc = npc_dict.get(node_id, 0)
                    
                    if npc_val > last_npc:
                        # Déduction heuristique : la pénalité fait un bond lors d'un "parent switch"
                        # Historiquement = +50 ou +200 par saut. On vérifie le modulo pour déduire le nb de sauts nets.
                        diff = npc_val - last_npc
                        if diff % 200 == 0:
                            f_parent_changes += diff // 200
                        elif diff % 50 == 0:
                            f_parent_changes += diff // 50
                        else:
                            f_parent_changes += 1 # Règle de repli sécurisée
                    
                    npc_dict[node_id] = npc_val
                    continue
                
                # --- 2. TRANSMISSION APPLICATION (UDP SENDER) ---
                m_send = send_pattern.search(line)
                if m_send:
                    time_us = int(m_send.group(1))
                    node_id = int(m_send.group(2))
                    seq = int(m_send.group(3))
                    
                    # On mémorise le Timestamp (Microsecondes) pour calculer la Latence plus tard
                    sent_dict[(node_id, seq)] = time_us
                    f_sent += 1
                    continue
                
                # --- 3. RECEPTION APPLICATION (UDP RECEIVER / SINK) ---
                m_recv = recv_pattern.search(line)
                if m_recv:
                    time_us = int(m_recv.group(1))
                    seq = int(m_recv.group(2))
                    node_id = int(m_recv.group(3))
                    
                    # VÉRIFICATION : Si le paquet reçu était bien attendu (match Source + Numéro Séquence)
                    if (node_id, seq) in sent_dict:
                        time_sent = sent_dict[(node_id, seq)]
                        
                        # Calcul du Délai de Bout-en-Bout (End-to-end Latency) en Millisecondes
                        latency_ms = (time_us - time_sent) / 1000.0
                        if latency_ms >= 0:
                            f_latency_sum += latency_ms
                            f_latency_count += 1
                    
                    f_recv += 1
                    continue
                
                # --- 4. SURCOÛT DE CONTRÔLE (CONTROL OVERHEAD RPL) ---
                # Chaque Node émet des paquets "fantômes" pour entretenir la topologie.
                if "Sending a multicast-DIO" in line or "Sending unicast-DIO" in line:
                    f_dio += 1
                elif "Sending a DAO" in line or "Sending a No-Path DAO" in line:
                    f_dao += 1
                elif "Sending a DIS" in line:
                    f_dis += 1

        f_pdr = (f_recv / f_sent * 100) if f_sent > 0 else 0.0
        f_avg_latency = (f_latency_sum / f_latency_count) if f_latency_count > 0 else 0.0
        f_overhead = f_dio + f_dao + f_dis
        
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
            'dis': f_dis
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
        
        for r in runs:
            log_print(f"File: {r['file']}")
            log_print(f"  Sent: {r['sent']}, Received: {r['recv']} -> PDR: {r['pdr']:.2f}%")
            log_print(f"  Avg Latency: {r['avg_latency']:.2f} ms")
            log_print(f"  Parent Changes: {r['parent_changes']}")
            log_print(f"  Total Overhead: {r['overhead']} (DIO: {r['dio']}, DAO: {r['dao']}, DIS: {r['dis']})")
            log_print("-" * 50)
            
            pdr_list.append(r['pdr'])
            latency_list.append(r['avg_latency'])
            pc_list.append(r['parent_changes'])
            overhead_list.append(r['overhead'])
            
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
        
        log_print(f"Average PDR            : {avg_pdr:.2f}% ± {std_pdr:.2f}")
        log_print(f"Average Latency        : {avg_lat:.2f} ms ± {std_lat:.2f}")
        log_print(f"Average Parent Changes : {avg_pc:.1f} ± {std_pc:.1f}")
        log_print(f"Average Control Ovhd   : {avg_ovh:.1f} ± {std_ovh:.1f}")

    # Save to .txt file
    txt_path = os.path.join(directory, "evaluation_results.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(report))
    print(f"\n[+] Results successfully saved to: {txt_path}")

if __name__ == '__main__':
    analyze_logs(r"e:\3emeAnneeEMP\PFE\Implémentation\Results\Run12")
