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
                # Chaque Node émet des paquets "fantômes" pour entretenir la topologie.
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
            'avg_energy': f_avg_energy
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
        
        for r in runs:
            log_print(f"File: {r['file']}")
            log_print(f"  Sent: {r['sent']}, Received: {r['recv']} -> PDR: {r['pdr']:.2f}%")
            log_print(f"  Avg Latency: {r['avg_latency']:.2f} ms")
            log_print(f"  NPC (Parent Changes): {r['parent_changes']}")
            log_print(f"  Total Overhead: {r['overhead']} (DIO: {r['dio']}, DAO: {r['dao']}, DIS: {r['dis']})")
            log_print(f"  Avg Energy/Node: {r['avg_energy']:.2f} mJ")
            log_print("-" * 50)
            
            pdr_list.append(r['pdr'])
            latency_list.append(r['avg_latency'])
            pc_list.append(r['parent_changes'])
            overhead_list.append(r['overhead'])
            dio_list.append(r['dio'])
            dao_list.append(r['dao'])
            dis_list.append(r['dis'])
            energy_list.append(r['avg_energy'])
            
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

    # Save to .txt file
    txt_path = os.path.join(directory, "evaluation_results.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(report))
    print(f"\n[+] Results successfully saved to: {txt_path}")

if __name__ == '__main__':
    analyze_logs(r"e:\3emeAnneeEMP\PFE\Implémentation\Results\S1miniPrj")
