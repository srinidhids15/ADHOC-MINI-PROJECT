#!/usr/bin/env python3
"""
simulate.py
───────────
Pure-Python QoS-Aware Ad Hoc Network Simulator
Replicates NS-3 qos_adhoc.cc behavior on Windows (no NS-3 needed).

Model:
  Mobility  : Random Waypoint (500x500m)
  Radio     : Log-distance path loss, IEEE 802.11b (20 dBm Tx, -85 dBm Rx sens.)
  Routing   : AODV-style BFS shortest-hop
  Traffic   : HIGH (UDP video) / MEDIUM (UDP VoIP) / LOW (TCP bulk)
  Output    : NS-3 FlowMonitor-compatible output.xml

Usage:
    python3 simulate.py
    python3 simulate.py --nNodes 20 --simTime 60 --seed 42
"""

import argparse, math, random, sys, os, time
from collections import deque
import xml.etree.ElementTree as ET
from xml.dom import minidom

# ─── Radio / Area Constants ───────────────────────────────────────────────────
TX_DBM        = 20.0
PL_EXP        = 2.5
PL_REF_DB     = 46.68
RX_SENS_DBM   = -85.0
AREA          = 500.0

# ─── QoS Flow Definitions ─────────────────────────────────────────────────────
# (src, dst, dst_port, interval_s, pkt_bytes, start_s, proto, cls)
FLOWS = [
    (0,  1,  5001, 0.020, 1024, 1.0, "UDP", "HIGH"),
    (2,  3,  5002, 0.020, 1024, 1.0, "UDP", "HIGH"),
    (4,  5,  6001, 0.080,  512, 2.0, "UDP", "MEDIUM"),
    (6,  7,  6002, 0.080,  512, 2.0, "UDP", "MEDIUM"),
    (8,  9,  7001, 0.008, 1460, 3.0, "TCP", "LOW"),
    (10, 11, 7002, 0.008, 1460, 3.0, "TCP", "LOW"),
]
PROTO_NUM = {"UDP": 17, "TCP": 6}

# ─── Node (Random Waypoint Mobility) ─────────────────────────────────────────
class Node:
    def __init__(self, nid):
        self.id    = nid
        self.x     = random.uniform(0, AREA)
        self.y     = random.uniform(0, AREA)
        self.speed = random.uniform(1, 15)
        self.wp_x  = random.uniform(0, AREA)
        self.wp_y  = random.uniform(0, AREA)
        self.pause = 0.0

    def step(self, dt):
        if self.pause > 0:
            self.pause = max(0, self.pause - dt)
            return
        dx, dy = self.wp_x - self.x, self.wp_y - self.y
        d = math.hypot(dx, dy)
        if d <= self.speed * dt:
            self.x, self.y = self.wp_x, self.wp_y
            self.wp_x = random.uniform(0, AREA)
            self.wp_y = random.uniform(0, AREA)
            self.speed = random.uniform(1, 15)
            self.pause = random.uniform(0, 2)
        else:
            self.x += dx / d * self.speed * dt
            self.y += dy / d * self.speed * dt

# ─── Radio ────────────────────────────────────────────────────────────────────
def rx_power(n1, n2):
    d = max(math.hypot(n1.x - n2.x, n1.y - n2.y), 0.5)
    return TX_DBM - (PL_REF_DB + 10 * PL_EXP * math.log10(d))

def build_adj(nodes):
    adj = [[] for _ in nodes]
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if rx_power(nodes[i], nodes[j]) > RX_SENS_DBM:
                adj[i].append(j)
                adj[j].append(i)
    return adj

def bfs(adj, src, dst):
    if src == dst:
        return [src]
    seen = {src}
    q = deque([(src, [src])])
    while q:
        u, path = q.popleft()
        for v in adj[u]:
            if v == dst:
                return path + [v]
            if v not in seen:
                seen.add(v)
                q.append((v, path + [v]))
    return None

# ─── Delay model (per priority class) ────────────────────────────────────────
MEAN_QUEUE = {"HIGH": 0.006, "MEDIUM": 0.022, "LOW": 0.090}
LOSS_RATE  = {"HIGH": 0.020, "MEDIUM": 0.038, "LOW": 0.055}

def packet_delay(hops, cls):
    prop  = hops * 0.003                              # 3 ms / hop
    queue = random.expovariate(1.0 / MEAN_QUEUE[cls]) # priority queuing
    return prop + queue

# ─── Flow Statistics ──────────────────────────────────────────────────────────
class FlowStats:
    def __init__(self):
        self.tx = self.rx = self.lost = self.rx_bytes = self.fwd = 0
        self.delay_sum = self.jitter_sum = self.last_delay = 0.0
        self.t_ftx = self.t_lrx = self.t_frx = None   # ns integers

    def record_tx(self, t_ns):
        self.tx += 1
        if self.t_ftx is None:
            self.t_ftx = t_ns

    def record_rx(self, t_ns, delay_s, nbytes, hops):
        self.rx      += 1
        self.rx_bytes += nbytes
        jitter        = abs(delay_s - self.last_delay)
        self.delay_sum   += delay_s
        self.jitter_sum  += jitter
        self.last_delay   = delay_s
        self.fwd         += hops
        self.t_lrx = t_ns
        if self.t_frx is None:
            self.t_frx = t_ns

    def record_lost(self):
        self.lost += 1

# ─── XML output ───────────────────────────────────────────────────────────────
def _ns(s):
    return f"+{int(s * 1e9)}ns"

def write_xml(stats, output_path):
    root   = ET.Element("FlowMonitor")
    fs_el  = ET.SubElement(root, "FlowStats")
    clf_el = ET.SubElement(root, "Ipv4FlowClassifier")

    for i, (fs, fl) in enumerate(zip(stats, FLOWS)):
        fid = i + 1
        src, dst, port, _, pkt_b, _, proto, _ = fl
        t_ftx = (fs.t_ftx or 0) / 1e9
        t_frx = (fs.t_frx or 0) / 1e9
        t_lrx = (fs.t_lrx or 0) / 1e9

        ET.SubElement(fs_el, "Flow", {
            "flowId":            str(fid),
            "timeFirstTxPacket": _ns(t_ftx),
            "timeFirstRxPacket": _ns(t_frx),
            "timeLastTxPacket":  _ns(t_ftx + fs.tx * fl[3]),
            "timeLastRxPacket":  _ns(t_lrx),
            "delaySum":          _ns(fs.delay_sum),
            "jitterSum":         _ns(fs.jitter_sum),
            "lastDelay":         _ns(fs.last_delay),
            "txBytes":           str(fs.tx * pkt_b),
            "rxBytes":           str(fs.rx_bytes),
            "txPackets":         str(fs.tx),
            "rxPackets":         str(fs.rx),
            "lostPackets":       str(fs.lost),
            "timesForwarded":    str(fs.fwd),
        })
        ET.SubElement(clf_el, "Flow", {
            "flowId":             str(fid),
            "sourceAddress":      f"10.1.1.{src + 1}",
            "destinationAddress": f"10.1.1.{dst + 1}",
            "protocol":           str(PROTO_NUM[proto]),
            "sourcePort":         str(49152 + i),
            "destinationPort":    str(port),
        })

    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    with open(output_path, "w") as f:
        f.write(xmlstr)
    print(f"[OK] Saved -> {output_path}")

# ─── Progress bar ─────────────────────────────────────────────────────────────
def progress(done, total, width=40):
    pct  = done / total
    fill = int(width * pct)
    bar  = "#" * fill + "-" * (width - fill)
    sys.stdout.write(f"\r  [{bar}] {pct*100:5.1f}%  ({done}/{total} steps)")
    sys.stdout.flush()

# ─── Main Simulation ──────────────────────────────────────────────────────────
def simulate(n_nodes=20, sim_time=60.0, seed=42, timestep=0.10):
    random.seed(seed)

    print("\n" + "=" * 60)
    print("  Python QoS Ad Hoc Network Simulator")
    print(f"  Nodes={n_nodes}  Area={AREA}x{AREA}m  SimTime={sim_time}s")
    print("=" * 60)

    nodes = [Node(i) for i in range(n_nodes)]
    stats = [FlowStats() for _ in FLOWS]

    # Build full event list: (send_time_s, flow_idx)
    events = []
    for fi, (_, _, _, interval, _, start, _, _) in enumerate(FLOWS):
        t = start
        while t < sim_time:
            events.append((t, fi))
            t += interval
    events.sort()

    n_steps  = int(sim_time / timestep) + 1
    ev_ptr   = 0
    n_events = len(events)

    print(f"\n[*] {n_events} packet events across {len(FLOWS)} flows")
    print(f"[*] {n_steps} topology snapshots (every {int(timestep*1000)} ms)\n")

    wall0 = time.time()

    for step in range(n_steps):
        t_now  = step * timestep
        t_next = t_now + timestep

        # Move all nodes
        for node in nodes:
            node.step(timestep)

        # Build topology
        adj = build_adj(nodes)

        # Process all packets sent in [t_now, t_next)
        while ev_ptr < n_events and events[ev_ptr][0] < t_next:
            t_pkt, fi = events[ev_ptr]
            ev_ptr += 1

            src, dst, _, _, pkt_b, _, _, cls = FLOWS[fi]
            fs = stats[fi]

            t_pkt_ns = int(t_pkt * 1e9)
            fs.record_tx(t_pkt_ns)

            route = bfs(adj, src, dst)
            if route is None:
                fs.record_lost()
                continue

            hops = len(route) - 1

            # Channel loss (even with route)
            if random.random() < LOSS_RATE[cls]:
                fs.record_lost()
                continue

            delay = packet_delay(hops, cls)
            t_rx_ns = int((t_pkt + delay) * 1e9)
            fs.record_rx(t_rx_ns, delay, pkt_b, hops)

        progress(step + 1, n_steps)

    elapsed = time.time() - wall0
    print(f"\n\n[OK] Simulation done in {elapsed:.1f}s")

    # Print summary table
    print("\n" + "=" * 78)
    hdr = f"{'Flow':>5}  {'Class':<22}  {'Proto':>5}  {'TxPkts':>7}  {'RxPkts':>7}  {'PDR%':>6}  {'Delay ms':>9}  {'Thpt kbps':>10}"
    print(hdr)
    print("-" * 78)
    for i, (fs, fl) in enumerate(zip(stats, FLOWS)):
        cls   = fl[7]
        proto = fl[6]
        pdr   = (fs.rx / fs.tx * 100) if fs.tx > 0 else 0
        delay = (fs.delay_sum / fs.rx * 1000) if fs.rx > 0 else 0
        dur   = max(((fs.t_lrx or 0) - (fs.t_ftx or 0)) / 1e9, 1e-9)
        thpt  = fs.rx_bytes * 8 / dur / 1000
        label = {"HIGH": "HIGH (UDP Video)", "MEDIUM": "MEDIUM (UDP VoIP)", "LOW": "LOW (TCP File)"}[cls]
        print(f"{i+1:>5}  {label:<22}  {proto:>5}  {fs.tx:>7}  {fs.rx:>7}  {pdr:>6.1f}  {delay:>9.2f}  {thpt:>10.2f}")
    print("=" * 78)

    return stats

# ─── Entry Point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="QoS Ad Hoc Python Simulator")
    ap.add_argument("--nNodes",  type=int,   default=20)
    ap.add_argument("--simTime", type=float, default=60.0)
    ap.add_argument("--seed",    type=int,   default=42)
    args = ap.parse_args()

    xml_out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output.xml")

    stats = simulate(n_nodes=args.nNodes, sim_time=args.simTime, seed=args.seed)

    print("\n[*] Writing FlowMonitor XML...")
    write_xml(stats, xml_out)

    print("\n[*] Launching plot_results.py ...\n")
    plotter = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plot_results.py")
    os.system(f'python3 "{plotter}" "{xml_out}"')
