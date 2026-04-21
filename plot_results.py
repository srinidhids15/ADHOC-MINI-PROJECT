#!/usr/bin/env python3
"""
plot_results.py
───────────────
Parse NS-3 FlowMonitor output.xml and produce publication-quality graphs:
  1. Throughput per traffic class (bar chart)
  2. Mean End-to-End Delay per traffic class (bar chart)
  3. Packet Delivery Ratio per traffic class (bar chart)
  4. Combined comparison (grouped bar)

Usage:
    python3 plot_results.py              # reads output.xml in current dir
    python3 plot_results.py output.xml   # explicit path

Requirements:
    pip install matplotlib numpy
"""

import sys
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os

matplotlib.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 11,
    "axes.titlesize": 13,
    "axes.labelsize": 12,
    "figure.dpi": 120,
})

# ─── Port → Class Mapping (must match C++ code) ──────────────────────────────
PORT_CLASS = {
    5001: "HIGH\n(UDP Video)",
    5002: "HIGH\n(UDP Video)",
    6001: "MEDIUM\n(UDP VoIP)",
    6002: "MEDIUM\n(UDP VoIP)",
    7001: "LOW\n(TCP File)",
    7002: "LOW\n(TCP File)",
}

CLASS_ORDER  = ["HIGH\n(UDP Video)", "MEDIUM\n(UDP VoIP)", "LOW\n(TCP File)"]
CLASS_COLORS = ["#E63946", "#457B9D", "#2A9D8F"]

# ─── XML Parsing ─────────────────────────────────────────────────────────────

def parse_flowmonitor(xml_path: str):
    """Return list of dicts with per-flow metrics."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Locate <FlowStats> and <Ipv4FlowClassifier>
    flow_stats_elem = root.find("FlowStats")
    classifier_elem  = root.find("Ipv4FlowClassifier")

    if flow_stats_elem is None or classifier_elem is None:
        raise ValueError("Cannot find FlowStats or Ipv4FlowClassifier in XML.")

    # Build FlowId → 5-tuple dict
    flow_map = {}
    for flow_el in classifier_elem.findall("Flow"):
        fid   = int(flow_el.get("flowId"))
        dport = int(flow_el.get("destinationPort"))
        proto = int(flow_el.get("protocol"))
        flow_map[fid] = {
            "dport": dport,
            "protocol": "TCP" if proto == 6 else "UDP",
            "src": flow_el.get("sourceAddress"),
            "dst": flow_el.get("destinationAddress"),
        }

    results = []
    for fs in flow_stats_elem.findall("Flow"):
        fid = int(fs.get("flowId"))
        if fid not in flow_map:
            continue

        tx_pkts  = int(fs.get("txPackets",  0))
        rx_pkts  = int(fs.get("rxPackets",  0))
        rx_bytes = int(fs.get("rxBytes",    0))
        lost     = int(fs.get("lostPackets", 0))

        # delaySum is stored as "+1.23456e+09ns" style
        delay_sum_ns_str = fs.get("delaySum", "0ns").replace("+", "").replace("ns", "")
        try:
            delay_sum_s = float(delay_sum_ns_str) * 1e-9
        except ValueError:
            delay_sum_s = 0.0

        first_tx_str = fs.get("timeFirstTxPacket", "0ns").replace("+", "").replace("ns", "")
        last_rx_str  = fs.get("timeLastRxPacket",  "0ns").replace("+", "").replace("ns", "")
        try:
            first_tx_s = float(first_tx_str) * 1e-9
            last_rx_s  = float(last_rx_str)  * 1e-9
        except ValueError:
            first_tx_s = last_rx_s = 0.0

        duration = max(last_rx_s - first_tx_s, 1e-9)

        throughput_kbps = (rx_bytes * 8.0) / (duration * 1000.0)
        mean_delay_ms   = (delay_sum_s / rx_pkts * 1000.0) if rx_pkts > 0 else 0.0
        pdr             = (rx_pkts / tx_pkts * 100.0)      if tx_pkts > 0 else 0.0

        dport  = flow_map[fid]["dport"]
        tclass = PORT_CLASS.get(dport, "Other")

        results.append({
            "flow_id":    fid,
            "class":      tclass,
            "protocol":   flow_map[fid]["protocol"],
            "tx_pkts":    tx_pkts,
            "rx_pkts":    rx_pkts,
            "lost":       lost,
            "throughput": throughput_kbps,
            "delay":      mean_delay_ms,
            "pdr":        pdr,
        })

    return results


def aggregate_by_class(results):
    """Average metrics grouped by traffic class."""
    agg = {c: {"throughput": [], "delay": [], "pdr": []} for c in CLASS_ORDER}
    for r in results:
        c = r["class"]
        if c in agg:
            agg[c]["throughput"].append(r["throughput"])
            agg[c]["delay"].append(r["delay"])
            agg[c]["pdr"].append(r["pdr"])

    summary = {}
    for c in CLASS_ORDER:
        vals = agg[c]
        summary[c] = {
            "throughput": np.mean(vals["throughput"]) if vals["throughput"] else 0,
            "delay":      np.mean(vals["delay"])      if vals["delay"]      else 0,
            "pdr":        np.mean(vals["pdr"])        if vals["pdr"]        else 0,
        }
    return summary


def bar_chart(ax, values, title, ylabel, color_idx=None, unit=""):
    """Generic horizontal-style bar chart on given Axes."""
    labels  = CLASS_ORDER
    heights = [values[c] for c in CLASS_ORDER]
    colors  = [CLASS_COLORS[i] for i in range(len(CLASS_ORDER))]

    bars = ax.bar(labels, heights, color=colors, width=0.5, edgecolor="white", linewidth=1.2)
    ax.set_title(title, fontweight="bold", pad=10)
    ax.set_ylabel(ylabel)
    ax.set_ylim(0, max(heights) * 1.25 if max(heights) > 0 else 1)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # Annotate bars
    for bar, h in zip(bars, heights):
        ax.text(bar.get_x() + bar.get_width() / 2.0,
                bar.get_height() + max(heights) * 0.02,
                f"{h:.2f}{unit}",
                ha="center", va="bottom", fontsize=10, fontweight="bold")
    return bars


def plot_combined(summary, output_dir="."):
    """4-panel figure."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("QoS-Aware Ad Hoc Network — Performance Metrics\n(NS-3 FlowMonitor Results)",
                 fontsize=14, fontweight="bold", y=1.02)

    throughputs = {c: summary[c]["throughput"] for c in CLASS_ORDER}
    delays      = {c: summary[c]["delay"]      for c in CLASS_ORDER}
    pdrs        = {c: summary[c]["pdr"]        for c in CLASS_ORDER}

    bar_chart(axes[0], throughputs, "Throughput by Traffic Class",  "Throughput (kbps)", unit=" kbps")
    bar_chart(axes[1], delays,      "Avg End-to-End Delay",         "Delay (ms)",        unit=" ms")
    bar_chart(axes[2], pdrs,        "Packet Delivery Ratio (PDR)",  "PDR (%)",           unit="%")

    plt.tight_layout()
    out_path = os.path.join(output_dir, "qos_results.png")
    plt.savefig(out_path, bbox_inches="tight", dpi=150)
    print(f"[✓] Saved combined chart → {out_path}")
    plt.show()


def plot_per_flow(results, output_dir="."):
    """Per-flow throughput bar chart."""
    fig, ax = plt.subplots(figsize=(12, 5))

    flow_ids    = [r["flow_id"]    for r in results]
    throughputs = [r["throughput"] for r in results]
    classes     = [r["class"]      for r in results]

    x_labels = [f"F{fid}\n{cls.split(chr(10))[0]}" for fid, cls in zip(flow_ids, classes)]
    colors   = [CLASS_COLORS[CLASS_ORDER.index(c)] if c in CLASS_ORDER else "#999"
                for c in classes]

    bars = ax.bar(x_labels, throughputs, color=colors, edgecolor="white", linewidth=1.2)
    ax.set_title("Per-Flow Throughput", fontweight="bold")
    ax.set_ylabel("Throughput (kbps)")
    ax.set_xlabel("Flow ID")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    for bar, h in zip(bars, throughputs):
        ax.text(bar.get_x() + bar.get_width() / 2.0,
                bar.get_height() + 1,
                f"{h:.1f}", ha="center", va="bottom", fontsize=9)

    # Legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=CLASS_COLORS[i], label=CLASS_ORDER[i].replace("\n", " "))
                       for i in range(len(CLASS_ORDER))]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()
    out_path = os.path.join(output_dir, "per_flow_throughput.png")
    plt.savefig(out_path, bbox_inches="tight", dpi=150)
    print(f"[✓] Saved per-flow chart → {out_path}")
    plt.show()


def print_table(results):
    """Print a neat ASCII table of per-flow results."""
    print("\n" + "=" * 80)
    print(f"{'FlowID':>6}  {'Class':<22}  {'Proto':>5}  "
          f"{'TxPkts':>8}  {'RxPkts':>8}  {'PDR%':>6}  "
          f"{'Delay(ms)':>10}  {'Thpt(kbps)':>12}")
    print("-" * 80)
    for r in sorted(results, key=lambda x: x["flow_id"]):
        cls = r["class"].replace("\n", " ")
        print(f"{r['flow_id']:>6}  {cls:<22}  {r['protocol']:>5}  "
              f"{r['tx_pkts']:>8}  {r['rx_pkts']:>8}  {r['pdr']:>6.1f}  "
              f"{r['delay']:>10.3f}  {r['throughput']:>12.2f}")
    print("=" * 80 + "\n")


# ─── Main ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    xml_path = sys.argv[1] if len(sys.argv) > 1 else "output.xml"

    if not os.path.exists(xml_path):
        print(f"[ERROR] File not found: {xml_path}")
        print("  Run NS-3 simulation first, then re-run this script.")
        sys.exit(1)

    print(f"[*] Parsing FlowMonitor XML: {xml_path}")
    results = parse_flowmonitor(xml_path)

    if not results:
        print("[WARNING] No flows found in XML. Check simulation output.")
        sys.exit(1)

    print(f"[*] Found {len(results)} flows.")
    print_table(results)

    summary = aggregate_by_class(results)

    output_dir = os.path.dirname(os.path.abspath(xml_path))
    plot_combined(summary, output_dir)
    plot_per_flow(results,  output_dir)

    print("[✓] All graphs saved successfully.")
