#!/usr/bin/env python3
import json
import argparse
from pathlib import Path

def make_edge_set(nodes, directed=True):
    s = set()
    for i in range(len(nodes)-1):
        u = int(nodes[i]); v = int(nodes[i+1])
        if directed:
            s.add((u, v))
        else:
            s.add((u, v) if u <= v else (v, u))
    return s

def compute_penalty_one_result(paths, overlap_threshold_frac, directed=True):
    if not paths:
        return 0.0

    shortest = float(paths[0]["length"])
    k = len(paths)

    edge_sets = [make_edge_set(p["path"], directed=directed) for p in paths]
    dist_pen = [((float(p["length"]) - shortest) / shortest) + 0.1 for p in paths]

    overlap_count = [0] * k
    for i in range(k):
        Ei = edge_sets[i]
        for j in range(i, k):
            Ej = edge_sets[j]
            common = len(Ei & Ej) if Ei and Ej else 0
            overlapi = (common / len(Ei)) if Ei else 0.0
            overlapj = (common / len(Ej)) if Ej else 0.0

            if overlapi > overlap_threshold_frac:
                overlap_count[i] += 1
            if overlapj > overlap_threshold_frac:
                overlap_count[j] += 1

            if i == j:
                overlap_count[i] -= 1

    total = 0.0
    for i in range(k):
        total += overlap_count[i] * dist_pen[i]
    return total

def main():
    ap = argparse.ArgumentParser(description="Compute total penalty from output.json (C++ computePenalty equivalent).")
    ap.add_argument("output_json", help="Path to output.json")
    ap.add_argument("--overlap-threshold", type=float, default=0.6,
                    help="Overlap threshold. If >1, treated as percent (e.g., 60 -> 0.6). Default 0.6.")
    ap.add_argument("--undirected", action="store_true",
                    help="Treat edges as undirected (normalize (u,v) == (v,u)); default directed.")
    ap.add_argument("--per-query", action="store_true",
                    help="Print per-query penalties.")
    args = ap.parse_args()

    thr = args.overlap_threshold
    if thr > 1.0:
        thr = thr / 100.0

    data = json.loads(Path(args.output_json).read_text())

    if isinstance(data, dict):
        results = data.get("results", data.get("events", []))
    else:
        results = data

    total_pen = 0.0
    perq_vals = []
    for item in results:
        paths = item.get("paths", [])
        pen = compute_penalty_one_result(paths, thr, directed=not args.undirected)
        total_pen += pen
        perq_vals.append({"id": item.get("id"), "penalty": pen, "k": len(paths)})

    print(f"Total penalty: {total_pen:.6f}")
    if args.per_query:
        for row in perq_vals:
            print(f"id={row['id']}, k={row['k']}, penalty={row['penalty']:.6f}")

if __name__ == "__main__":
    main()
