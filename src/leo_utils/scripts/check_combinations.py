#!/usr/bin/env python3
import json
import sys
from pathlib import Path

num = 4

def analyze_combinations(root_path):
    root = Path(root_path)
    
    if not root.exists():
        print(f"Path not found: {root}")
        return

    # Collect all runs from all result.json files
    runs = []
    for result_file in root.rglob("result.json"):
        with open(result_file) as f:
            data = json.load(f)
        
        mode = data.get("mode", "unknown")
        # Keep rss modes separate (rss_1, rss_2, rss_3, rss_4)

        runs.append({
            "mode": mode,
            "termination_reason": data.get("termination_reason", "unknown"),
            "robot_start": tuple(data.get("robot_start", [])),
            "router_position": tuple(data.get("router_position", []))
        })

    # Get unique combinations and assign numbers
    combinations = sorted(set((r["robot_start"], r["router_position"]) for r in runs))
    combo_index  = {combo: i+1 for i, combo in enumerate(combinations)}
    combo_lookup = {i+1: combo for i, combo in enumerate(combinations)}

    # All modes present
    rss_modes = sorted(set(r["mode"] for r in runs if r["mode"].startswith("rss")))
    all_modes = ["yamauchi", "gao"] + rss_modes

    # Count non-flip runs per combination per mode
    counts = {}
    for r in runs:
        if r["termination_reason"] == "flip":
            continue
        combo_num = combo_index[(r["robot_start"], r["router_position"])]
        mode = r["mode"]
        if combo_num not in counts:
            counts[combo_num] = {m: 0 for m in all_modes}
        counts[combo_num][mode] = counts[combo_num].get(mode, 0) + 1

    # Column widths
    col_w = 8

    # Header
    header = f"{'Combo':<8}"
    for m in all_modes:
        header += f"{m:<{col_w}}"
    header += f"  {'robot_start':<22} {'router'}"
    print(f"\n{header}")
    print("-" * (8 + col_w * len(all_modes) + 46))

    for combo_num in sorted(counts.keys()):
        c = counts[combo_num]
        robot, router = combo_lookup[combo_num]
        row = f"{combo_num:<8}"
        for m in all_modes:
            row += f"{c.get(m, 0):<{col_w}}"
        row += f"  {str(robot):<22} {str(router)}"
        print(row)

    print(f"\nTotal combinations: {len(combinations)}")
    print(f"Yamauchi missing (< {num}): {sum(1 for c in counts.values() if c.get('yamauchi', 0) < num)}")
    print(f"Gao missing (< {num}):      {sum(1 for c in counts.values() if c.get('gao', 0) < 3)}")
    for rm in rss_modes:
        print(f"{rm} missing (< {num}):   {sum(1 for c in counts.values() if c.get(rm, 0) < num)}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 check_combinations.py <root_folder>")
        sys.exit(1)
    analyze_combinations(sys.argv[1])