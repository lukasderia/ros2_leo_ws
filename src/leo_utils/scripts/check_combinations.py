import json
import sys
from pathlib import Path

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
        if mode.startswith("rss"):
            mode = "rss"
        
        runs.append({
            "mode": mode,
            "termination_reason": data.get("termination_reason", "unknown"),
            "robot_start": tuple(data.get("robot_start", [])),
            "router_position": tuple(data.get("router_position", []))
        })

    # Get unique combinations and assign numbers
    combinations = sorted(set((r["robot_start"], r["router_position"]) for r in runs))
    combo_index = {combo: i+1 for i, combo in enumerate(combinations)}
    combo_lookup = {i+1: combo for i, combo in enumerate(combinations)}

    # Count non-flip runs per combination per mode
    counts = {}
    for r in runs:
        if r["termination_reason"] == "flip":
            continue
        combo_num = combo_index[(r["robot_start"], r["router_position"])]
        mode = r["mode"]
        counts.setdefault(combo_num, {"yamauchi": 0, "gao": 0, "rss": 0})
        counts[combo_num][mode] = counts[combo_num].get(mode, 0) + 1

    # Print summary
    print(f"\n{'Combo':<8} {'yamauchi':<12} {'gao':<8} {'rss':<8} {'robot_start':<20} {'router':<20}")
    print("-" * 72)
    for combo_num in sorted(counts.keys()):
        c = counts[combo_num]
        y = c.get("yamauchi", 0)
        g = c.get("gao", 0)
        r = c.get("rss", 0)
        robot, router = combo_lookup[combo_num]
        flag = " <-- NEEDS RUNS" if y < 3 or g < 3 else ""
        print(f"{combo_num:<8} {y:<12} {g:<8} {r:<8} {str(robot):<20} {str(router):<20}{flag}")

    print(f"\nTotal combinations: {len(combinations)}")
    print(f"Yamauchi missing (<3): {sum(1 for c in counts.values() if c.get('yamauchi',0) < 3)}")
    print(f"Gao missing (<3):      {sum(1 for c in counts.values() if c.get('gao',0) < 3)}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 check_combinations.py <root_folder>")
        sys.exit(1)
    analyze_combinations(sys.argv[1])