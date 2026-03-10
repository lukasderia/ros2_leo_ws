import os
import json
import sys
from collections import Counter

def count_terminations(session_path):
    counts = Counter()

    for run_folder in os.listdir(session_path):
        result_path = os.path.join(session_path, run_folder, "result.json")
        if not os.path.exists(result_path):
            continue
        with open(result_path) as f:
            data = json.load(f)
        reason = data.get("termination_reason", "unknown")
        counts[reason] += 1

    print(f"\nResults for: {os.path.basename(session_path)}")
    for reason, count in sorted(counts.items()):
        print(f"  {reason}: {count}")
    print(f"  total: {sum(counts.values())}")

if __name__ == "__main__":
    session_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/ros2_leo_ws/bags/session_rss_4")
    count_terminations(session_path)