import json
import sys
from pathlib import Path

def analyze_session(session_path):
    session = Path(session_path)
    
    if not session.exists():
        print(f"Path not found: {session}")
        return

    counts = {"router_found": 0, "timeout": 0, "flip": 0, "unknown": 0}
    total = 0

    for result_file in session.rglob("result.json"):
        with open(result_file) as f:
            data = json.load(f)
        reason = data.get("termination_reason", "unknown")
        counts[reason] = counts.get(reason, 0) + 1
        total += 1

    print(f"\nSession: {session.name}")
    print(f"Total runs: {total}")
    print(f"  Success (router_found): {counts['router_found']}  ({100*counts['router_found']/total:.1f}%)")
    print(f"  Timeout:                {counts['timeout']}  ({100*counts['timeout']/total:.1f}%)")
    print(f"  Flip:                   {counts['flip']}  ({100*counts['flip']/total:.1f}%)")
    if counts['unknown'] > 0:
        print(f"  Unknown:              {counts['unknown']}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 quick_session_analysis.py <session_folder>")
        sys.exit(1)
    analyze_session(sys.argv[1])