import os
import json
import sys
from collections import Counter

def count_terminations(bags_root):
    counts = Counter()
    sessions = [d for d in os.listdir(bags_root) 
                if os.path.isdir(os.path.join(bags_root, d)) and d.startswith("session_")]
    
    for session in sorted(sessions):
        session_path = os.path.join(bags_root, session)
        session_counts = Counter()
        
        for run_folder in os.listdir(session_path):
            result_path = os.path.join(session_path, run_folder, "result.json")
            if not os.path.exists(result_path):
                continue
            with open(result_path) as f:
                data = json.load(f)
            reason = data.get("termination_reason", "unknown")
            session_counts[reason] += 1
            counts[reason] += 1
        
        print(f"\n{session}:")
        for reason, count in session_counts.items():
            print(f"  {reason}: {count}")
    
    print(f"\nTotal across all sessions:")
    for reason, count in counts.items():
        print(f"  {reason}: {count}")

if __name__ == "__main__":
    bags_root = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/ros2_leo_ws/bags")
    count_terminations(bags_root)