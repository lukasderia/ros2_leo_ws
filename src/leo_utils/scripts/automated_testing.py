import subprocess
import time
import os
import signal

MODE = ['rss_1', 'rss_2', 'rss_3', 'rss_4']

COMBINATIONS = [
    # robot (-19, -19)
    (-19.0, -19.0,  19.0, -19.0),
    (-19.0, -19.0,  -8.0, -18.0),
    (-19.0, -19.0, -19.0,  19.0),
    (-19.0, -19.0,  18.0,  18.0),
    (-19.0, -19.0,   8.0,   0.0),
    # robot (19, -19)
    ( 19.0, -19.0, -19.0, -19.0),
    ( 19.0, -19.0,  -8.0, -18.0),
    ( 19.0, -19.0, -19.0,  19.0),
    ( 19.0, -19.0,  18.0,  18.0),
    ( 19.0, -19.0,   8.0,   0.0),
    # robot (0, 0)
    (  0.0,   0.0, -19.0, -19.0),
    (  0.0,   0.0,  19.0, -19.0),
    (  0.0,   0.0,  -8.0, -18.0),
    (  0.0,   0.0, -19.0,  19.0),
    (  0.0,   0.0,  18.0,  18.0),
    (  0.0,   0.0,   8.0,   0.0),
    # robot (-19, 19)
    (-19.0,  19.0, -19.0, -19.0),
    (-19.0,  19.0,  19.0, -19.0),
    (-19.0,  19.0,  -8.0, -18.0),
    (-19.0,  19.0,  18.0,  18.0),
    (-19.0,  19.0,   8.0,   0.0),
    # robot (19, 19)
    ( 19.0,  19.0, -19.0, -19.0),
    ( 19.0,  19.0,  19.0, -19.0),
    ( 19.0,  19.0,  -8.0, -18.0),
     ( 19.0,  19.0, -19.0,  19.0),
    ( 19.0,  19.0,   8.0,   0.0),
]

COOLDOWN = 10  # seconds between runs

def run_combination(robot_x, robot_y, router_x, router_y, mode):
    print(f"\n--- Starting run: robot=({robot_x},{robot_y}) router=({router_x},{router_y}) mode={mode} ---")
    
    stddev = mode.split('_')[1]
    
    process = subprocess.Popen(
        ['ros2', 'launch', 'leo_bringup', 'sim_testing.launch.py',
         f'robot_x:={robot_x}', f'robot_y:={robot_y}',
         f'router_x:={router_x}', f'router_y:={router_y}',
         f'stddev:={stddev}'],
        preexec_fn=os.setsid
    )

    try:
        subprocess.run(
            f'ros2 run leo_utils recorder.py --ros-args -p robot_x:={robot_x} -p robot_y:={robot_y} -p router_x:={router_x} -p router_y:={router_y} -p mode:={mode}',
            shell=True
        )
    finally:
        print("Shutting down simulation...")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait()
        print(f"--- Run complete: robot=({robot_x},{robot_y}) router=({router_x},{router_y}) ---")

def main():
    for run in MODE:
        stddev = run.split('_')[1]
        for repeat in range(3):
            total = len(COMBINATIONS)
            for i, (robot_x, robot_y, router_x, router_y) in enumerate(COMBINATIONS):
                print(f"\n[Mode: {run} | StdDev: {stddev} | Repeat: {repeat+1}/3 | Combination: {i+1}/{total}]")
                print(f"  Robot: ({robot_x}, {robot_y}) | Router: ({router_x}, {router_y})")
                run_combination(robot_x, robot_y, router_x, router_y, run)
                
                if i < total - 1:
                    print(f"  Cooldown {COOLDOWN}s before next run...")
                    time.sleep(COOLDOWN)
        
        print(f"\n--- All runs complete for mode: {run} (stddev={stddev}) ---")
    
    print("\n=== All modes complete! ===")

if __name__ == '__main__':
    main()