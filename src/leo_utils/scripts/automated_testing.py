import subprocess
import time
import os
import signal

ROBOT_X = 0.0
ROBOT_Y = 0.0
ROUTER_X = 0.0
ROUTER_Y = 13.0
MODE = 'rss'

def main():
    # Launch simulation
    process = subprocess.Popen(
        ['ros2', 'launch', 'leo_bringup', 'sim_testing.launch.py',
         f'robot_x:={ROBOT_X}', f'robot_y:={ROBOT_Y}',
         f'router_x:={ROUTER_X}', f'router_y:={ROUTER_Y}'],
        preexec_fn=os.setsid
    )

    # Run recorder node (blocks until done)
    subprocess.run(
        f'ros2 run leo_utils recorder.py --ros-args -p robot_x:={ROBOT_X} -p robot_y:={ROBOT_Y} -p router_x:={ROUTER_X} -p router_y:={ROUTER_Y} -p mode:={MODE}',
        shell=True
    )

    # Kill simulation
    print("Shutting down simulation...")
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    process.wait()
    print("Done")

if __name__ == '__main__':
    main()