import subprocess
import time
import os
import signal

def main():
    # Step 1: Launch the system
    process = subprocess.Popen(
        ['ros2', 'launch', 'leo_bringup', 'sim_testing.launch.py',
         'robot_x:=-1.0', 'robot_y:=0.0', 'router_x:=1.0', 'router_y:=13.0'],
        preexec_fn=os.setsid  # Creates a process group so we can kill everything later
    )

    # Step 2: Wait for everything to initialize
    print("Waiting for system to initialize...")
    time.sleep(10)

    # Step 3: Enable auto mode
    print("Enabling auto mode...")
    subprocess.run([
        'ros2', 'topic', 'pub', '-1',
        '/auto_mode', 'std_msgs/msg/Bool', '{"data": true}',
        '--qos-durability', 'transient_local'
    ])

    # Step 4: Wait for exploration to finish or timeout
    print("Waiting for exploration to complete...")
    start_time = time.time()
    max_duration = 8 * 60  # 15 minutes in seconds
    
    while (time.time() - start_time) < max_duration:
            try:
                result = subprocess.run(
                    ['ros2', 'topic', 'echo', '-1', '/exploration_complete', 'std_msgs/msg/Bool'],
                    capture_output=True, text=True, timeout=2
                )
                if 'data: true' in result.stdout:
                    print("Exploration complete - router found!")
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(1)
    else:
        print("Timeout reached - stopping exploration")

    # Step 5: Disable auto mode
    print("Disabling auto mode...")
    subprocess.run([
        'ros2', 'topic', 'pub', '-1',
        '/auto_mode', 'std_msgs/msg/Bool', '{"data": false}',
        '--qos-durability', 'transient_local'
    ])

    # Step 6: Kill the launcher and all child processes
    print("Shutting down...")
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    process.wait()
    print("Done")

if __name__ == '__main__':
    main()