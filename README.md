Autonomous Explorer (NUS EE CDE2310)

This repository contains an autonomous frontier-based exploration system for a modified TurtleBot3. It integrates Google Cartographer SLAM with the Nav2 stack, utilizing a custom Python node to intelligently navigate and map unknown environments.
🛠 Project Structure

    run_explorer.py: The core "brain" of the exploration. It subscribes to /map and /odom, identifies frontiers, clusters them to reduce noise, and dispatches goals via a MapsToPose Action Client.

    nav2_params.yaml: Custom navigation configuration. It includes the critical asymmetrical footprint for the robot's launcher attachment and tuned Regulated Pure Pursuit parameters.

    explorer_launch.py: The master launch file that coordinates the cartographer_node, nav2_bringup, and the custom explorer node.

🧠 Exploration Logic

The explorer node follows a "Search -> Cluster -> Score -> Navigate" pipeline:

    Frontier Detection: Scans the OccupancyGrid for "Free" cells (0) that border "Unknown" space (-1).

    BFS Clustering: Groups individual frontier pixels into clusters. This prevents the robot from chasing single-pixel noise and helps it target significant "openings" in the map.

    Utility Scoring: Each cluster is evaluated using a weighted formula:
    Score=Distance+0.1Size1.5​

    This biases the robot toward large, unexplored areas while still prioritizing nearby targets to save battery.

    Hysteresis & Commitment: To prevent "stuttering," the script uses a SWITCH_THRESHOLD. It will only abandon its current goal if a new frontier is significantly better (3x the score).

    Stuck Recovery: If the robot's odometry remains static while a goal is active, the node blacklists that coordinate and re-plans.

⚙️ Hardware-Specific Tuning
Asymmetrical Footprint

The robot is equipped with a launcher extending to the left. The Nav2 costmap is configured with a custom polygon to ensure the launcher doesn't clip corners during tight turns:
YAML

robot radius is 0.15

SLAM Configuration

The system uses turtlebot3_cartographer. Note that due to the high computational overhead of Cartographer on laptop hardware, the nav2_params are tuned with a transform_tolerance of 1.0s to handle occasional LIDAR scan drops.
🚀 Installation & Usage
Prerequisites

    ROS 2 Humble / Iron

    TurtleBot3 Packages (burger model)

    Cartographer ROS 2 binaries

Build
Bash

cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select my_explorer
source install/setup.bash

Run
Bash

ros2 launch my_explorer explorer_launch.py
