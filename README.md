# Custom DWA Planner

This repository contains the a custom Dynamic Window Node integrated in the turtlebot3 navigation. It replaces the default DWB controller with a lightweight DWA-style local planner that evaluates velocity samples, simulates trajectories, and selects the optimal command using weighted cost functions.

---

## Integration with Navigation

```bash
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_plugins: ["CustomDWAPlanner"]

    CustomDWAPlanner:
      plugin: "custom_dwa_planner::CustomDWAPlanner"
      desired_linear_vel: 0.5
      max_angular_vel: 1.0
      goal_weight: 1.0
      obstacle_weight: 1.0
      speed_weight: 0.5
      smoothness_weight: 0.5
```

---

## Update system packages

```bash
sudo apt update && sudo apt upgrade -y
```

## One time setup
```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
source ~/.bashrc
```

---


## 🛠️ Workspace Setup

1. **Create the ROS 2 workspace and source folder:**
    ```bash
    mkdir -p ~/planner_ws
    cd ~/planner_ws
    ```

2. **Clone the repository:**
    ```bash
    git clone https://github.com/reynash02/dwa_local_planner.git
    mv dwa_local_planner/ src
    ```
3. **Install Dependencies:**
    ```bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

4. **Build the workspace:**
    ```bash
    cd ~/planner_ws
    colcon build --symlink-install
    ```

4. **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

---

## 🚀 Robot Bringup

To start the robot with all required nodes:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```


This will launch:
- Turtlebot URDF in Gazebo environment
- Robot State Publisher

---

## 📍 Navigation

1. **Open a new terminal and:**
    ```bash
    source install/setup.bash
    ```

2. **Run localization using nav2_bringup:**
    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py
    ```

3. **In Rviz2:**
    - Click on 2D Pose Estimate and in the map choose approx location of the robot
    - Click the 2D Goal Pose tool
    - Click a point on the map to send the robot to that goal


---

## 📊 Visualization

1. Add a **Path** display in rviz.
2. Set the topic to ```/dwa_selected_trajectory ```
3. Choose ```map``` as fixed frame