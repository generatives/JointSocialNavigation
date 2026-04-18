# ROS2 Workspace
Workspace for projects from [MARS Lab](https://sfumars.com/) running **ROS2 Humble**
- Originally fork of https://github.com/athackst/vscode_ros2_workspace/tree/humble-nvidia

## Requirements
- Ubuntu 22.04 LTS
- Nvidia driver 555 with CUDA 12.5

## Setup
### Host 
If compatible Nvidia GPU is available:
- See `setup/host_gpu_driver_setup.sh`

If compatible Nvidia GPU is not available:
- Switch to the `no_cuda` branch

### Dev container
See [general instructions for using workspaces with docker](https://github.com/SFU-MARS/ros2_tutorial/wiki/Building-and-using-the-dev-container).

## Running the simulation
1. Launch human simulator: 
    - Settings are in `hunavis/params/hunavsim.yaml`
```bash
ros2 launch hunavis mars.launch.py
```
![Human and robot in an empty room](images/human_robot_gazebo.png)

2. Launch nav2: 
    - Settings are in `hunavis/params/tb3_custom_sim.yaml`
```bash
ros2 launch hunavis tb3_custom_sim.launch.py
```

3. Launch human detection (including loading rviz)
```bash
ros2 launch hunavis hudet.launch.py
```
![Rviz display](images/human_robot_rviz.png)

#### Known issues
- **If the simulator doesn't run properly**, killing all processes that still are running after `^C` may help. 
    - Execute the following line to kill the likely remaining processes:
    - Run `ps -a` to verify no processes remain. If any remain, kill them one by one until `ps -a` shows they are gone.
    - Double check that these processes are not running both within the docker container and on the host.
```bash
pkill -9 gazebo && pkill -9 gzclient && pkill -9 gzserver && pkill -9 ros2 && pkill -9 python3
```
    
- **If you get an permission denied error** like `error creating runtime directory '/run/user/1001' (permission denied)`:
  - Run the following:
  - Lines 2 and 3 ensure ros has ownership of the runtime directory. The last line sets up the Gazebo environment.
```bash
source ~/.bashrc
sudo chmod -R 700 /run/user
sudo chown -R ros:ros /run/user
source /usr/share/gazebo/setup.sh
```
- **If the initial `colcon build` fails**, it is likely due to dependency issues.
    - Try running
```bash
sudo apt-get update
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
```


## Social Simulator

First build:
```
colcon build --symlink-install --packages-skip zed_isaac_ros_nitros_sub zed_isaac_ros_april_tag
colcon build --symlink-install --packages-select social_simulator
```

To run the simulator (I suggest to use tmux):
```
ros2 launch social_simulator social_simulator.launch.py scenario:=agents_wide_hallway.yaml
```

### DEBUGGING: 

To run each launch file separately
```
ros2 launch social_simulator hunavsim.launch.py world:=wide_hallway.world scenario:=agents_wide_hallway.yaml
```

```
ros2 launch social_simulator tb3_custom_sim.launch.py map:=wide_hallway.yaml scenario:=agents_wide_hallway.yaml
```

```
ros2 launch social_simulator hudet.launch.py scenario_params_file:=agents_wide_hallway.yaml
```

```
ros2 run social_navigation navigator
```

### To Run Evaluations

Start the simulator with the correct map and scenario
```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_1_ahead_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

Start the evaluation with the right scenario
```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_1_ahead_robot.yaml experiment_tag:=test_run run_id:=0
```

### To start HuNavSim Evaluator:


```
ros2 launch hunav_rviz2_panel hunav_rviz2_launch.py map:=/workspaces/JointSocialNavigation/src/social_simulator/maps/wide_hallway.yaml
```


Start evaluator launch
```
ros2 launch hunav_evaluator hunav_evaluator_launch.py metrics_file:=/workspaces/JointSocialNavigation/src/social_simulator/config/metrics.yaml
```

Start recording
```
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation "{robot_goal: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, experiment_tag: 'wide_hallway', run_id: 1}"
```

Stop recording
```
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```


## Narrower Hallway

### JointMCTS

```
ros2 launch social_simulator social_simulator.launch.py world:=narrower_hallway.world scenario:=narrower_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrower_hallway/agents_2_towards_robot.yaml experiment_tag:=test_run run_id:=0
```

```
ros2 launch hunav_evaluator hunav_evaluator_launch.py metrics_file:=/workspaces/JointSocialNavigation/src/social_simulator/config/metrics.yaml
```

### Nav2 

```
ros2 launch social_simulator social_simulator.launch.py world:=narrower_hallway.world scenario:=narrower_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose'
```

```
ros2 launch social_simulator social_simulator.launch.py world:=narrower_hallway.world scenario:=narrower_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' use_humans:=False
```


## Teleop command

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
