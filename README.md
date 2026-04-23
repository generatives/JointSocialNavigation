# Joint Social Navigation with MCTS and SFM

## Overview
This repository contains a socially aware planning system for a mobile robot. It uses Monte Carlo Tree Search and the Social Force Model to find trajectories that minimize human impact while effectively guiding the robot to its' goal.

We apply a modified version of Monte Carlo Tree Search which uses Decoupled UCT, PUCT, and Progressive Widening to better model the social plannning process and manage the infinite branching factor of a continous action space.

The repository contains the core algorithm, a simple PyGame simulator for early testing, a ROS frontend, a HuNavSim evaluation system and set of scenarios.

## Tour of the Codebase
The core system is contained in the "social_navigation" ROS module inside the src folder.
- social_navigation/mcts/decoupled_mcts.py contains an implementation of MCTS with Decoupled UCT, PUCT, and Progressive Widening over sampled actions
- social_navigation/simulator/mcts_game_state.py contains the application specific logic, including value functions, action probabilities, and action sampling
- social_navigation/simulator and social_navigation/rendering contain code for the PyGame simulator
- social_navigation/navigator.py contains the ROS2 frontend, which operates global path finding, runs the local MCTS planner, and the pure pursuit controller

The "social_simulator" module contains launch files for the simulator and evaluation. It also contains all the test scenarios and a simple ROS Node for launching the evaluation.

At runtime the system is setup as follows:
- The ROS Node navigator.py receives the goal, human state information, and a cost map for static obstacles.
- The Node runs A* to get a global path to the goal.
- Every 500ms the Node runs the MCTS planner to get a local plan for the next 500ms of action.
- The Node has to setup the MCTS planner to run it. It creates an instance of MCTSGameState to represent the current world state. MCTSGameState implements the GameStateProtocol required by the MCTS implementation. GameStateProtocol contains world state, logic for sampling and applying actions, accessing probabilities of actions, and calculating the value of nodes.
- The MCTS planner returns a trajectory of actions and states for the robot to execute.
- A Pure Pursuit planner implemented inside the navigator node is used to track the trajectory, combining the MCTS actions with ones calculated in real time to account for error.

## Run the Social Simulator

Start by following the standard ROS2 Workspace instructions at the bottom of this README

Then build:
```
colcon build --symlink-install --packages-skip zed_isaac_ros_nitros_sub zed_isaac_ros_april_tag
colcon build --symlink-install --packages-select social_simulator
```

To run the simulator (I suggest to use tmux):
```
ros2 launch social_simulator social_simulator.launch.py scenario:=agents_wide_hallway.yaml
```

## To Run Evaluations

To accurately evaluate the MCTS algorithm, Gazebo simulation must be slowed down. Change the following settings in hunav_gazebo_wrapper/src/WorldGenerator.cpp
```
time_factor->SetText(0.1);
time_rate->SetText(100);  // 100

rtf->SetText(0.1);
rtur->SetText(100);  // 100
```

Start the simulator with the correct map and scenario
```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_1_ahead_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

Start the evaluation with the right scenario. See below for possible scenarios.
```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_1_ahead_robot.yaml experiment_tag:=test_run run_id:=0
```

## Narrower Hallway

### JointMCTS

```
ros2 launch social_simulator social_simulator.launch.py world:=narrower_hallway.world scenario:=narrower_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrower_hallway/agents_2_towards_robot.yaml experiment_tag:=2_towards_robot run_id:=0 evaluation_goal_topic:='/evaluation_goal_set'
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=narrower_hallway.world scenario:=narrower_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrower_hallway/agents_2_towards_robot.yaml experiment_tag:=2_towards_robot run_id:=0 evaluation_goal_topic:='/goal_pose'
```

## Narrow Hallway

### JointMCTS

#### 2 towards robot
```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_2_towards_robot.yaml experiment_tag:=mcts_2_towards_robot run_id:=0 
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' headless:=True
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_2_towards_robot.yaml experiment_tag:=nav2_2_towards_robot run_id:=0 evaluation_goal_topic:='/goal_pose'
```

### JointMCTS

#### 1 ahead robot
```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_1_ahead_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_1_ahead_robot.yaml experiment_tag:=mcts_1_ahead_robot run_id:=0
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_1_ahead_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' headless:=True
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_1_ahead_robot.yaml experiment_tag:=nav2_1_ahead_robot run_id:=0 evaluation_goal_topic:='/goal_pose'
```

### JointMCTS

#### 1 behind robot
```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_1_behind_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_1_behind_robot.yaml experiment_tag:=1_behind_robot run_id:=0
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=narrow_hallway.world scenario:=narrow_hallway/agents_1_behind_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' headless:=True
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=narrow_hallway/agents_1_behind_robot.yaml experiment_tag:=nav2_1_behind_robot run_id:=0 evaluation_goal_topic:='/goal_pose'
```

## Intersection Hallway

### JointMCTS

#### 3 crossing
```
ros2 launch social_simulator social_simulator.launch.py world:=intersection_hallway.world scenario:=intersection_hallway/agents_3_crossing.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=intersection_hallway/agents_3_crossing.yaml experiment_tag:=3_crossing run_id:=0
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=intersection_hallway.world scenario:=intersection_hallway/agents_3_crossing.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' headless:=True
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=intersection_hallway/agents_3_crossing.yaml experiment_tag:=nav2_3_crossing run_id:=0 evaluation_goal_topic:='/goal_pose'
```

## Wide Hallway

### JointMCTS

#### 3 agents group
```
ros2 launch social_simulator social_simulator.launch.py world:=wide_hallway.world scenario:=wide_hallway/agents_3_agent_group.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=wide_hallway/agents_3_agent_group.yaml experiment_tag:=3_agents_group run_id:=0
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=wide_hallway.world scenario:=wide_hallway/agents_3_agent_group.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' headless:=True
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=wide_hallway/agents_3_agent_group.yaml experiment_tag:=nav2_3_agent_group run_id:=0 evaluation_goal_topic:='/goal_pose'
```

## T-shape Hallway

### JointMCTS

#### 3 agents group
```
ros2 launch social_simulator social_simulator.launch.py world:=t_shape_hallway.world scenario:=t_shape_hallway/agents_t_shape_hallway.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=t_shape_hallway/agents_t_shape_hallway.yaml experiment_tag:=t_shape_hallway run_id:=0
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=t_shape_hallway.world scenario:=t_shape_hallway/agents_t_shape_hallway.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' headless:=True
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=t_shape_hallway/agents_t_shape_hallway.yaml experiment_tag:=nav2_t_shape_hallway run_id:=0 evaluation_goal_topic:='/goal_pose'
```

## Doors Hallway

### JointMCTS

#### 3 agents group
```
ros2 launch social_simulator social_simulator.launch.py world:=doors_hallway.world scenario:=doors_hallway/agents_doors_hallway.yaml use_navgoal_to_start:=true navgoal_topic:='/evaluation_goal_set'
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=doors_hallway/agents_doors_hallway.yaml experiment_tag:=doors_hallway run_id:=0
```

### Nav2

```
ros2 launch social_simulator social_simulator.launch.py world:=doors_hallway.world scenario:=doors_hallway/agents_doors_hallway.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' headless:=True
```

```
ros2 launch social_simulator evaluation.launch.py scenario:=doors_hallway/agents_doors_hallway.yaml experiment_tag:=nav2_agents_doors_hallway run_id:=0 evaluation_goal_topic:='/goal_pose'
```

### Debugging

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

### Nav2 without people
```
ros2 launch social_simulator social_simulator.launch.py world:=narrower_hallway.world scenario:=narrower_hallway/agents_2_towards_robot.yaml use_navgoal_to_start:=true navgoal_topic:='/goal_pose' use_humans:=False
```

### Teleop command

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

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
