# MARL Bot

The goal of this project is to build multiple robots that can search for an object/person fully autonomously using Multi Agent Reinforcement Learning to navigate. It is built using the ROS2 Humble middleware to facilitate low latency and scalable communication between robots. The mechanical design, electrical circuit and methodology for action has been implemented from scratch. The project has progressed from ideation to implementation of teleoperation and sensor fusion for two robots simulatneously in the real world over the span of 6 months. For more details on the methodology used to achieve this, a patent under the title "A scalable approach to autonomous multi robot collaboration searching targets in unchartered environment" is underway.


# Media

## Robots and team

![Single_robot_real_world](https://github.com/user-attachments/assets/46733821-410f-4603-8862-fc761324afa9)

![robots_in_real_world](https://github.com/user-attachments/assets/61815460-300b-42de-bd77-b87d82438d71)

![Multi_robot_outdoor_with_team](https://github.com/user-attachments/assets/976dc8c3-9185-464f-9cd6-f72060798962)

![Project_team_and_robots](https://github.com/user-attachments/assets/b40c1361-c500-454a-9d10-c88812a94834)



## Planning and Designing

### Component Placement

<img width="385" alt="Component_placement_diagram" src="https://github.com/user-attachments/assets/078b995f-6b44-430e-b703-25dc28a7a2b1" />


### Circuit diagram

<img width="506" alt="Circuit_diagram" src="https://github.com/user-attachments/assets/1f853cda-bb13-4922-b6f4-ea45e6d2806e" />


### Chassis design on CAD

#### top base

<img width="509" alt="top_base_CAD_design" src="https://github.com/user-attachments/assets/48d9203c-db64-4dd1-8f75-3d6231ec21ad" />

#### bottom base

<img width="512" alt="bottom_base_CAD_design" src="https://github.com/user-attachments/assets/1e5de679-e234-4307-9c70-a388aba2eb89" />

### ROS nodes and topics

![Ros_nodes_and_topics](https://github.com/user-attachments/assets/99e498d4-a04f-4172-b4dc-06264a4d5ea6)

### Circuit bench testing

https://github.com/user-attachments/assets/a2cea076-f7d1-41e1-a3c3-59174be2199e


---

## Simulation

### Modeling robots on Isaac Sim

![Isaac_Sim_attempt](https://github.com/user-attachments/assets/82596e49-f2f1-40da-a0b1-f69442ea4270)

### Single robot SLAM on Gazebo (teleoperated via game controller)

https://github.com/user-attachments/assets/8a5e5b89-11f0-42ea-8848-9872b5d1b518

### Multi Robot SLAM (teleoperated simultaneously via game controller)

https://github.com/user-attachments/assets/10db9335-9ac9-4b61-898a-47960818a7cc


## Demo

https://github.com/user-attachments/assets/f83c2946-bb07-490c-a98c-d7762431d8e6


# MARL Bot Workspace Setup

This following guide explains how to set up the `marl_bot` package in a new ROS 2 Humble workspace.


## Installation Steps

Run the following commands to create the necessary directories, clone the repository, move the `debug.sh` script, and execute it.

### 1. Create the workspace and clone the repository
```bash
mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/your-username/marl_bot.git
```
### 2. Move `debug.sh` to the workspace root and make it executable
```bash
mv marl_bot/debug.sh ../
chmod +x ../debug.sh
```
### 3. Source ROS 2 and run the debug script
```bash
source /opt/ros/humble/setup.bash
cd ..
./debug.sh
```
### 4. Source marl_bot package after buiding
```bash
source install/setup.bash
```

After running these steps the installation should be complete.

## Launch file
```bash
ros2 launch marl_bot iteration.launch.py
```



