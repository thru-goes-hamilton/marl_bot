# MARL Bot Workspace Setup

This guide explains how to set up the `marl_bot` repository in a new ROS 2 Humble workspace.


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

## Launch files

...
