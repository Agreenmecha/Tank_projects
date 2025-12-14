# Tankbot Scripts

Helper scripts for managing the tankbot visualization and debugging.

## clean_start_viz.sh

**Clean restart script for remote visualization.**

Kills all existing processes (robot_state_publisher, joint_state_publisher, rviz2) and starts them fresh. Use this when you're experiencing TF transform issues or want to ensure a clean state.

**Usage:**
```bash
cd ~/Tank_projects/tank_ws
bash src/tankbot/scripts/clean_start_viz.sh
```

**What it does:**
1. Kills existing processes
2. Sources ROS and workspace environment
3. Validates the URDF
4. Launches `remote_viz.launch.py`

**When to use:**
- After making URDF changes
- When experiencing "no transform" errors
- When multiple robot_state_publisher instances are running
- To ensure a clean start state

## start_viz.sh

**Quick start script - just launches the visualization.**

Does NOT kill existing processes. Use this for a quick start if you're sure everything is clean.

**Usage:**
```bash
cd ~/Tank_projects/tank_ws
bash src/tankbot/scripts/start_viz.sh
```

**When to use:**
- When you know no conflicting processes are running
- For quick restarts during normal use
- If you just want to launch RViz without cleanup

