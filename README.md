# RoverBehaviorTree
--

## Installation
Follow these instructions to clone and setup this repository.  

### Setup Catkin Workspace
`mkdir -p catkin_ws/src`
`cd catkin_ws`
`catkin build`
`cd catkin_ws/src`

### Clone Repo
`git clone https://github.com/CJcool06/RoverBehaviorTree.git`

### Source Workspace
`cd ..`
`catkin build`
`source devel/setup.bash`

## Running
Each of these commands should be run in individual bash instances.  

1. `roscore`
2. `rosrun behavior_tree Comms.py`
3. `rosrun behavior_tree Runner.py`

At this stage you can use the `behavior_tree_comms` node to input coordinates while the `behavior_tree_runner` node simulates the behavior tree states.
