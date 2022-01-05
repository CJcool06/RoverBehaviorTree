# RoverBehaviorTree
--

## Installation
Follow these instructions to clone and setup this repository.  

### Clone Repo
`git clone https://github.com/CJcool06/RoverBehaviorTree.git`

### Build Catkin Workspace
`cd path/to/repo/RoverBehaviorTree/catkin_ws`
`catkin_build`

### Source Workspace
`source devel/setup.bash`

## Running
Each of these commands should be run in individual bash instances.  

1. `roscore`
2. `rosrun behavior_tree Comms.py`
3. `rosrun behavior_tree Runner.py`

At this stage you can use the `behavior_tree_comms` node terminal to input coordinates while the `behavior_tree_runner` node simulates the behavior tree states.
