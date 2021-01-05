Simple action server for generating trajectories and sending them to [panda_simulator](https://github.com/justagist/panda_simulator).

Availiable actions:
- [robot_module_msgs/JointMinJerk.action](https://github.com/ReconCycle/robot_module_msgs/blob/ijs_controllers_update/action/JointMinJerk.action)

# Installation

```
cd catkin_ws/src
git clone https://github.com/ReconCycle/sim_controllers_interface
git clone -b ijs_controllers_update https://github.com/ReconCycle/robot_module_msgs
catkin build
source devel/setup.bash
```

# Example usage  

- Start simulation 

      $ roslaunch panda_gazebo panda_world.launch 

- Start action server

      $ rosrun sim_controllers_interface joint_min_jerk_action_server.py

- Test action server

      $ rosrun sim_controllers_interface joint_min_jerk_test_client.py

# Docker example

https://github.com/abr-ijs/panda_dockers

