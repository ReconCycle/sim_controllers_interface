# Example usage  

* Starting simulation 

```
docker run -p 9090:9090 -it reconcycle/docker_examples:ros1-simulation --name panda_sim roslaunch panda_gazebo panda_world.launch 
```

* Webinterface is at http://localhost:9090/vnc.html

* Install into docker
```
docker exec -it panda_sim bash
cd /ros_ws/src
git clone https://github.com/ReconCycle/sim_controllers_interface
cd robot_module_msgs
git pull && git checkout ijs_controllers_update
catkin build
```
TODO: integrate into docker image


* Run action server 
```
docker exec -it panda_sim bash
source devel/setup.bash
rosrun sim_controllers_interface joint_min_jerk_action_server.py
```

* Action server specification

[robot_module_msgs/JointMinJerk.action](https://github.com/ReconCycle/robot_module_msgs/blob/ijs_controllers_update/action/JointMinJerk.action)

* Test action server
```
docker exec -it panda_sim bash
source devel/setup.bash
rosrun sim_controllers_interface joint_min_jerk_test_client.py
```

