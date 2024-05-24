# Para correr el XARM IRL

# Correr el driver del xarm (planner)
ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.203 [add_gripper:=true] 

# Correr el planner otra vez? xd
ros2 launch xarm_planner follow.launch.py

# Correr la trayectoria
ros2 run xarm trajectory_planner.py

# Running Xarm with object Tracking 

# xarm_planner[simulated] 
ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]
# xarm planner [real]
ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.203 [add_gripper:=true] 

# Run object tracking launch file
ros2 launch xarm xarm_launch.py 
# Topic /xarm_obj_marker to visualize object in rviz

# Run Object Tracking (in TurboPlusPlus_ImplementacionRobotica/reto3_ws)
ros2 run reto3 ObjPos_node.py



