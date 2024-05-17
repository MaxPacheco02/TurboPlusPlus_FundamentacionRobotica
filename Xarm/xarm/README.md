# Para correr el XARM IRL

# Correr el driver del xarm (planner)
ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.203 [add_gripper:=true] 

# Correr el planner otra vez? xd
ros2 launch xarm_planner follow.launch.py

# Correr la trayectoria
ros2 run xarm trajectory_planner.py