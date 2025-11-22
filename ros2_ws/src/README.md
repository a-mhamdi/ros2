# Suggested order to examine the folders

To understand how everything fits together, it is best to look at the packages in this order:

1. `listen_talk_pkg`              -> Simple talker/listener using standard String messages
1. `demo_ros_py`                  -> Basic **ROS 2** nodes (publisher/subscriber) 
1. `demo_ros_cpp`                 -> Same as above but in `C++`  
1. `my_interfaces`                -> Customr messages/services/actions
1. `demo_interfaces_py`           -> How to use custom interfaces in `Python` *(publisher)*
1. `demo_interfaces_cpp`          -> How to use custom interfaces in `C++` *(subscriber)*
1. `turtle_controller`            -> Turtlesim node controller
1. `basic_gazebo_rviz_demo`       -> Simulation (Gazebo) and visualization (RViz)
1. `my_robot`                     -> Full custom robot (URDF, controller, Gazebo, RViz, navigation)
