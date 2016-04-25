# couch_simulation
Couch v-rep files and ros nodes

# Getting started
- Assumes you've got V-REP runnning (Downloaded/installed V-REP from http://www.coppeliarobotics.com/downloads.html) and ROS indigo installed.
- In your catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace if you don't have one) git clone this repo and vrep_ros (https://github.com/StephMc/vrep_ros) into the src directory.
- Build from the workspace root directory ($catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo)
- Copy libv_repExtRos.so and libv_repExtRosSkeleton.so from <catkin_workspace>/devel/lib to the root of your V-REP install directory
- Soft link the couch plugin to the root of your V-REP install (ln -s <catkin_workspace>/build/couch_simulation/vrep_couch_sim/vrep_couch_sim <V_REP_install_root>/vrep_couch_sim)
- Start roscore
- Start vrep (in the start up text it should say "Plugin 'Ros': load succeeded.", if not then something is broken)
- Open the sim_scenes/simple_outdoors.ttt scene in V-REP
- Start the simulation, rostopic list should show the front and back lidar scans
- The couch moves with Twist commands (a rqt virtual joystick: rosrun rqt_robot_steering rqt_robot_steering)
