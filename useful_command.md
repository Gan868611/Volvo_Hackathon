##mavigation
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/volvo2/catkin_ws/src/turtlebot3/turtlebot3_navigation/maps/map.yaml


##intrinsic
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/image camera:=/camera




roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration
    rosrun rqt_reconfigure rqt_reconfigure


    roslaunch turtlebot3_slam turtlebot3_slam.launch

roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action


#detect lane
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration


roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action 
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch

roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_moving.launch



roslaunch teleop_twist_joy teleop.launch config_filepath:=/home/volvo2/catkin_ws/ps4.config.yaml

roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch mission:=intersection

source devel/setup.bash

rosrun image_view image_saver image:=/camera/image _fps:=10.0  _filename_format:="/home/volvo2/catkin_ws/collected_images/10_jan/myframe_%04d.jpg"
2000 images => 73MB


blue 
72,108,19,255

yellow
0,96,70,255

