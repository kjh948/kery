Kery
Alternative variations inspired by kuri robot.
Special thanks to linorobot project members.

Here are short tips of kery operation.

1. audio
sudo vi /etc/asound.conf 
cat /proc/asound/cards

2. linorobot related script

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
For mapping.
roslaunch linorobot bringup.launch
roslaunch linorobot slam.launch

roscd lino_visualize/rviz
rviz -d slam.rviz

rosrun map_server map_saver -f ~/linorobot_ws/src/linorobot/maps/map
roscd linorobot/maps
ls -a map.pgm map.yaml

For navi.
roslaunch linorobot bringup.launch
roslaunch linorobot navigate.launch
roscd lino_visualize/rviz
rviz -d navigate.rviz

For lidar checking,
  roscd lino_visualize/rviz
  rviz -d laser.rviz
  
3. audio device id keeps chaging
https://askubuntu.com/questions/991484/how-to-prevent-automatic-change-of-the-audio-device


4. ros python pkg quickguide
https://htsstory.tistory.com/entry/ROS-python%ED%8C%8C%EC%9D%B4%EC%8D%AC-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D-1-%ED%86%A0%ED%94%BD-%EB%A9%94%EC%8B%9C%EC%A7%80-%ED%86%B5%EC%8B%A0

catkin_create_pkg NAME std_msgs rospy
