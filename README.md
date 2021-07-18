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

4. pip tips
TMPDIR=~/workspace/tmp/ pip install --cache-dir=~/workspace/tmp/ opencv-contrib-python==4.2.0.32

5. ros python pkg quickguide
https://htsstory.tistory.com/entry/ROS-python%ED%8C%8C%EC%9D%B4%EC%8D%AC-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D-1-%ED%86%A0%ED%94%BD-%EB%A9%94%EC%8B%9C%EC%A7%80-%ED%86%B5%EC%8B%A0

catkin_create_pkg NAME std_msgs rospy

6. pygame downgrade
https://stackoverflow.com/questions/62543965/pygame-audio-error-unrecognized-audio-format

7. service registration on boot
https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/
rosrun robot_upstart install kery_face/launch/kery_face.launch --job kery_face --symlink

8. Move cursor
xdotool mousemove 10 10

9. astra camera
rosrun usb_cam usb_cam_node _video_device:=/dev/video1 _framerate:=10  _pixel_format:=yuyv _image_width:=640 _image_height:=480
roslaunch astra_launch astrapro.launch

10. smach install
sudo apt-get install ros-kinetic-smach ros-kinetic-smach-ros ros-kinetic-executive-smach ros-kinetic-smach-viewer

11. ros repo key update
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

12. 