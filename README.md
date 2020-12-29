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
