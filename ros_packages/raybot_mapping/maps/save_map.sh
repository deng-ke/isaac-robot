#! /bin/bash
rosservice call /write_state /home/rayrobot/raybot_ws/src/raybot_mapping/maps/map.pbstream

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/rayrobot/raybot_ws/src/raybot_mapping/maps/map -pbstream_filename=/home/rayrobot/raybot_ws/src/raybot_mapping/maps/map.pbstream -resolution=0.05
