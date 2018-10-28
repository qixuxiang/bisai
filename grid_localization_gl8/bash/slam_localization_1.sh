gnome-terminal \
--tab -t "pose_fusion" -e 'bash -c "roslaunch pose_fusion filter_strong.launch; "' \
--tab -t "gps_to_slam" -e 'bash -c "sleep 2; roslaunch fix2pose gps_to_slam.launch;"' \
--tab -t "grid_localization" -e "sh -c 'sleep 1;roslaunch grid_localization_gl8 gl8_localizaiton_offline.launch;'" \
--tab -t "slam_" -e 'bash -c "sleep 2; roslaunch fix2pose slam2.launch;"' \
--tab -t "rviz" -e 'bash -c "sleep 1; rviz;"' \
