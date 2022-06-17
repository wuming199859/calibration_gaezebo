# calibration_gaezebo
calibration_gaezebo

1、标定
 roslaunch husky_gazebo husky_user_world.launch
roslaunch velo2cam_calibration mono_pattern.launch camera_name:=/realsense/color image_topic:=image_raw frame_name:=camera_realsense_gazebo
roslaunch velo2cam_calibration lidar_pattern.launch cloud_topic:=/velodyne_points（滤波） x 0~6  y -1.7~1.7  z -0.5~6
roslaunch velo2cam_calibration registration.launch sensor1_type:=mono sensor2_type:=lidar

2、点云融合
 roslaunch husky_gazebo husky_playpen.launch
 fusion/buid   ./pcl_fuison
