# CMake generated Testfile for 
# Source directory: /home/fan/ros_slam/cartographer/cartographer/sensor
# Build directory: /home/fan/ros_slam/cartographer/build/cartographer/sensor
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(sensor_compressed_point_cloud_test "sensor_compressed_point_cloud_test")
add_test(sensor_laser_test "sensor_laser_test")
add_test(sensor_point_cloud_test "sensor_point_cloud_test")
add_test(sensor_voxel_filter_test "sensor_voxel_filter_test")
subdirs(proto)
