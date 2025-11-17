import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rocotics/ros2_ws/src/custom_robot_sim/install/custom_robot_sim'
