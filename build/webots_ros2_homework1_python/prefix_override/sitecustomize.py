import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/colep/f24_robotics/webots_ros2_homework1_python/install/webots_ros2_homework1_python'
