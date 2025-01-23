import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/ssd/home/edt/Documents/lunabotics_ros_ws/install/brain_pkg'
