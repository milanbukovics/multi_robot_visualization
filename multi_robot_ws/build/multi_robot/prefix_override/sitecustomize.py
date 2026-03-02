import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/drl/multi_robot/multi_robot_ws/install/multi_robot'
