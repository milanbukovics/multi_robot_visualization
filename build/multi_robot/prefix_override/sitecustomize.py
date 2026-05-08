import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/drl/multi_robot_RViz/multi_robot_visualization/install/multi_robot'
