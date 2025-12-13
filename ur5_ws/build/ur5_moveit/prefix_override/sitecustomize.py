import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abdulrahman/me5250/src/ur5_ws/install/ur5_moveit'
