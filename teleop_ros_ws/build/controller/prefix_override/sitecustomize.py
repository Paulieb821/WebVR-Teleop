import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/paul/Documents/GitHub/WebVR-Teleop/teleop_ros_ws/install/controller'
