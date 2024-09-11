import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/deep13/ros2_ws/src/comprobo24_warmup_project/install/warmup_project'
