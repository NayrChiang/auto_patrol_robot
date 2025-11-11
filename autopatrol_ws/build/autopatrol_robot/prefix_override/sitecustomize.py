import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nayr/auto_patrol_robot/autopatrol_ws/install/autopatrol_robot'
