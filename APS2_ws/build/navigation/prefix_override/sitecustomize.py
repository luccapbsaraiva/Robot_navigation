import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/luccapbs/Desktop/Robotica/APS2_ws/install/navigation'
