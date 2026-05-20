import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/husarion/Desktop/tourguide/src/tourguide_navigation/install/tourguide_navigation'
