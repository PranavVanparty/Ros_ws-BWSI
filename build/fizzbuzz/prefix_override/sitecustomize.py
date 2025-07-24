import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/firefox/Ros_ws-BWSI/install/fizzbuzz'
