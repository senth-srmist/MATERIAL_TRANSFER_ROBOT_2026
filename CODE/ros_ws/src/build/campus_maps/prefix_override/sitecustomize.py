import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lalithesh/major_project/MATERIAL_TRANSFER_ROBOT_2026/CODE/ros_ws/src/install/campus_maps'
