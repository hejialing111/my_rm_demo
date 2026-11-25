import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hejialing/demo_01/RM_Energy_Hit_Simulator_Linux_20261009_RedR/ros2_ws/install/rm_energy_bridge'
