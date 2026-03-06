import time
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *
from franky import *

robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)
robot.relative_dynamics_factor = 0.02

motion1 = CartesianMotion(Affine([0.04, 0.0, 0.0]), ReferenceType.Relative)
robot.move(motion1, asynchronous=True)

time.sleep(0.5)
motion2 = CartesianMotion(Affine([0.0, 0.01, 0.0]), ReferenceType.Relative)
robot.move(motion2, asynchronous=True)
