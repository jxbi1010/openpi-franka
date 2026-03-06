import time
from franky import *
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *

robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)
robot.relative_dynamics_factor = 0.05


motion = CartesianMotion(Affine([-0.05, -0.05, -0.05]), ReferenceType.Relative)
robot.move(motion)
time.sleep(1)

motion = CartesianMotion(Affine([0.05, 0.05, 0.05]), ReferenceType.Relative)
robot.move(motion)
