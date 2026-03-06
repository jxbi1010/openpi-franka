import numpy as np
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *
from franky import *

robot = Robot("172.16.0.2", realtime_config=RealtimeConfig.Ignore)


# Accelerate to the given joint velocity and hold it. After 1000ms, stop the robot again.
m_jv1 = JointVelocityMotion(
    [0.1, 0.3, -0.1, 0.0, 0.1, -0.2, 0.4], duration=Duration(1000)
)

m_jv2 = JointVelocityWaypointMotion(
    [
        JointVelocityWaypoint(
            [0.1, 0.3, -0.1, 0.0, 0.1, -0.2, 0.4], hold_target_duration=Duration(1000)
        ),
        JointVelocityWaypoint(
            [-0.1, -0.3, 0.1, -0.0, -0.1, 0.2, -0.4],
            hold_target_duration=Duration(2000),
        ),
        JointVelocityWaypoint(
            [0.1, 0.3, -0.1, 0.0, 0.1, -0.2, 0.4], hold_target_duration=Duration(1000)
        ),
        JointVelocityWaypoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]
)

# Stop the robot in joint Motion()
m_jv3 = JointVelocityStopMotion()

# robot.move(m_jv1)
robot.move(m_jv2)
# robot.move(m_jv3)
