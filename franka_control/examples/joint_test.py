from franky import *
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *

robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)

# A point-to-point motion in the joint space
m_jp1 = JointMotion([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7])

# A motion in joint space with multiple waypoints. The robot will stop at each of these
# waypoints. If you want the robot to move continuously, you have to specify a target velocity
# at every waypoint as shown in the example following this one.
m_jp2 = JointWaypointMotion(
    [
        JointWaypoint([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7]),
        JointWaypoint([0.0, 0.3, 0.3, -1.5, -0.2, 1.5, 0.8]),
        JointWaypoint([0.1, 0.4, 0.3, -1.4, -0.3, 1.7, 0.9]),
        JointWaypoint([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7]),
    ]
)

# Intermediate waypoints also permit specifying target velocities. The default target velocity
# is 0, meaning that the robot will stop at every waypoint.
m_jp3 = JointWaypointMotion(
    [
        JointWaypoint([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7]),
        JointWaypoint(
            JointState(
                position=[0.0, 0.3, 0.3, -1.5, -0.2, 1.5, 0.8],
                velocity=[0.1, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0],
            )
        ),
        JointWaypoint([0.1, 0.4, 0.3, -1.4, -0.3, 1.7, 0.9]),
        JointWaypoint([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7]),
    ]
)

# Stop the robot in joint position control mode. The difference between JointStopMotion to other
# stop-motions, such as CartesianStopMotion, is that JointStopMotion stops the robot in joint
# position control mode while CartesianStopMotion stops it in cartesian pose control mode. The
# difference becomes relevant when asynchronous move commands are being sent or reactions are
# being used(see below).
m_jp4 = JointStopMotion()



# Before moving the robot, set an appropriate dynamics factor. We start small:
robot.relative_dynamics_factor = 0.05
# or alternatively, to control the scaling of velocity, acceleration, and jerk limits
# separately:
# robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.1, 0.15)
# If these values are set too high, you will see discontinuity errors
robot.move(m_jp3)

# # We can also set a relative dynamics factor in the move command. It will be multiplied by
# # the other relative dynamics factors (robot and motion if present).
# robot.move(m_jp2, relative_dynamics_factor=0.8)
