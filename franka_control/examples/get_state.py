from franky import *
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *

robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)

# Get the current state as `franky.RobotState`. See the documentation for a list of fields.
state = robot.state

# Get the robot's cartesian state
cartesian_state = robot.current_cartesian_state
robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
ee_pose = robot_pose.end_effector_pose
elbow_pos = robot_pose.elbow_state
robot_velocity = cartesian_state.velocity  # Contains end-effector twist and elbow velocity
ee_twist = robot_velocity.end_effector_twist
elbow_vel = robot_velocity.elbow_velocity

print(f"ee_pose: {ee_pose}")
print(f"elbow_pos: {elbow_pos}")
print(f"ee_twist: {ee_twist}")
print(f"elbow_vel: {elbow_vel}")

# Get the robot's joint state
joint_state = robot.current_joint_state
joint_pos = joint_state.position
joint_vel = joint_state.velocity

# Use the robot model to compute kinematics
q = [-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7]
f_t_ee = Affine()
ee_t_k = Affine()
ee_pose_kin = robot.model.pose(Frame.EndEffector, q, f_t_ee, ee_t_k)

# Get the Jacobian of the current robot state
jacobian = robot.model.body_jacobian(Frame.EndEffector, state)

# Alternatively, just get the URDF as a string and do the kinematics computation yourself (only
# for libfranka >= 0.15.0)
urdf_model = robot.model_urdf
