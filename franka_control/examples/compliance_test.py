from franky import *
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *

robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)
robot.relative_dynamics_factor = 0.05

motion = CartesianMotion(Affine([0.0, 0.0, -0.1]), ReferenceType.Relative)  # Move down 10cm

reaction_motion = CartesianMotion(Affine([0.0, 0.0, -0.01]), ReferenceType.Relative)

# Trigger reaction if the Z force is greater than 30N
reaction = Reaction(Measure.FORCE_Z > 1.0, reaction_motion)
motion.add_reaction(reaction)

robot.move(motion)
