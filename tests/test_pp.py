from mpc.mpc_casadi import MPCCasadi
import math
import casadi as ca
import numpy as np

path = ca.SX.sym('path', 3, 2)
pos = ca.SX.sym('pos', 2).T
heading = ca.SX.sym('heading')

# xtrack, heading_error = MPCCasadi.get_track_cost(path, pos, heading)

# f_xt = ca.Function('f_xt', [path, pos, heading], [xtrack])
# f_he = ca.Function('f_he', [path, pos, heading], [heading_error])

path = ca.DM([[0.,0.], [0., 10.], [10., 10.]])
pos = ca.DM([1., 2.]).T
heading = ca.DM(math.radians(0))

print(f"f_xt:{MPCCasadi.get_track_cost(path, pos, heading)}")
# print(f"f_he:{math.degrees(f_he(path, pos, heading))}")

pos = ca.DM([1., 9.5]).T
print(f"f_xt:{MPCCasadi.get_track_cost(path, pos, heading)}")

# print(f"f_xt:{f_xt(path, pos, heading)}")
# print(f"f_he:{math.degrees(f_he(path, pos, heading))}")
