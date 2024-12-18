import casadi as ca
from mpc import mpc_casadi

path = ca.DM([[1, 1], [1, 5], [4,1]])
circle_pos = ca.DM([[-1, 3]])
circle_radius = 1.
print(mpc_casadi.MPCCasadi.get_distance(path, circle_pos, circle_radius))