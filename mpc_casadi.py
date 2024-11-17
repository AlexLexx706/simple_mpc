import casadi as ca
from typing import Tuple
import math


class Mpc:
    def __init__(self, dt=0.1, steps=10, xtrack_weight=1., heading_weight=150., max_iter=50):
        opti = ca.Opti()
        self.dt = dt
        # Variables
        x = opti.variable(steps + 1)
        y = opti.variable(steps + 1)
        heading = opti.variable(steps + 1)

        # N + 1 control angle:
        angle = opti.variable(steps + 1)

        # Parameters:
        # line description
        a_x = opti.parameter()
        a_y = opti.parameter()

        b_x = opti.parameter()
        b_y = opti.parameter()

        # initial values
        x_0 = opti.parameter()
        y_0 = opti.parameter()

        heading_0 = opti.parameter()

        angle_0 = opti.parameter()
        speed = opti.parameter()
        wheel_base = opti.parameter()
        max_rate = opti.parameter()
        max_angle = opti.parameter()

        # calculation of line heading
        line_heading = ca.atan2(b_y - a_y, b_x - a_x)

        common_cost = 0
        xtrack = self.create_xtrack_fun()

        # common cost:
        for i in range(steps):
            common_cost += xtrack_weight * xtrack(
                ca.vertcat(a_x, a_y),
                ca.vertcat(b_x, b_y),
                ca.vertcat(x[i], y[i]),) ** 2

            common_cost += heading_weight * (heading[i] - line_heading) ** 2
        opti.minimize(common_cost)

        # constrains:
        for i in range(steps):
            opti.subject_to(x[i + 1] == x[i] + dt * speed * ca.cos(heading[i]))
            opti.subject_to(y[i + 1] == y[i] + dt * speed * ca.sin(heading[i]))
            opti.subject_to(heading[i + 1] == heading[i] +
                            dt * speed * ca.tan(angle[i]) / wheel_base)

            # limits:
            opti.subject_to(
                opti.bounded(-max_rate, (angle[i + 1] - angle[i]) / dt, max_rate))
            opti.subject_to(opti.bounded(-max_angle, angle[i], max_angle))

        # initial constrain:
        opti.subject_to(x[0] == x_0)
        opti.subject_to(y[0] == y_0)
        opti.subject_to(heading[0] == heading_0)
        opti.subject_to(angle[0] == angle_0)

        opti.solver(
            'ipopt',
            {
                'ipopt.print_level': 0,
                'print_time': 0,
                'ipopt.sb': 'yes'},
            {"max_iter": max_iter}
        )

        self.mpc_fun = opti.to_function(
            'MPC',
            [a_x, a_y, b_x, b_y, x_0, y_0, heading_0, angle_0,
                speed, wheel_base, max_rate, max_angle],
            [x, y, heading, angle])

    def optimize_controls(
            self,
            a: Tuple[float, float],
            b: Tuple[float, float],
            state_0: Tuple[float, float, float],
            angle_0: float,
            speed: float,
            wheel_base: float,
            max_rate: float,
            max_angle: float):
        """_summary_

        Args:
            a (Tuple[float, float]): A point of line
            b (Tuple[float, float]): B point of line
            state_0 (Tuple[float, float, float]): initial state: [x, y, heading]
            angle_0 (float): initial steering angle
            speed (float): speedwheel_base
            wheel_base (float): wheel_base len of vehicle
            max_rate (float): max rate of steering wheelwheel_base
            max_angle (float): max angle of steering wheel

        Returns:
            _type_: _description_
        """
        solution = self.mpc_fun(
            a[0], a[1],
            b[0], b[1],
            state_0[0], state_0[1], state_0[2],
            angle_0,
            speed,
            wheel_base,
            max_rate,
            max_angle)
        return float(solution[3][1])

    @staticmethod
    def create_xtrack_fun() -> ca.Function:
        """create casadi function for calculating Xtrack to line.
            Parameters of the function:
                A[x,y] - A point of the line
                B[x,y] - B point of the line
                Point[x,y] - position
            result of the function: scalar - distance to linewheel_base
        Returns:
            ca.Function: casadi function
        """
        a = ca.SX.sym('a', 2)
        b = ca.SX.sym('b', 2)
        pos = ca.SX.sym('pos', 2)
        ba = (b - a)
        ba_n = ba / ca.norm_2(ba)
        distance = pos - (a + (ba_n.T @ (pos - a)) * ba_n)
        return ca.Function('xtrack', [a, b, pos], [ca.norm_2(distance)])


if __name__ == "__main__":
    import math
    mpc = Mpc()

    A = [0., 0.]
    B = [100., 0.]
    STATE_0 = [0., 10, 0.]
    ANGLE_0 = 0.
    SPEED = 1.
    WHEEL_BASE = 3
    MAX_RATE = math.radians(60)
    MAX_ANGLE = math.radians(25)

    res = mpc.optimize_controls(
        A, B, STATE_0, ANGLE_0, SPEED, WHEEL_BASE, MAX_RATE, MAX_ANGLE)
    print(res)
