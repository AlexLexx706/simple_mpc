import casadi as ca
from typing import Tuple
import math


class Mpc:
    def __init__(self, dt=0.1, steps=10, xtrack_weight=1., heading_weight=150., max_iter=50):
        opti = ca.Opti()
        self.dt = dt
        # state: x,y,heading:
        states = opti.variable(steps + 1, 3)

        # N + 1 control angle:
        angle = opti.variable(steps + 1)

        # Parameters:
        # line description
        a = opti.parameter(2)
        b = opti.parameter(2)

        # initial values
        state_0 = opti.parameter(3)
        angle_0 = opti.parameter()
        speed = opti.parameter()
        wheel_base = opti.parameter()
        max_rate = opti.parameter()
        max_angle = opti.parameter()

        # calculation of line heading
        ba = b - a
        line_heading = ca.atan2(ba[1], ba[0])

        common_cost = 0
        xtrack = self.create_xtrack_fun()

        # common cost:
        for i in range(steps):
            pos = states[i, :2].T
            common_cost += xtrack_weight * xtrack(a, b, pos) ** 2
            common_cost += heading_weight * (states[i, 2] - line_heading) ** 2
        opti.minimize(common_cost)

        # constrains:
        for i in range(steps):
            ds = ca.horzcat(
                speed * ca.cos(states[i, 2]),
                speed * ca.sin(states[i, 2]),
                speed * ca.tan(angle[i]) / wheel_base)

            opti.subject_to(states[i + 1, :] == states[i, :] + ds * dt)

            # limits:
            opti.subject_to(
                opti.bounded(-max_rate, (angle[i + 1] - angle[i]) / dt, max_rate))
            opti.subject_to(opti.bounded(-max_angle, angle[i], max_angle))

        # initial constrain:
        opti.subject_to(states[0, :] == state_0.T)
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
            [a, b, state_0, angle_0, speed, wheel_base, max_rate, max_angle],
            [states, angle])

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
            a,
            b,
            state_0,
            angle_0,
            speed,
            wheel_base,
            max_rate,
            max_angle)
        return float(solution[1][1])

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
