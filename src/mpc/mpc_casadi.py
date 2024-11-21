from typing import Tuple, List
import math
import casadi as ca


class MPCCasadi:
    SOFT_CONSTRAIN=True

    def __init__(
            self,
            dt: float = 0.3,
            steps: int = 30,
            max_iter: int = 50):
        """MPC for controlling trailer points

        Args:
            dt (float, optional): integration step, s. Defaults to 0.5.
            steps (int, optional): number of steps - horizon prediction. Defaults to 10.
            max_iter (int, optional): limit of max iterations for solver. Defaults to 50.
        """
        opti = ca.Opti()

        # state: x, y, heading, trailer_heading:
        states = opti.variable(steps + 1, 4)
        self.last_states = ca.DM.zeros(states.shape)

        # N + 1 control angle:
        angle = opti.variable(steps + 1)

        # Parameters:

        # AB line:
        a = opti.parameter(2)
        b = opti.parameter(2)

        # initial values:
        state_0 = opti.parameter(4)
        angle_0 = opti.parameter()
        speed = opti.parameter()
        wheel_base = opti.parameter()
        max_rate = opti.parameter()
        max_angle = opti.parameter()
        max_trailer_angle = opti.parameter()
        trailer_length = opti.parameter()
        trailer_offset = opti.parameter()
        trailer_point = opti.parameter(2)
        xtrack_weight = opti.parameter()
        heading_weight = opti.parameter()
        circle_1 = opti.parameter(3)
        radius = opti.parameter()

        # calculation of line heading:
        ba = b - a
        line_heading = ca.atan2(ba[1], ba[0])

        common_cost = 0
        xtrack = self.create_xtrack_fun()

        # cost-function:
        for i in range(steps):
            pos = states[i, :2].T
            control_point_offset = ca.vertcat(
                ca.cos(states[i, 3]) * trailer_point[0] -
                ca.sin(states[i, 3]) * trailer_point[1],
                ca.sin(states[i, 3]) * trailer_point[0] +
                ca.cos(states[i, 3]) * trailer_point[1],
            )
            trailer_pos = ca.vertcat(
                pos[0] - ca.cos(states[i, 2]) * trailer_offset -
                ca.cos(states[i, 3]) * trailer_length,
                pos[1] - ca.sin(states[i, 2]) * trailer_offset - ca.sin(states[i, 3]) * trailer_length)

            common_cost += xtrack_weight * \
                xtrack(a, b, trailer_pos + control_point_offset) ** 2
            common_cost += heading_weight * (states[i, 3] - line_heading) ** 2

            if self.SOFT_CONSTRAIN:
                # Adding a constraint for an obstacle described by a circle.
                pos = states[i, :2]
                c_pos = circle_1[:2].T
                distance = ca.norm_2(pos - c_pos)
                distance_cost = 1000. ** (-(distance - (circle_1[2] + radius)))
                common_cost += distance_cost

        opti.minimize(common_cost)

        # model constrains:
        for i in range(steps):
            # kinematic model:
            dh = states[i, 2] - states[i, 3]
            ds = ca.horzcat(
                speed * ca.cos(states[i, 2]),
                speed * ca.sin(states[i, 2]),
                speed * ca.tan(angle[i]) / wheel_base,
                speed / trailer_length * (ca.sin(dh) - trailer_offset * ca.cos(dh) * ca.tan(angle[i]) / wheel_base))
            opti.subject_to(states[i + 1, :] == states[i, :] + ds * dt)

            # dynamic limits:
            opti.subject_to(
                opti.bounded(-max_rate, (angle[i + 1] - angle[i]) / dt, max_rate))
            opti.subject_to(opti.bounded(-max_angle, angle[i], max_angle))
            opti.subject_to(opti.bounded(-max_trailer_angle,
                            states[i, 3] - states[i, 2], max_trailer_angle))
            if not self.SOFT_CONSTRAIN:
                pos = states[i, :2]
                c_pos = circle_1[:2].T
                opti.subject_to(ca.norm_2(pos - c_pos) >= (circle_1[2] + radius))

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
            'MPC', [
                states,
                a,
                b,
                state_0,
                angle_0,
                speed,
                wheel_base,
                max_rate,
                max_angle,
                max_trailer_angle,
                trailer_length,
                trailer_offset,
                trailer_point,
                xtrack_weight,
                heading_weight,
                circle_1,
                radius],
            [states, angle])

    def optimize_controls(
            self,
            a: Tuple[float, float],
            b: Tuple[float, float],
            state_0: Tuple[float, float, float, float],
            angle_0: float,
            speed: float,
            wheel_base: float,
            max_rate: float,
            max_angle: float,
            max_trailer_angle: float,
            trailer_length: float,
            trailer_offset: float,
            trailer_point: Tuple[float, float],
            xtrack_weight: float,
            heading_weight: float,
            circle_1: Tuple[float, float, float],
            radius: float) -> Tuple[
                List[Tuple[float, float, float, float]],
                List[Tuple[float, float, float, float]]]:
        """Run MPC for car model

        Args:
            a (Tuple[float, float]): A point of line
            b (Tuple[float, float]): B point of line
            state_0 (Tuple[float, float, float, float]): initial state: [x, y, heading, trailer_heading]
            angle_0 (float): initial steering angle, rad
            speed (float): speed, m
            wheel_base (float): wheel_base len of vehicle, m
            max_rate (float): max rate of steering wheel, rad/s
            max_angle (float): max angle of steering wheel, rad
            max_trailer_angle (float): Max angle between car heading and trailer heading, rad
            xtrack_weight (float, optional): weight of xtrack in control low.
            heading_weight (float, optional): weight of heading error in control low
            trailer_length (float, optional): length of trailer, m.
            trailer_offset (float, optional): offset of the trailer coupling, m
            trailer_point Tuple[float, float]: control point [x,y] in trailer coordinates, m
            circle_1 (Tuple[float, float, float]) - parameter of the obstacle circle: x,y,radius
            radius (float): radius of border around car
        Returns:
            Tuple[
                List[Tuple[float, float, float, float]],
                List[Tuple[float, float, float, float]]]: tuple of [states, control_angles]:
                    states : List[Tuple[x, y, heading, trailer_heading],...] - len depends from MPC model
                    control_angles: List[angle,...] - len depends from MPC model
        """
        sol = self.mpc_fun(
            self.last_states,
            a,
            b,
            state_0,
            angle_0,
            speed,
            wheel_base,
            max_rate,
            max_angle,
            max_trailer_angle,
            trailer_length,
            trailer_offset,
            trailer_point,
            xtrack_weight,
            heading_weight,
            circle_1,
            radius)
        self.last_states[:-1] = sol[0][1:]
        return sol

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
    mpc = MPCCasadi()

    A = [0., 0.]
    B = [100., 0.]
    STATE_0 = [0., 10, 0., 0.]
    ANGLE_0 = 0.
    SPEED = 1.
    WHEEL_BASE = 3
    MAX_RATE = math.radians(60)
    MAX_ANGLE = math.radians(25)
    MAX_TRAILER_ANGLE = math.radians(25)

    res = mpc.optimize_controls(
        A, B, STATE_0, ANGLE_0, SPEED, WHEEL_BASE,
        MAX_RATE, MAX_ANGLE, MAX_TRAILER_ANGLE)
    mpc.mpc_fun.generate('gen.c')
    print(res)
