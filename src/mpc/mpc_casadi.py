from typing import Tuple, List
import math
import casadi as ca
import math


class MPCCasadi:
    def __init__(
            self,
            dt: float,
            steps: int,
            max_iter: int,
            soft_constrain: bool,
            circles_num: int,
            path_len: int) -> None:
        """Build MPC for controlling trailer points

        Args:
            dt (float, optional): integration step, s. Defaults to 0.5.
            steps (int, optional): number of steps - horizon prediction. Defaults to 10.
            max_iter (int, optional): limit of max iterations for solver. Defaults to 50.
            soft_constrain (bool):
                true - Soft constraint for the obstacle is used;
                false - Hard constraint used;
            circles_num (int): - the number of circle constraints.
        """
        self.dt = dt
        self.steps = steps
        self.last_states = None
        self.mpc = self.build_mpc(
            dt,
            steps,
            max_iter,
            soft_constrain,
            circles_num,
            path_len)

    @staticmethod
    def build_mpc(
            dt: float,
            steps: int,
            max_iter: int,
            soft_constrain: bool,
            circles_num: int,
            path_len:int) -> None:
        """Build MPC for controlling trailer points

        Args:
            dt (float, optional): integration step, s. Defaults to 0.5.
            steps (int, optional): number of steps - horizon prediction. Defaults to 10.
            max_iter (int, optional): limit of max iterations for solver. Defaults to 50.
            soft_constrain (bool):
                true - Soft constraint for the obstacle is used;
                false - Hard constraint used;
            circles_num (int): - the number of circle constraints.
            path_len (int): - nambers of point in steering path
        """
        opti = ca.Opti()

        # state: x, y, heading, trailer_heading:
        states = opti.variable(steps + 1, 4)

        # N + 1 control angle:
        angle = opti.variable(steps + 1)

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
        radius = opti.parameter()
        circles = opti.parameter(circles_num, 3)
        path = opti.parameter(path_len, 2)

        common_cost = 0

        # cost-function:
        for i in range(steps):
            pos = states[i, :2].T
            control_point_offset = ca.horzcat(
                ca.cos(states[i, 3]) * trailer_point[0] -
                ca.sin(states[i, 3]) * trailer_point[1],
                ca.sin(states[i, 3]) * trailer_point[0] +
                ca.cos(states[i, 3]) * trailer_point[1],
            )
            control_point = control_point_offset + ca.horzcat(
                pos[0] - ca.cos(states[i, 2]) * trailer_offset -
                ca.cos(states[i, 3]) * trailer_length,
                pos[1] - ca.sin(states[i, 2]) * trailer_offset - ca.sin(states[i, 3]) * trailer_length)

            # adding xtrack, heading_error to cost function
            xtrack, heading_error = MPCCasadi.get_track_cost(path, control_point, states[i, 3])
            common_cost +=  xtrack_weight * xtrack **2
            common_cost +=  heading_weight * heading_error **2

            if soft_constrain:
                # Adding a constraint for an obstacle described by a circle.
                pos = states[i, :2]
                for i in range(circles_num):
                    c_pos = circles[i,:2]
                    distance = ca.norm_2(pos - c_pos)
                    distance_cost = 1000. ** (-(distance - (circles[i, 2] + radius)))
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

            # hard constrain for obstacles
            if not soft_constrain:
                pos = states[i, :2]
                for i in range(circles_num):
                    c_pos = circles[i, :2]
                    opti.subject_to(ca.norm_2(pos - c_pos) >= (circles[i, 2] + radius))

        # initial constrain:
        opti.subject_to(states[0, :] == state_0.T)
        opti.subject_to(angle[0] == angle_0)

        opti.solver(
            'ipopt',
            {
                'ipopt.print_level': 0,
                'print_time': 0,
                'ipopt.sb': 'yes'},
            {
                "max_iter": max_iter,
                'acceptable_tol':0.001}
        )

        return opti.to_function(
            'MPC', [
                states,
                path,
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
                circles,
                radius],
            [states, angle])

    @staticmethod
    def get_track_values(a:Tuple[float, float], b:Tuple[float, float], pos:Tuple[float, float], heading:float)->Tuple[float, float]:
        """Get track values

        Args:
            a (Tuple[float, float]): A point of the line: [x,y]
            b (Tuple[float, float]): B point of the line: [x,y]
            pos (Tuple[float, float]): vehicle pos: [x,y]
            heading (float): vehicle heading, rad
        Returns:
            Tuple[float, float]: xtrack, heading_error, t
        """

        ba = b - a
        # Checking if the projection of a point belongs to a segment
        denominator = ba[0] ** 2 + ba[1] ** 2
        t = ((pos[0] - a[0]) * ba[0] + (pos[1] - a[1]) * ba[1]) / denominator


        ba_norm = ba / ca.norm_2(ba)
        xtrack = ca.norm_2(pos - (a + (ba_norm @ (pos - a).T) * ba_norm))
        heading = ca.acos(ca.horzcat(ca.cos(heading), ca.sin(heading)) @ ba_norm.T)
        heading = ca.if_else(heading > ca.pi / 2., ca.pi - heading, heading)
        return xtrack, heading, t

    @staticmethod
    def get_track_cost(
            path:List[Tuple[float, float]],
            control_point:Tuple[float, float],
            heading:float)->Tuple[float, float]:
        """Calculate cost for track

        Args:
            path (List[Tuple[float, float]]): path points: [x,y]
            control_point (Tuple[float, float]): vehicle point: [x,y]
            heading (float): vehicle heading, rad

        Returns:
            Tuple[float, float]: xtrack, heading
        """
        min_xt = ca.inf
        min_he = ca.inf

        # generate graphs of path computation
        for i in range(1, path.shape[0]):
            a = path[i - 1, :]
            b = path[i, :]
            xt, he, t = MPCCasadi.get_track_values(a, b, control_point, heading)

            # save first xt and he
            if i == 1:
                first_xt = xt
                first_he = he
                first_len = ca.norm_2(control_point - path[i - 1, :])

            condition = ca.logic_and(ca.logic_and(t >= 0, t < 1.), xt < min_xt)
            min_xt = ca.if_else(condition, xt, min_xt)
            min_he = ca.if_else(condition, he, min_he)
        last_len = ca.norm_2(control_point - path[i, :])

        # if out size path
        xt = ca.if_else(min_xt == ca.inf, ca.if_else(first_len < last_len, first_xt, xt), min_xt)
        he = ca.if_else(min_xt == ca.inf, ca.if_else(first_len < last_len, first_he, he), min_he)

        return xt, he

    def optimize_controls(
            self,
            path: List[Tuple[float, float]],
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
            circles: List[Tuple[float, float, float]],
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
            circles (List[Tuple[float, float, float]]) - list of parameters of the obstacle circles: x,y,radius
            radius (float): radius of border around car
        Returns:
            Tuple[
                List[Tuple[float, float, float, float]],
                List[Tuple[float, float, float, float]]]: tuple of [states, control_angles]:
                    states : List[Tuple[x, y, heading, trailer_heading],...] - len depends from MPC model
                    control_angles: List[angle,...] - len depends from MPC model
        """

        # first init
        if self.last_states is  None:
            states = [[
                state_0[0],
                state_0[1],
                state_0[2],
                state_0[3]]]

            for _ in range(1, self.steps + 1):
                states.append([
                    states[-1][0] + math.cos(states[-1][2]) * speed * self.dt,
                    states[-1][1] + math.sin(states[-1][2]) * speed * self.dt,
                    states[-1][2],
                    states[-1][3]])

            self.last_states = ca.DM(states)

        sol = self.mpc(
            self.last_states,
            path,
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
            circles,
            radius)
        self.last_states[:-1] = sol[0][1:]
        return sol

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
