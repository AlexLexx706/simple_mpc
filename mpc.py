import numpy as np
from scipy.optimize import minimize
import math


class MPCController:
    def __init__(
            self,
            v=20,
            l_1=50,
            l_2=50,
            l_1c=0,
            dt=0.3,
            N=3,
            xtrack_factor=1,
            heading_error_factor=60,
            delta_max_change=math.radians(35),
            max_delta_rate=math.radians(80),
            max_iterations=50):
        # Model and optimization parameters
        self.v = v  # Velocity (m/s)
        self.l_1 = l_1  # Wheelbase length
        self.l_2 = l_2  # trailer wheelbase length
        self.l_1c = l_1c

        self.dt = dt  # Time step (s)
        self.N = N  # Prediction horizon (number of steps)
        self.xtrack_factor = xtrack_factor  # Cross-track error weight
        self.heading_error_factor = heading_error_factor  # Heading error weight
        # Maximum steering angle change per step (radians)
        self.delta_max_change = delta_max_change
        # Maximum rate of change for steering angle per second (radians)
        self.max_delta_rate = max_delta_rate
        # Initial state [x, y, theta]
        self.initial_state = np.array([0, 0, 0, -self.l_2, 0, 0])
        self.line_point1 = np.array([0, 0])  # First point of the path line
        self.line_point2 = np.array([1, 1])  # Second point of the path line
        # Initial steering angles (one for each prediction step)
        self.initial_deltas = np.full(N, 0.0)
        # Maximum number of optimization iterations
        self.max_iterations = max_iterations

        # Constraints on the change of delta, considering the time step (dt)
        self.constraints = []
        # Maximum allowable change per time step
        max_change = self.max_delta_rate * self.dt

        for i in range(0, self.N - 1):
            def delta_change_constraint(deltas, i=i):
                # Ensure the change in steering angle between steps is within the allowed maximum rate
                return max_change - abs(deltas[i + 1] - deltas[i])
            self.constraints.append(
                {'type': 'ineq', 'fun': delta_change_constraint})

        initial_angle = self.initial_deltas[0]  # Initial control angle

        def initial_angle_constraint(deltas):
            # Ensure the first delta angle does not exceed the allowed maximum change
            return max_change - abs(deltas[0] - initial_angle)
        self.constraints.append(
            {'type': 'ineq', 'fun': initial_angle_constraint})

        # Constraints on the delta values (steering angles) e.g., from -30 to 30 degrees (in radians)
        self.bounds = [
            (-self.delta_max_change, self.delta_max_change)] * self.N

    def set_model_parameters(self, v, L, dt):
        # Set the vehicle model parameters (velocity, wheelbase length, time step)
        self.v = v
        self.l_1 = L
        self.dt = dt

    def set_cost_parameters(self, xtrack_factor, heading_error_factor):
        # Set the parameters for the cost function (weights for cross-track error and heading error)
        self.xtrack_factor = xtrack_factor
        self.heading_error_factor = heading_error_factor

    def set_initial_state(self, initial_state):
        # Set the initial state of the vehicle [x, y, theta]
        self.initial_state = np.array(initial_state)

    def set_initial_deltas(self, initial_angle):
        # Set the initial steering angles for each prediction step
        self.initial_deltas = np.full(self.N, initial_angle)

    def set_path_parameters(self, point1, point2):
        # Set the two points defining the path (line) the vehicle should follow
        self.line_point1 = np.array(point1)
        self.line_point2 = np.array(point2)

    @staticmethod
    def vehicle_model(state, v, l, delta):
        """
        Vehicle dynamics model to predict the next state based on the current state and steering angle.

        Args:
            state (np.array): A 1D array representing the current state of the vehicle [x, y, theta], where:
                x (float) - current x-coordinate of the vehicle,
                y (float) - current y-coordinate of the vehicle,
                theta (float) - current orientation angle of the vehicle in radians.
            delta (float): Steering angle in radians.
            v (float): speed
            l (float): base len

        Returns:
            np.array: A 1D array representing the change in state [dx, dy, d_theta], where:
                dx (float) - change in the x-coordinate,
                dy (float) - change in the y-coordinate,
                d_theta (float) - change in the orientation angle.
        """
        theta = state[2]
        vx = v * np.cos(theta)
        vy = v * np.sin(theta)
        d_theta = (v / l) * np.tan(delta)
        return np.array([vx, vy, d_theta])

    # Kinematic model of a car with a trailer

    @staticmethod
    def trailer_model(state, v_1, delta, l_1=2.5, l_2=3.5, l_1c=1):
        """
        Function to predict the state changes of a car with a trailer.

        Parameters:
        state (dict): Current state of the model with keys:
            'car_pos' - position of the car (x, y),
            'car_theta' - orientation of the car (angle),
            'trailer_pos' - position of the trailer (x, y),
            'trailer_theta' - orientation of the trailer (angle).
        v_1 (float): Speed of the car.
        delta (float): New steering angle of the car's wheels (in radians).
        dt (float): Integration step (in seconds).
        l_1 (float): Length of the car.
        l_2 (float): Length of the trailer.
        l_1c (float): Offset of the trailer coupling.

        Returns:
        dict: The new state of the model.
        """

        # Extract current state values
        psi_1 = state[2]
        psi_2 = state[5]

        # process model
        gamma_1 = psi_1 - psi_2
        psi_dot_1 = v_1 * np.tan(delta) / l_1
        v_2 = v_1 * np.cos(gamma_1) - psi_dot_1 * l_1c * np.sin(gamma_1)
        psi_dot_2 = (v_1 * np.sin(gamma_1) + psi_dot_1 * l_1c * np.cos(gamma_1)) / l_2

        return np.array([
            v_1 * np.cos(psi_1),
            v_1 * np.sin(psi_1),
            psi_dot_1,
            v_2 * np.cos(psi_2),
            v_2 * np.sin(psi_2),
            psi_dot_2])

    def predict_trajectory(self, state, deltas):
        # Predict the vehicle's trajectory over the prediction horizon
        trajectory = [state]
        current_state = state.copy()

        for i in range(self.N):
            current_state += self.trailer_model(
                current_state,
                self.v,
                deltas[i],
                self.l_1,
                self.l_2,
                self.l_1c) * self.dt

            trajectory.append(current_state)
        return np.array(trajectory)

    def distance_point_to_line(self, point):
        # Calculate the perpendicular distance from a point to the path line
        line_vec = self.line_point2 - self.line_point1
        point_vec = point - self.line_point1
        line_length = np.linalg.norm(line_vec)
        line_unit_vec = line_vec / line_length
        # Perpendicular unit vector of the line
        perp_line_unit_vec = np.array([-line_unit_vec[1], line_unit_vec[0]])
        return np.dot(point_vec, perp_line_unit_vec)

    def cost_function(self, deltas):
        # Compute the cost function based on the predicted trajectory and path line
        trajectory = self.predict_trajectory(self.initial_state, deltas)
        cost = 0
        line_vec = self.line_point2 - self.line_point1
        line_angle = np.arctan2(line_vec[1], line_vec[0])

        for k in range(self.N):
            x, y, theta, t_x, t_y, t_theta = trajectory[k]
            # point = np.array([t_x, t_y])
            point = np.array([x, y])

            # Distance from the point to the line
            distance = self.distance_point_to_line(point)
            orientation_error = abs(theta - line_angle)
            # Add to the total cost
            cost += self.xtrack_factor * distance**2 + \
                self.heading_error_factor * orientation_error**2

        return cost

    def optimize_controls(self):
        # Solve the optimization problem to minimize the cost function
        result = minimize(
            self.cost_function,
            self.initial_deltas,
            bounds=self.bounds,
            constraints=self.constraints,
            method='SLSQP',
            options={'maxiter': self.max_iterations})

        if result.success:
            return result.x  # Return the optimal steering angles
        else:
            raise ValueError("Optimization failed: " + result.message)


if __name__ == "__main__":
    # Example usage
    mpc = MPCController(max_iterations=50)  # Limit the number of iterations
    mpc.set_model_parameters(v=5, L=2.5, dt=0.1)
    mpc.set_cost_parameters(xtrack_factor=1, heading_error_factor=1)
    mpc.set_initial_state([1, 2, np.pi/4])  # Initial state
    mpc.set_initial_deltas(0.05)  # Initial steering angle
    mpc.set_path_parameters([0, 0], [5, 5])  # Path parameters

    optimal_deltas = mpc.optimize_controls()
    print("Optimal steering angles (in radians):", optimal_deltas)
