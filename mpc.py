import numpy as np
from scipy.optimize import minimize
import math


class MPCController:
    def __init__(
            self,
            v=20,
            L=50,
            dt=0.3,
            N=3,
            xtrack_factor=1,
            heading_error_factor=60,
            delta_max_change=math.radians(35),
            max_delta_rate=math.radians(80),
            max_iterations=50):
        # Model and optimization parameters
        self.v = v  # Velocity (m/s)
        self.L = L  # Wheelbase length (m)
        self.dt = dt  # Time step (s)
        self.N = N  # Prediction horizon (number of steps)
        self.xtrack_factor = xtrack_factor  # Cross-track error weight
        self.heading_error_factor = heading_error_factor  # Heading error weight
        # Maximum steering angle change per step (radians)
        self.delta_max_change = delta_max_change
        # Maximum rate of change for steering angle per second (radians)
        self.max_delta_rate = max_delta_rate
        self.initial_state = np.array([0, 0, 0])  # Initial state [x, y, theta]
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
        self.L = L
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

    def vehicle_model(self, state, delta):
        # Vehicle dynamics model to predict next state based on current state and steering angle
        x, y, theta = state
        dx = self.v * np.cos(theta)
        dy = self.v * np.sin(theta)
        dtheta = (self.v / self.L) * np.tan(delta)
        return np.array([dx, dy, dtheta])

    def predict_trajectory(self, state, deltas):
        # Predict the vehicle's trajectory over the prediction horizon
        trajectory = [state]
        current_state = state.copy()
        for i in range(self.N):
            current_state = current_state + \
                self.vehicle_model(current_state, deltas[i]) * self.dt
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
            x, y, theta = trajectory[k]
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
