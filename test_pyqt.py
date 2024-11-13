import sys
import math
import numpy as np
import time
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsLineItem
from mpc import MPCController
import logging

LOG = logging.getLogger(__name__)


class CarModel(QGraphicsItem):
    PERIOD = 10.  # Time period for steering change
    MAX_ANGLE = math.radians(20.)  # Maximum steering angle in radians

    def __init__(self):
        super().__init__()
        self.length = 50  # Length of the car base (in pixels)
        self.width = 20  # Width of the car (in pixels)

        self.trailer_len = 60  # Length of the trailer
        self.trailer_width = 10  # Length of the trailer
        self.trailer_theta = 0.

        self.trailer_rect = QRectF(
            -self.trailer_len,
            -self.trailer_width / 2.,
            self.trailer_len,
            self.trailer_width)

        # Rectangle for the car body
        self.body_rect = QRectF(
            0., int(-self.width / 2), self.length, self.width)
        self.speed = 20  # Car speed (pixels per second)
        self.steering_angle = 0.  # Initial steering angle (in radians)
        self.state = np.array([0., 0., 0., -self.trailer_len, 0., 0.])  # Initial state: [x, y, theta]
        self.last_time = None  # Last timestamp for time delta calculation
        self.wheel_len = 10.  # Length of the wheels (in pixels)
        self.wheel_width = 5.  # Width of the wheels (in pixels)

        # Rectangles for the car wheels
        self.wheel_rect = QRectF(
            -self.wheel_len / 2.,
            -self.wheel_width / 2.,
            self.wheel_len,
            self.wheel_width
        )

        # Boundaries for the car movement
        self.bounds = QRectF(
            -100,
            -100,
            200,
            200)

        self.mpc = MPCController(
            v=self.speed,
            l_1=self.length,
            l_2=self.trailer_len
        )  # Initialize MPC controller with speed and car length

    def move(self):
        # Moves the car by updating its position and orientation
        cur_time = time.time()
        if self.last_time:
            dt = cur_time - self.last_time  # Calculate time delta
            # Choose whether to use a sinusoidal steering change or MPC-based steering
            if 0:  # Example for sinusoidal steering (disabled by setting to 0)
                self.steering_angle = math.sin(
                    time.time() / self.PERIOD * 2 * math.pi) * self.MAX_ANGLE
            else:
                # Use the MPC controller to optimize steering angles
                self.mpc.set_initial_state(self.state)
                self.mpc.set_initial_deltas(self.steering_angle)
                try:
                    res = self.mpc.optimize_controls()  # Get optimized control actions from MPC
                    # Update the steering angle based on optimization result
                    self.steering_angle = res[0]
                except ValueError as e:
                    LOG.error(e)  # Log error if optimization fails

            # self.steering_angle = math.radians(-20)

            # Update car state
            self.state += self.mpc.trailer_model(
                self.state,
                self.speed,
                self.steering_angle,
                self.length,
                self.trailer_len,
                0) * dt

        self.last_time = time.time()
        self.setPos(self.state[0], self.state[1])  # Update car position
        self.setRotation(math.degrees(self.state[2]))  # Update car orientation
        self.update()

    def boundingRect(self):
        # Return the bounding rectangle of the car for collision detection
        return self.bounds

    def paint(self, painter, option, widget):
        # Paint the car model (body and wheels) using QPainter
        painter.setBrush(QColor(0, 0, 255))  # Blue color for the car body
        painter.drawRect(self.body_rect)

        # Draw the front wheels
        painter.setBrush(QColor(255, 0, 0))  # Red color for wheels
        painter.save()
        painter.translate(0, -self.width / 2)
        painter.drawRect(self.wheel_rect)
        painter.restore()

        painter.save()
        painter.translate(self.length, -self.width / 2)
        # Rotate the wheels based on the steering angle
        painter.rotate(math.degrees(self.steering_angle))
        painter.drawRect(self.wheel_rect)
        painter.restore()

        painter.save()
        painter.translate(0, self.width / 2)
        painter.drawRect(self.wheel_rect)
        painter.restore()

        painter.save()
        painter.translate(self.length, self.width / 2)
        # Rotate the wheels based on the steering angle
        painter.rotate(math.degrees(self.steering_angle))
        painter.drawRect(self.wheel_rect)
        painter.restore()

        painter.save()
        # Rotate the wheels based on the steering angle
        trailer_theta = math.degrees(self.state[2] - self.state[5])
        painter.rotate(-trailer_theta)
        painter.drawRect(self.trailer_rect)
        painter.restore()




A = (-2000, 50)  # Start point of the path
B = (100, -550)  # End point of the path


class GridScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.setSceneRect(-5000, -5000, 10000, 10000)  # Set scene size
        self.grid_size = 50  # Grid size for background grid
        self.car = CarModel()  # Create car model
        self.addItem(self.car)  # Add car to the scene

        # Add a black line to the scene representing the path
        self.black_line = QGraphicsLineItem(A[0], A[1], B[0], B[1])
        # Set line color and width
        self.black_line.setPen(QPen(QColor(0, 0, 0), 2))
        self.addItem(self.black_line)
        # Set the path for the MPC controller
        self.car.mpc.set_path_parameters(A, B)

        # Timer to periodically update car movement
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateCar)
        self.timer.start(1000 // 20)  # 20 FPS

    def updateCar(self):
        self.car.move()  # Move the car based on optimized steering angles

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)

        # Draw a grid background
        grid_color = QColor(230, 230, 230)
        painter.setPen(grid_color)

        left = math.floor(rect.left() / self.grid_size) * self.grid_size
        top = math.floor(rect.top() / self.grid_size) * self.grid_size
        right = math.ceil(rect.right() / self.grid_size) * self.grid_size
        bottom = math.ceil(rect.bottom() / self.grid_size) * self.grid_size

        # Draw vertical grid lines
        for x in range(int(left), int(right), self.grid_size):
            painter.drawLine(x, top, x, bottom)
        # Draw horizontal grid lines
        for y in range(int(top), int(bottom), self.grid_size):
            painter.drawLine(left, y, right, y)


class CameraView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

    def wheelEvent(self, event):
        # Zoom in and out with the mouse wheel
        factor = 1.1
        if event.angleDelta().y() < 0:
            factor = 1 / factor
        self.scale(factor, factor)


def main():
    logging.basicConfig(level=logging.DEBUG)  # Set logging level to DEBUG
    app = QApplication(sys.argv)  # Initialize the Qt application
    scene = GridScene()  # Create the scene
    view = CameraView(scene)  # Create the camera view for rendering the scene
    view.setRenderHint(QPainter.Antialiasing)
    view.setRenderHint(QPainter.SmoothPixmapTransform)
    view.setScene(scene)  # Set the scene to the view
    view.setWindowTitle("Car Animation with Ackermann Steering")
    view.setGeometry(100, 100, 800, 600)  # Set window size and position
    view.show()  # Show the window
    sys.exit(app.exec_())  # Run the application loop


if __name__ == "__main__":
    main()
