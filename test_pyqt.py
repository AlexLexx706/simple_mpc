import sys
import math
import numpy as np
import time
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsLineItem
import mpc
import logging
import mpc_casadi

LOG = logging.getLogger(__name__)


A = (-2000, -100)  # Start point of the path
B = (2000, 50)  # End point of the path


class CarModel(QGraphicsItem):
    PERIOD = 10.  # Time period for steering change
    MAX_ANGLE = math.radians(25.)  # Maximum steering angle in radians
    MAX_RATE = math.radians(30)  # Maximum steering angle in radians

    LINE_PEN = QPen(QColor(0, 0, 0), 2)
    LINE_PEN.setCosmetic(True)

    BODY_COLOR = QColor(0, 0, 255)  # Blue color for the car body
    WHEEL_COLOR = QColor(255, 0, 0)

    CONTROL_POINT_PEN = QPen(QColor(0, 0, 255), 10)
    CONTROL_POINT_PEN.setCosmetic(True)
    CONTROL_POINT_PEN.setCapStyle(Qt.PenCapStyle.RoundCap)

    def __init__(self):
        super().__init__()
        self.length = 3  # Length of the car base, m
        self.width = 2  # Width of the car, m
        self.speed = -5.  # Car speed m/s
        self.steering_angle = 0.  # Initial steering angle (in radians)
        self.trailer_len = 5  # Length of the trailer, m
        self.trailer_point = [0., 3.]  # Length of the trailer, m

        # Initial state: [x, y, heading, trailer_heading, trailer_x, trailer_y]
        self.state = np.array([0., 0., 0., 0., -self.trailer_len, 0.])

        self.trailer_width = 2.  # Length of the trailer, m
        self.wheel_len = 1.  # Length of the wheels, m
        self.wheel_width = 0.3  # Width of the wheels, m

        self.trailer_rect = QRectF(
            -self.trailer_len,
            -self.trailer_width / 2.,
            self.trailer_len,
            self.trailer_width)

        # Rectangle for the car body
        self.body_rect = QRectF(
            0., int(-self.width / 2), self.length, self.width)

        self.last_time = None  # Last timestamp for time delta calculation

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

        # MPC controller
        self.mpc = mpc_casadi.Mpc(
            trailer_length=self.trailer_len,
            trailer_point=self.trailer_point)

    def move(self):
        # Moves the car by updating its position and orientation
        cur_time = time.time()
        if self.last_time:
            dt = cur_time - self.last_time  # Calculate time delta
            if 0:
                # Example for sinusoidal steering (disabled by setting to 0)
                self.steering_angle = math.sin(
                    time.time() / self.PERIOD * 2 * math.pi) * self.MAX_ANGLE
            else:
                try:
                    self.steering_angle = self.mpc.optimize_controls(
                        A,
                        B,
                        self.state[:4],
                        self.steering_angle,
                        self.speed,
                        self.length,
                        self.MAX_RATE,
                        self.MAX_ANGLE)
                except ValueError as e:
                    LOG.error(e)  # Log error if optimization fails

            # Update car state
            self.state += mpc.MPCController.trailer_model(
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
        painter.setPen(self.LINE_PEN)
        painter.setBrush(self.BODY_COLOR)
        painter.drawRect(self.body_rect)

        # Draw the left rear wheels
        painter.setBrush(self.WHEEL_COLOR)  # Red color for wheels
        painter.save()
        painter.translate(0, -self.width / 2)
        painter.drawRect(self.wheel_rect)
        painter.restore()

        # Draw the left front wheels
        painter.save()
        painter.translate(self.length, -self.width / 2)
        painter.rotate(math.degrees(self.steering_angle))
        painter.drawRect(self.wheel_rect)
        painter.restore()

        # Draw the right rear wheels
        painter.save()
        painter.translate(0, self.width / 2)
        painter.drawRect(self.wheel_rect)
        painter.restore()

        # Draw the right front wheels
        painter.save()
        painter.translate(self.length, self.width / 2)
        painter.rotate(math.degrees(self.steering_angle))
        painter.drawRect(self.wheel_rect)
        painter.restore()

        # Draw trailer
        painter.save()
        trailer_theta = math.degrees(self.state[2] - self.state[3])
        painter.rotate(-trailer_theta)
        painter.drawRect(self.trailer_rect)
        painter.restore()

        # draw control point
        painter.save()
        painter.rotate(-trailer_theta)
        painter.setPen(self.CONTROL_POINT_PEN)
        painter.drawPoint(
            QPointF(self.trailer_point[0] - self.trailer_len, self.trailer_point[1]))
        painter.restore()


class GridScene(QGraphicsScene):
    LINE_PEN = QPen(QColor(0, 0, 0), 2)
    LINE_PEN.setCosmetic(True)

    GREED_PEN = QPen(QColor(230, 230, 230), 2)
    GREED_PEN.setCosmetic(True)
    GRID_SIZE = 10  # Grid size for background grid

    def __init__(self):
        super().__init__()
        self.setSceneRect(-5000, -5000, 10000, 10000)  # Set scene size
        self.car = CarModel()  # Create car model
        self.addItem(self.car)  # Add car to the scene

        # Add a black line to the scene representing the path
        self.black_line = QGraphicsLineItem(A[0], A[1], B[0], B[1])
        # Set line color and width
        self.black_line.setPen(self.LINE_PEN)
        self.addItem(self.black_line)
        # Set the path for the MPC controller
        # self.car.mpc.set_path_parameters(A, B)

        # Timer to periodically update car movement
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateCar)
        self.timer.start(100)  # 10 FPS

    def updateCar(self):
        self.car.move()  # Move the car based on optimized steering angles

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)

        # Draw a grid background
        painter.setPen(self.GREED_PEN)

        left = math.floor(rect.left() / self.GRID_SIZE) * self.GRID_SIZE
        top = math.floor(rect.top() / self.GRID_SIZE) * self.GRID_SIZE
        right = math.ceil(rect.right() / self.GRID_SIZE) * self.GRID_SIZE
        bottom = math.ceil(rect.bottom() / self.GRID_SIZE) * self.GRID_SIZE

        # Draw vertical grid lines
        for x in range(int(left), int(right), int(self.GRID_SIZE)):
            painter.drawLine(x, top, x, bottom)
        # Draw horizontal grid lines
        for y in range(int(top), int(bottom), self.GRID_SIZE):
            painter.drawLine(left, y, right, y)


class CameraView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.scale(10, 10)

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
