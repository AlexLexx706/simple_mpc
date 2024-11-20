import math
import numpy as np
import time
import logging
from typing import Tuple
from PyQt5 import QtCore
from PyQt5 import Qt
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc import mpc_back_box
from mpc import mpc_casadi

LOG = logging.getLogger(__name__)


class CarModel(QtWidgets.QGraphicsItem):
    """Simple visualization of car with trailer
    """
    PERIOD = 10.  # Time period for steering change
    MAX_ANGLE = math.radians(25.)  # Maximum steering angle in radians
    MAX_RATE = math.radians(30)  # Maximum steering angle in radians
    MAX_TRAILER_ANGLE = math.radians(45)  # Maximum steering angle in radians

    LINE_PEN = QtGui.QPen(QtGui.QColor(0, 0, 0), 2)
    LINE_PEN.setCosmetic(True)

    BODY_COLOR = QtGui.QColor(0, 0, 255)  # Blue color for the car body
    WHEEL_COLOR = QtGui.QColor(255, 0, 0)

    CONTROL_POINT_PEN = QtGui.QPen(QtGui.QColor(0, 0, 255), 10)
    CONTROL_POINT_PEN.setCosmetic(True)
    CONTROL_POINT_PEN.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)

    CIRCLE_BRUSH = QtGui.QBrush(QtCore.Qt.black, QtCore.Qt.NoBrush)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.length = 3  # Length of the car base, m
        self.width = 2  # Width of the car, m
        self.speed = -5.  # Car speed m/s
        self.steering_angle = 0.  # Initial steering angle (in radians)
        self.trailer_len = 5  # Length of the trailer, m
        self.trailer_point = [0., 3.]  # Length of the trailer, m
        self.line = ((-2000, -100), (2000, 50))
        self.radius = 5
        self.circle_1 = [0, 0, 10]

        # Initial state: [x, y, heading, trailer_heading, trailer_x, trailer_y]
        heading = math.radians(20)
        self.state = np.array([0., 0., heading, 0., -self.trailer_len, 0.])

        self.trailer_width = 2.  # Length of the trailer, m
        self.wheel_len = 1.  # Length of the wheels, m
        self.wheel_width = 0.3  # Width of the wheels, m

        self.trailer_rect = QtCore.QRectF(
            -self.trailer_len,
            -self.trailer_width / 2.,
            self.trailer_len,
            self.trailer_width)

        # Rectangle for the car body
        self.body_rect = QtCore.QRectF(
            0., int(-self.width / 2),
            self.length, self.width)

        # Rectangles for the car wheels
        self.wheel_rect = QtCore.QRectF(
            -self.wheel_len / 2.,
            -self.wheel_width / 2.,
            self.wheel_len,
            self.wheel_width
        )

        # Boundaries for the car movement
        self.bounds = QtCore.QRectF(
            -100,
            -100,
            200,
            200)

        # MPC controller
        self.mpc = mpc_casadi.MPCCasadi(
            trailer_length=self.trailer_len,
            trailer_point=self.trailer_point)

    def set_circle_1(self, circle_1: Tuple[float, float, float]):
        """Setup parameters of the obstacle circle: x,y,radius

        Args:
            circle_1 (Tuple[float, float, float]): x, y, radius
        """
        self.circle_1 = circle_1

    def set_line(self, line: Tuple[Tuple[float, float], Tuple[float, float]]):
        """Set line for MCP

        Args:
            line (Tuple[Tuple[float, float], Tuple[float, float]]): line - A, B points
        """
        self.line = line

    def set_heading(self, heading: float):
        """set car heading, rad

        Args:
            heading (float): heading in rad
        """
        dh = heading - self.state[2]
        self.state[2] = heading
        self.state[3] = self.state[3] + dh
        self.setRotation(math.degrees(self.state[2]))  # Update car orientation
        self.update()

    def get_heading(self) -> float:
        """get heading, rad

        Returns:
            float: heading in rad
        """
        return self.state[2]

    def set_trailer_angle(self, angle: float):
        """set trailer angle, rad

        Args:
            heading (float): angle in rad
        """
        self.state[3] = self.state[2] + angle
        self.update()

    def get_trailer_angle(self) -> float:
        """return trailer angle, rad

        Returns:
            float: trailer angle, rad
        """
        return self.state[3] - self.state[2]

    def set_state(self, pos: Tuple[float, float], heading: float, trailer_angle: float):
        """Set state of car
        Args:
            pos (Tuple[float, float]): position
            heading (float): orientation
            trailer_angle (float): trailer angle
        """
        t_heading = heading + trailer_angle
        self.state = np.array([
            pos[0], pos[1],
            heading,
            t_heading,
            pos[0] - np.cos(t_heading) * self.trailer_len,
            pos[1] - np.sin(t_heading) * self.trailer_len])

        self.setPos(self.state[0], self.state[1])  # Update car position
        self.setRotation(math.degrees(self.state[2]))  # Update car orientation
        self.update()

    def move(self, dt):
        """Moves the car by updating its position and orientation"""
        try:
            self.steering_angle = self.mpc.optimize_controls(
                self.line[0],
                self.line[1],
                self.state[:4],
                self.steering_angle,
                self.speed,
                self.length,
                self.MAX_RATE,
                self.MAX_ANGLE,
                self.MAX_TRAILER_ANGLE,
                self.circle_1,
                self.radius)
        except ValueError as e:
            LOG.error(e)  # Log error if optimization fails

        # Update car state
        self.state += mpc_back_box.MPCBlackBox.trailer_model(
            self.state,
            self.speed,
            self.steering_angle,
            self.length,
            self.trailer_len,
            0) * dt

        self.setPos(self.state[0], self.state[1])  # Update car position
        self.setRotation(math.degrees(self.state[2]))  # Update car orientation
        self.update()

    def boundingRect(self) -> QtCore.QRectF:
        """Return bounding box of object

        Returns:
            QtCore.QRectF: bounding box
        """
        return self.bounds

    def paint(self, painter: QtGui.QPainter, option, widget):
        """Draw car and trailer

        Args:
            painter (QtGui.QPainter): painter
            option (QStyleOptionGraphicsItem): options
            widget (QWidget): widget
        """
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
            QtCore.QPointF(self.trailer_point[0] - self.trailer_len, self.trailer_point[1]))
        painter.restore()

        painter.setBrush(self.CIRCLE_BRUSH)
        painter.drawEllipse(QtCore.QRectF(
            -self.radius,
            -self.radius,
            2 * self.radius,
            2 * self.radius))
