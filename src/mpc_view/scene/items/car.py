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
from mpc_view import scene

LOG = logging.getLogger(__name__)


class CarModel(QtWidgets.QGraphicsItemGroup):
    """Visualization of car with trailer
    """
    PERIOD = 10.  # Time period for steering change
    MAX_ANGLE = math.radians(25.)  # Maximum steering angle in radians
    MAX_RATE = math.radians(30)  # Maximum steering angle in radians
    MAX_TRAILER_ANGLE = math.radians(30)  # Maximum steering angle in radians
    WIDTH = 3  # default width base, m
    WHEEL_BASE = 5  # default wheel base, m
    TRAILER_WIDTH = 2.  # default Length of the trailer, m
    WHEEL_LEN = 1.  # default Length of the wheels, m
    WHEEL_WIDTH = 0.3  # default Width of the wheels, m
    STEERING_ANGLE = 0.  # Initial steering angle (in radians)
    TRAILER_LEN = 5  # Length of the trailer, m
    TRAILER_OFFSET = 0.  # offset of trailer joint, m
    TRAILER_CTRL_POINT = [0., 3.]  # Steering point in trailer frame, m
    RADIUS = 5.  # bounding circle radius, m
    SPEED = 5   # Car speed m/s

    XTRACK_WEIGHT = 1.
    HEADING_WEIGHT = 30

    LINE_PEN = QtGui.QPen(QtGui.QColor(0, 0, 0), 2)
    LINE_PEN.setCosmetic(True)

    BODY_COLOR = QtGui.QColor(0, 0, 255)  # Blue color for the car body
    WHEEL_COLOR = QtGui.QColor(255, 0, 0)

    CONTROL_POINT_PEN = QtGui.QPen(QtGui.QColor(0, 0, 255), 10)
    CONTROL_POINT_PEN.setCosmetic(True)
    CONTROL_POINT_PEN.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)

    CIRCLE_BRUSH = QtGui.QBrush(QtCore.Qt.black, QtCore.Qt.NoBrush)

    CTRL_POINT_RADIUS = 0.3
    CTRL_POINT_PEN = QtGui.QPen(QtCore.Qt.black, 2)
    CTRL_POINT_PEN.setCosmetic(True)
    CTRL_POINT_BRUSH = QtCore.Qt.black

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.speed = self.SPEED

        # set geometry of the car:
        self.body = QtWidgets.QGraphicsRectItem(
            0,
            -self.WIDTH / 2.,
            self.WHEEL_BASE,
            self.WIDTH,
            self)
        self.body.setPen(self.LINE_PEN)
        self.body.setBrush(self.BODY_COLOR)

        self.front_left_wheel = QtWidgets.QGraphicsRectItem(
            -self.WHEEL_LEN / 2.,
            -self.WHEEL_WIDTH / 2.,
            self.WHEEL_LEN,
            self.WHEEL_WIDTH,
            self)
        self.front_left_wheel.setPos(self.WHEEL_BASE, -self.WIDTH / 2.)
        self.front_left_wheel.setPen(self.LINE_PEN)
        self.front_left_wheel.setBrush(self.WHEEL_COLOR)

        self.front_right_wheel = QtWidgets.QGraphicsRectItem(
            self.front_left_wheel.rect(),
            self)
        self.front_right_wheel.setPos(self.WHEEL_BASE, self.WIDTH / 2.)
        self.front_right_wheel.setPen(self.LINE_PEN)
        self.front_right_wheel.setBrush(self.WHEEL_COLOR)

        self.rear_left_wheel = QtWidgets.QGraphicsRectItem(
            self.front_left_wheel.rect(),
            self)
        self.rear_left_wheel.setPos(0., -self.WIDTH / 2.)
        self.rear_left_wheel.setPen(self.LINE_PEN)
        self.rear_left_wheel.setBrush(self.WHEEL_COLOR)

        self.rear_right_wheel = QtWidgets.QGraphicsRectItem(
            self.front_left_wheel.rect(),
            self)
        self.rear_right_wheel.setPos(0., self.WIDTH / 2.)
        self.rear_right_wheel.setPen(self.LINE_PEN)
        self.rear_right_wheel.setBrush(self.WHEEL_COLOR)

        self.trailer = QtWidgets.QGraphicsRectItem(
            -self.TRAILER_LEN,
            -self.TRAILER_WIDTH / 2.,
            self.TRAILER_LEN,
            self.TRAILER_WIDTH,
            self)
        self.trailer.setPen(self.LINE_PEN)
        self.trailer.setBrush(self.BODY_COLOR)

        self.trailer_left_wheel = QtWidgets.QGraphicsRectItem(
            self.front_left_wheel.rect(),
            self.trailer)
        self.trailer_left_wheel.setPos(
            -self.TRAILER_LEN,
            -self.TRAILER_WIDTH / 2.)
        self.trailer_left_wheel.setPen(self.LINE_PEN)
        self.trailer_left_wheel.setBrush(self.WHEEL_COLOR)

        self.trailer_right_wheel = QtWidgets.QGraphicsRectItem(
            self.front_left_wheel.rect(),
            self.trailer)
        self.trailer_right_wheel.setPos(
            -self.TRAILER_LEN,
            self.TRAILER_WIDTH / 2.)
        self.trailer_right_wheel.setPen(self.LINE_PEN)
        self.trailer_right_wheel.setBrush(self.WHEEL_COLOR)

        self.circle = QtWidgets.QGraphicsEllipseItem(
            -self.RADIUS,
            -self.RADIUS,
            2 * self.RADIUS,
            2 * self.RADIUS,
            self)
        self.circle.setPen(self.LINE_PEN)

        self.ctrl_point = QtWidgets.QGraphicsEllipseItem(
            -self.CTRL_POINT_RADIUS,
            -self.CTRL_POINT_RADIUS,
            2 * self.CTRL_POINT_RADIUS,
            2 * self.CTRL_POINT_RADIUS,
            self.trailer)
        self.ctrl_point.setPos(
            self.TRAILER_CTRL_POINT[0] - self.TRAILER_LEN,
            self.TRAILER_CTRL_POINT[1])
        self.ctrl_point.setPen(self.CTRL_POINT_PEN)
        self.ctrl_point.setBrush(self.CTRL_POINT_BRUSH)

        # MPC controller
        self.mpc = mpc_casadi.MPCCasadi()

    def move(
            self,
            dt: float,
            line: Tuple[Tuple[float, float], Tuple[float, float]],
            circle_obstacle: Tuple[float, float, float]):
        """Moves the car based on MPC, by updating its position and orientation

        Args:
            dt (float): dt, sec
            line (Tuple[Tuple[float, float], Tuple[float, float]]): line path description:
                A point: [x,y]
                B point: [x,y]
            circle_obstacle (Tuple[float, float, float]): description of circle obstacle: x,y,radius
        """
        try:
            heading = self.rotation()
            state = np.array([
                self.x(),
                self.y(),
                math.radians(heading),
                math.radians(heading + self.trailer.rotation())])

            ctrl_point_pos = self.ctrl_point.pos()
            trailer_len = self.trailer.rect().width()
            solution = self.mpc.optimize_controls(
                line[0],
                line[1],
                state,
                math.radians(self.front_left_wheel.rotation()),
                self.speed,
                self.body.rect().width(),
                self.MAX_RATE,
                self.MAX_ANGLE,
                self.MAX_TRAILER_ANGLE,
                trailer_len,
                self.TRAILER_OFFSET,
                (trailer_len - ctrl_point_pos.x(), ctrl_point_pos.y()),
                self.XTRACK_WEIGHT,
                self.HEADING_WEIGHT,
                circle_obstacle,
                self.circle.rect().width() / 2.)
        except ValueError as e:
            # optimization fails
            LOG.error(e)
            return

        # Update car state
        steering_angle = float(solution[1][1])
        state += mpc_back_box.MPCBlackBox.trailer_model(
            state,
            self.speed,
            steering_angle,
            self.body.rect().width(),
            self.trailer.rect().width(),
            0)[:4] * dt

        self.front_left_wheel.setRotation(math.degrees(steering_angle))
        self.front_right_wheel.setRotation(math.degrees(steering_angle))
        self.setPos(state[0], state[1])
        self.setRotation(math.degrees(state[2]))
        self.trailer.setRotation(math.degrees(state[3] - state[2]))
