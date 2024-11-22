import math
import logging
from typing import List, Tuple
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc_view.scene.items import car
from mpc_view.scene.items import line_endpoint
from mpc_view.scene.items import editable_line
import numpy as np
from mpc.mpc_casadi import MPCCasadi
import casadi as ca


LOG = logging.getLogger(__name__)


class Scene(QtWidgets.QGraphicsScene):
    LINE_PEN = QtGui.QPen(QtGui.QColor(0, 0, 0), 2)
    LINE_PEN.setCosmetic(True)

    GREED_PEN = QtGui.QPen(QtGui.QColor(230, 230, 230), 2)
    GREED_PEN.setCosmetic(True)
    GRID_SIZE = 10  # Grid size for background grid
    DT = 1 / 10.  # update car period, sec
    CIRCLE_RADIUS = 50.

    PREDICTED_PATH_PEN = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
    PREDICTED_PATH_PEN.setCosmetic(True)
    FONT = QtGui.QFont("Times", 16, QtGui.QFont.Bold);
    TEXT_OFFSET = QtCore.QPointF(-10, -30)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setSceneRect(-5000, -5000, 10000, 10000)  # Set scene size
        self.points_path = []
        self.circles = []

        # Add a black line to the scene representing the path
        self.add_path([(-100, -200), (-100, -20), (100, -20), (200, -40), (300, 0)])

        # creating circle
        self.add_circle([100, 50], self.CIRCLE_RADIUS)

        # Create car model
        self.car = car.CarModel(
            dt=0.3,
            steps=10,
            max_iter=5,
            soft_constrain=True,
            circles_num=len(self.circles),
            path_len=len(self.points_path))
        self.car.setRotation(-10)
        self.addItem(self.car)

        self.predicted_path = QtWidgets.QGraphicsPathItem()
        self.predicted_path.setPen(self.PREDICTED_PATH_PEN)
        self.addItem(self.predicted_path)

    def add_circle(self, pos:Tuple[float, float], radius:float):
        """Add circle to the scene
        Args:
            pos (Tuple[float, float]): position
            radius (float): radius
        """
        circle = self.addEllipse(
            QtCore.QRectF(
                -radius,
                -radius,
                radius * 2,
                radius * 2),
            self.LINE_PEN)
        circle.setPos(pos[0], pos[1])
        circle.setFlags(
            QtWidgets.QGraphicsRectItem.ItemIsMovable
            | QtWidgets.QGraphicsRectItem.ItemIsSelectable
            | QtWidgets.QGraphicsRectItem.ItemSendsGeometryChanges)
        self.circles.append(circle)


    def add_path(self, points: List[Tuple[float, float]]):
        """Add editable path to the scene
        Args:
            points (List[Tuple[float, float]]): list of points [[x,y], ...]
        """
        if len(points) < 2:
            raise RuntimeError("len(points) < 2")

        index = 0
        prev_pos = points[0]
        prev_end_point = line_endpoint.LineEndpoint()
        prev_end_point.setPos(prev_pos[0], prev_pos[1])
        self.addItem(prev_end_point)

        text = QtWidgets.QGraphicsSimpleTextItem(f'{index}', prev_end_point)
        index += 1
        text.setFont(self.FONT)
        text.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
        text.setPos(self.TEXT_OFFSET)

        self.points_path = [prev_end_point]

        for pos in points[1:]:
            line = editable_line.EditableLine(
                prev_pos[0],
                prev_pos[1],
                pos[0],
                pos[1])
            self.addItem(line)

            cur_end_point = line_endpoint.LineEndpoint()
            self.addItem(cur_end_point)
            self.points_path.append(cur_end_point)
            cur_end_point.setPos(pos[0], pos[1])
            prev_end_point.callbacks.append(line.move_a_point)
            cur_end_point.callbacks.append(line.move_b_point)

            text = QtWidgets.QGraphicsSimpleTextItem(f'{index}', cur_end_point)
            index += 1
            text.setFont(self.FONT)
            text.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
            text.setPos(self.TEXT_OFFSET)

            prev_pos = pos
            prev_end_point = cur_end_point

    def get_path(self)->List[Tuple[float, float]]:
        """return path

        Returns:
            List[Tuple[float, float]]: list of points: [x,y]
        """
        path = []
        for point in self.points_path:
            pos = point.pos()
            path.append((pos.x(), pos.y()))
        return np.array(path)

    def get_circles(self)->List[Tuple[float, float, float]]:
        """return circles descriptions

        Returns:
            List[Tuple[float, float, float]]: list of circles descriptions: [x,y,radius]
        """
        circles_state = []
        for circle in self.circles:
            circle_pos = circle.pos()
            circles_state.append((
                circle_pos.x(),
                circle_pos.y(),
                circle.rect().width() / 2.))
        return np.array(circles_state)

    def update_car(self):
        # MPC drive car
        solution = self.car.move(
            self.DT,
            self.get_path(),
            self.get_circles())

        # update prediction path
        if solution:
            predicted_path = QtGui.QPainterPath()
            states = solution[0]
            predicted_path.moveTo(
                states[0, 0],
                states[0, 1])
            for index in range(states.shape[0]):
                predicted_path.lineTo(
                    states[index, 0],
                    states[index, 1])
            self.predicted_path.setPath(predicted_path)

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

    def mousePressEvent(self, event):
        pos = event.scenePos()
        pos = ca.DM([pos.x(), pos.y()]).T
        path = ca.DM(self.get_path())

        xt, he = MPCCasadi.get_track_cost(
            path=path,
            control_point=pos,
            heading=0)
        print(xt, he)
        return super().mousePressEvent(event)
