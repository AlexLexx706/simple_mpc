import math
import logging
from typing import List, Tuple
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc_view.view.scene.items import car
from mpc_view.view.scene.items import line_endpoint
from mpc_view.view.scene.items import editable_line
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

    # default MPC settings
    MPC_STEPS = 10
    MPC_DT = 0.1
    MPC_SOFT_CONSTRAIN = True
    MPC_MAX_ITER = 50

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setSceneRect(-5000, -5000, 10000, 10000)  # Set scene size
        self.points_path = []
        self.circles = []

        # Add a black line to the scene representing the path
        self.add_path([
            (-100, -200),
            (-100, -20),
            (100, -20),
            (200, -40),
            (300, 0)])

        # creating circle
        self.add_circle([100, 50], self.CIRCLE_RADIUS, False)

        # Create car model
        self.car = car.CarModel(
            dt=self.MPC_DT,
            steps=self.MPC_STEPS,
            max_iter=self.MPC_MAX_ITER,
            soft_constrain=self.MPC_SOFT_CONSTRAIN,
            circles_num=len(self.circles),
            path_len=len(self.points_path))
        self.car.setRotation(-10)
        self.addItem(self.car)

        self.predicted_path = QtWidgets.QGraphicsPathItem()
        self.predicted_path.setPen(self.PREDICTED_PATH_PEN)
        self.addItem(self.predicted_path)

    def add_circle(self, pos: Tuple[float, float], radius: float, rebuild_mpc: bool):
        """Add circle to the scene
        Args:
            pos (Tuple[float, float]): position
            radius (float): radius
            rebuild_mpc (bool): true - rebuild mpc, false - not
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

        if rebuild_mpc:
            self.rebuild_mpc()

    def add_path(self, points: List[Tuple[float, float]]):
        """Add editable path to the scene
        Args:
            points (List[Tuple[float, float]]): list of points [[x,y], ...]
        """
        if len(points) < 2:
            raise RuntimeError("len(points) < 2")

        index = 0
        pos = points[0]
        endpoint = line_endpoint.LineEndpoint(index)
        endpoint.setPos(pos[0], pos[1])
        self.addItem(endpoint)
        self.points_path = [endpoint]

        for pos in points[1:]:
            self.add_path_point(pos, False)

    def get_path(self) -> List[Tuple[float, float]]:
        """return path

        Returns:
            List[Tuple[float, float]]: list of points: [x,y]
        """
        path = []
        for point in self.points_path:
            pos = point.pos()
            path.append((pos.x(), pos.y()))
        return np.array(path)

    def get_circles(self) -> List[Tuple[float, float, float]]:
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

    def add_path_point(self, pos: Tuple[float, float], rebuild_mpc: bool):
        """adding new points to path

        Args:
            pos (Tuple[float, float]): new point pos:[x,y]
            rebuild_mpc (bool): true - rebuild car mpc, false - not
        """
        prev_end_point = self.points_path[-1]

        # line adding
        line = editable_line.EditableLine(
            prev_end_point.pos().x(),
            prev_end_point.pos().y(),
            pos[0],
            pos[1])
        self.addItem(line)

        # endpoint adding
        end_point = line_endpoint.LineEndpoint(len(self.points_path))
        self.addItem(end_point)
        end_point.setPos(pos[0], pos[1])
        prev_end_point.bind_line(line, line.move_a_point)
        end_point.bind_line(line, line.move_b_point)

        # adding end-point to points_path
        self.points_path.append(end_point)

        if rebuild_mpc:
            self.rebuild_mpc()

    def remove_selected(self):
        """remove selected objects
        """
        for obj in self.selectedItems():
            # removing circle
            if isinstance(obj, QtWidgets.QGraphicsEllipseItem):
                try:
                    self.circles.remove(obj)
                    self.removeItem(obj)
                    self.rebuild_mpc()
                except ValueError as e:
                    LOG.error(e)
            # remove end-point of line
            elif isinstance(obj, line_endpoint.LineEndpoint):
                self.remove_endpoint(obj)

    def remove_endpoint(self, endpoint: line_endpoint.LineEndpoint) -> None:
        """Removing of endpoint

        Args:
            endpoint (line_endpoint.LineEndpoint): endpoint object
        """
        if len(self.points_path) == 2:
            if self.widget:
                QtWidgets.QMessageBox.warning(
                    self.widget, 'Warning', "Can`t remove last point of line!!!")
            else:
                raise RuntimeError(
                    'Warning', "Can`t remove last point of line!!!")
            return

        # tail of the path
        if len(endpoint.lines) == 1:
            line = endpoint.lines[0][0]
            # finding of other endpoint
            other_endpoint = self.find_other_endpoint(line, endpoint)
            self.remove_line_from_endpoint(other_endpoint, line)
            self.points_path.remove(endpoint)
            self.removeItem(line)
            self.removeItem(endpoint)
        # middle of the path
        else:
            line = endpoint.lines[0][0]
            endpoint_a = self.find_other_endpoint(line, endpoint)
            self.remove_line_from_endpoint(endpoint_a, line)
            self.removeItem(line)

            line = endpoint.lines[1][0]
            endpoint_b = self.find_other_endpoint(line, endpoint)
            self.remove_line_from_endpoint(endpoint_b, line)
            self.removeItem(line)

            self.points_path.remove(endpoint)
            self.removeItem(endpoint)

            # adding new line:
            # line adding
            line = editable_line.EditableLine(
                endpoint_a.pos().x(),
                endpoint_a.pos().y(),
                endpoint_b.pos().x(),
                endpoint_b.pos().y())
            self.addItem(line)
            endpoint_a.bind_line(line, line.move_a_point)
            endpoint_b.bind_line(line, line.move_b_point)

        # need to update text
        for index, endpoint in enumerate(self.points_path):
            endpoint.text.setText(f"{index}")
        self.rebuild_mpc()

    def rebuild_mpc(self):
        """rebuilding of mpc based on new parameters
        """
        self.car.rebuild_mpc(
            dt=self.MPC_DT,
            steps=self.MPC_STEPS,
            max_iter=self.MPC_MAX_ITER,
            soft_constrain=self.MPC_SOFT_CONSTRAIN,
            circles_num=len(self.circles),
            path_len=len(self.points_path))

    @staticmethod
    def find_other_endpoint(line: editable_line.EditableLine, endpoint: line_endpoint.LineEndpoint):
        """finding other endpoint in line

        Args:
            line (editable_line.EditableLine): line
            endpoint (line_endpoint.LineEndpoint): endpoint for exclude

        Returns:
            _type_: other endpoint
        """
        if len(line.end_points) < 2:
            raise RuntimeError("wrong logic: len(line.end_points) < 2")
        for other_endpoint in line.end_points:
            if other_endpoint is not endpoint:
                return other_endpoint

    @staticmethod
    def remove_line_from_endpoint(endpoint: line_endpoint.LineEndpoint, line: editable_line.EditableLine):
        """removeing of line from endpoint
        Args:
            endpoint (line_endpoint.LineEndpoint): endpoint
            line (editable_line.EditableLine): line
        """
        for index, desc in enumerate(endpoint.lines):
            # need to more this line
            if desc[0] is line:
                endpoint.lines.pop(index)
                return
