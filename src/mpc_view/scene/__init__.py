import math
import logging
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc_view.scene.items import car

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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setSceneRect(-5000, -5000, 10000, 10000)  # Set scene size
        self.car = car.CarModel()  # Create car model
        self.addItem(self.car)  # Add car to the scene
        self.car.setFlags(
            QtWidgets.QGraphicsRectItem.ItemIsMovable
            | QtWidgets.QGraphicsRectItem.ItemIsSelectable
            | QtWidgets.QGraphicsRectItem.ItemSendsGeometryChanges)
        # Add a black line to the scene representing the path
        self.line = QtWidgets.QGraphicsLineItem(-2000, -100, 2000, -100)

        # Set line color and width
        self.line.setPen(self.LINE_PEN)
        self.addItem(self.line)
        self.line.setFlags(
            QtWidgets.QGraphicsRectItem.ItemIsMovable
            | QtWidgets.QGraphicsRectItem.ItemIsSelectable
            | QtWidgets.QGraphicsRectItem.ItemSendsGeometryChanges)

        self.circle = self.addEllipse(
            QtCore.QRectF(
                -self.CIRCLE_RADIUS,
                -self.CIRCLE_RADIUS,
                self.CIRCLE_RADIUS * 2,
                self.CIRCLE_RADIUS * 2),
            self.LINE_PEN)
        self.circle.setPos(100, 0)
        self.circle.setFlags(
            QtWidgets.QGraphicsRectItem.ItemIsMovable
            | QtWidgets.QGraphicsRectItem.ItemIsSelectable
            | QtWidgets.QGraphicsRectItem.ItemSendsGeometryChanges)

        self.predicted_path = QtWidgets.QGraphicsPathItem()
        self.predicted_path.setPen(self.PREDICTED_PATH_PEN)
        self.addItem(self.predicted_path)

    def update_car(self):
        line = self.line.line()
        line_pos = self.line.pos()
        line = ((
            line.x1() + line_pos.x(),
            line.y1() + line_pos.y()), (
            line.x2() + line_pos.x(),
            line.y2() + line_pos.y()))
        circle_pos = self.circle.pos()
        circle = (circle_pos.x(), circle_pos.y(),
                  self.circle.rect().width() / 2.)
        # MPC drive car
        solution = self.car.move(
            self.DT,
            line,
            circle)

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
