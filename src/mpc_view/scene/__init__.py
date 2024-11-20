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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setSceneRect(-5000, -5000, 10000, 10000)  # Set scene size

        self.line = ((-2000, -100), (2000, 50))

        self.car = car.CarModel()  # Create car model
        self.car.set_line(self.line)
        self.addItem(self.car)  # Add car to the scene

        # Add a black line to the scene representing the path
        self.black_line = QtWidgets.QGraphicsLineItem(
            self.line[0][0],
            self.line[0][1],
            self.line[1][0],
            self.line[1][1])

        # Set line color and width
        self.black_line.setPen(self.LINE_PEN)
        self.addItem(self.black_line)

        self.ellipse = self.addEllipse(
            QtCore.QRectF(10, 10, 20, 20),
            self.LINE_PEN)
        self.ellipse.setPos(0, 0)

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
