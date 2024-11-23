import logging
from typing import Tuple, Callable
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc_view.scene.items.editable_line import EditableLine
LOG = logging.getLogger(__name__)


class LineEndpoint(QtWidgets.QGraphicsRectItem):
    """Visualization of line endpoint"""
    PEN = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
    PEN.setCosmetic(True)
    WIDTH = 10

    FONT = QtGui.QFont("Times", 16, QtGui.QFont.Bold)
    TEXT_OFFSET = QtCore.QPointF(-10, -30)

    def __init__(self, index):
        """CLass used fro control line end point
        """
        super().__init__()
        # text adding
        self.text = QtWidgets.QGraphicsSimpleTextItem(f'{index}', self)
        self.text.setFont(self.FONT)
        self.text.setFlag(
            QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
        self.text.setPos(self.TEXT_OFFSET)

        self.setPen(self.PEN)

        # list of lines
        self.lines: Tuple[EditableLine, Callable[[QtCore.QPointF], None]] = []

        self.setFlags(
            self.ItemIsMovable
            | self.ItemIsSelectable
            | self.ItemSendsGeometryChanges
            | self.ItemIgnoresTransformations)
        self.setRect(
            -self.WIDTH / 2,
            -self.WIDTH / 2,
            self.WIDTH,
            self.WIDTH)

    def bind_line(self, line: EditableLine, callback: Callable[[QtCore.QPointF], None]):
        """Bind line to endpoint

        Args:
            line (EditableLine): line object
            callback (Callable[[QtCore.QPointF],None]): callback - used for sending position of end point
        """
        self.lines.append((line, callback))
        line.end_points.append(self)

    def itemChange(self, change, value):
        """callback of changes

        Args:
            change (_type_): type of changes
            value (_type_): value

        Returns:
            _type_: result
        """
        if change == QtWidgets.QGraphicsRectItem.ItemPositionChange:
            pos = value

            for _, callback in self.lines:
                callback(pos)
        return super().itemChange(change, value)
