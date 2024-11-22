import logging
from typing import List, Callable
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

LOG = logging.getLogger(__name__)


class LineEndpoint(QtWidgets.QGraphicsRectItem):
    """Visualization of line endpoint"""
    PEN = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
    PEN.setCosmetic(True)
    WIDTH = 10

    def __init__(self, *args, **kwargs):
        """CLass used fro control line end point
        """
        super().__init__(*args, **kwargs)
        self.setPen(self.PEN)
        self.callbacks = []
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

            for call in self.callbacks:
                call(pos)
        return super().itemChange(change, value)
