import logging
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

LOG = logging.getLogger(__name__)


class EditableLine(QtWidgets.QGraphicsLineItem):
    """Line with supporting end points edit via callbacks"""
    PEN = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
    PEN.setCosmetic(True)

    def __init__(self, *args,  **kwargs):
        super().__init__(*args, **kwargs)
        self.setPen(self.PEN)
        # end points of line
        self.end_points = []

    def move_a_point(self, pos: QtCore.QPointF):
        """Move A point of line

        Args:
            pos (QtCore.QPointF): A point of line
        """
        line = self.line()
        line.setP1(pos)
        self.setLine(line)

    def move_b_point(self, pos: QtCore.QPointF):
        """Move B point of line

        Args:
            pos (QtCore.QPointF): A point of line
        """
        line = self.line()
        line.setP2(pos)
        self.setLine(line)
