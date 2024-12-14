import logging
from typing import Tuple, Callable, Any, List
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc_view.view.scene.items.editable_line import EditableLine
LOG = logging.getLogger(__name__)


class VertexEditor(QtWidgets.QGraphicsRectItem):
    """Helper allow to change something"""
    PEN = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
    PEN.setCosmetic(True)
    WIDTH = 10

    FONT = QtGui.QFont("Times", 16, QtGui.QFont.Bold)
    TEXT_OFFSET = QtCore.QPointF(-10, -30)

    def __init__(self, str_desc:str):
        """Class used fro control line end point
        """
        super().__init__()
        # text adding
        self.text = QtWidgets.QGraphicsSimpleTextItem(str_desc, self)
        self.text.setFont(self.FONT)
        self.text.setFlag(
            QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
        self.text.setPos(self.TEXT_OFFSET)

        self.setPen(self.PEN)

        # list of lines
        self.__callbacks: List[Tuple[Any,
                                     Callable[[Any, QtCore.QPointF], None]]] = []

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

    def bind(self, callback: Tuple[Any, Callable[[Any, QtCore.QPointF], None]]):
        """Add Bind of Editor to some external object
        Args:
            callback (Tuple[Any, Callable[[QtCore.QPointF, Any], None]]): [external data, callback] 
        """
        self.__callbacks.append(callback)

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
            for ext_data, callback in self.__callbacks:
                callback(ext_data, pos)
        return super().itemChange(change, value)
