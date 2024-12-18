import logging
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc import mpc_casadi
import casadi as ca

LOG = logging.getLogger(__name__)


class Circle(QtWidgets.QGraphicsEllipseItem):
    LINE_PEN = QtGui.QPen(QtGui.QColor(0, 0, 0), 2)
    LINE_PEN.setCosmetic(True)
    
    SELECTED_BRUSH = QtGui.QBrush(
        QtGui.QColor(0, 255, 0, 100),
        QtCore.Qt.BrushStyle.SolidPattern)

    NOT_SELECTED_BRUSH = QtGui.QBrush(
        QtGui.QColor(0, 0, 0, 100),
        QtCore.Qt.BrushStyle.NoBrush)


    def __init__(self, *args,  **kwargs):
        super().__init__(*args, **kwargs)
        self.setFlags(
            QtWidgets.QGraphicsRectItem.ItemIsMovable   
            | QtWidgets.QGraphicsRectItem.ItemIsSelectable
            | QtWidgets.QGraphicsRectItem.ItemSendsGeometryChanges)
        self.setPen(self.LINE_PEN)
    
    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsRectItem.ItemPositionChange:
            pos = value
            for item in self.scene().items():
                # checking intersection
                if isinstance(item, QtWidgets.QGraphicsPolygonItem):
                    polygon = item
                    circle_pos = ca.DM([[pos.x(), pos.y()]])
                    circle_radius = self.rect().width() / 2.
                    polygon_pos = polygon.pos()
                    points = list([point.x() + polygon_pos.x(), point.y() + polygon_pos.y()] for point in polygon.polygon()[:-1])
                    path = ca.DM(points)
                    distance = float(mpc_casadi.minimum_distance_between_polygon_and_circle(path, circle_pos, circle_radius))
                    print(distance)
                    if distance < 0:
                        self.setBrush(self.SELECTED_BRUSH)
                    else:
                        self.setBrush(self.NOT_SELECTED_BRUSH)
            
            
        return super().itemChange(change, value)