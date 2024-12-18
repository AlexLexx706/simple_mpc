import logging
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from mpc import mpc_casadi
import casadi as ca

LOG = logging.getLogger(__name__)

class Polygon(QtWidgets.QGraphicsPolygonItem):
    PEN = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
    PEN.setCosmetic(True)

    BRUSH = QtGui.QBrush(
        QtGui.QColor(255, 0, 0, 100),
        QtCore.Qt.BrushStyle.SolidPattern)

    SELECTED_BRUSH = QtGui.QBrush(
        QtGui.QColor(0, 255, 0, 100),
        QtCore.Qt.BrushStyle.SolidPattern)

    def __init__(self, *args,  **kwargs):
        super().__init__(*args, **kwargs)
        self.setFlags(
            QtWidgets.QGraphicsRectItem.ItemIsMovable   
            | QtWidgets.QGraphicsRectItem.ItemIsSelectable
            | QtWidgets.QGraphicsRectItem.ItemSendsGeometryChanges)
        self.setPen(self.PEN)
        self.setBrush(self.BRUSH)
        
    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsRectItem.ItemPositionChange:
            pos = value
            for item in self.scene().items():
                # checking intersection
                if isinstance(item, QtWidgets.QGraphicsPolygonItem) and item is not self:
                    polygon = item
                    polygon_pos = polygon.pos()
                    path_1 = ca.DM([[point.x() + polygon_pos.x(), point.y() + polygon_pos.y()]\
                        for point in polygon.polygon()[:-1]])
                    path_2 = ca.DM([[point.x() + pos.x(), point.y() + pos.y()]\
                        for point in self.polygon()[:-1]])

                    distance = float(mpc_casadi.minimum_distance_between_polygons(
                        path_1, path_2))
                    print(distance)
                    if distance < 0:
                        self.setBrush(self.SELECTED_BRUSH)
                    else:
                        self.setBrush(self.BRUSH)
            
            
        return super().itemChange(change, value)