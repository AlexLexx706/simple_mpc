#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import math
from PyQt5 import QtWidgets, uic, QtCore, QtGui
from mpc_view.view import scene
from mpc_view.view.scene.items import vertex_editor


class View(QtWidgets.QFrame):
    DT = 0.1
    POLYGON_SIZE = [10.0, 10.0]
    POLYGON_PEN = QtGui.QPen(QtGui.QColor(255, 0, 0), 2)
    POLYGON_PEN.setCosmetic(True)
    POLYGON_BRUSH = QtGui.QBrush(
        QtGui.QColor(255, 0, 0, 100),
        QtCore.Qt.BrushStyle.SolidPattern)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(
            os.path.join(os.path.split(__file__)[0], "view.ui"),
            self)
        # self.graphics_view.installEventFilter(self)
        self.graphics_view.wheelEvent = self.wheelEvent_handler
        self.graphics_view.scale(10, 10)

        self.scene = scene.Scene()
        self.graphics_view.setScene(self.scene)

        self.car = self.scene.car
        self.circle = self.scene.circles[0]

        # init values in UI
        prev_signal = self.spinBox_prediction_horizon.blockSignals(True)
        self.spinBox_prediction_horizon.setValue(self.scene.MPC_STEPS)
        self.spinBox_prediction_horizon.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_dt.blockSignals(True)
        self.doubleSpinBox_dt.setValue(self.car.DT)
        self.doubleSpinBox_dt.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_speed.blockSignals(True)
        self.doubleSpinBox_speed.setValue(self.car.speed)
        self.doubleSpinBox_speed.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_wheel_base.blockSignals(True)
        self.doubleSpinBox_wheel_base.setValue(self.car.get_wheel_base())
        self.doubleSpinBox_wheel_base.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_max_rate.blockSignals(True)
        self.doubleSpinBox_max_rate.setValue(math.degrees(self.car.MAX_RATE))
        self.doubleSpinBox_max_rate.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_max_angle.blockSignals(True)
        self.doubleSpinBox_max_angle.setValue(math.degrees(self.car.MAX_ANGLE))
        self.doubleSpinBox_max_angle.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_max_trailer_angle.blockSignals(True)
        self.doubleSpinBox_max_trailer_angle.setValue(
            math.degrees(self.car.MAX_TRAILER_ANGLE))
        self.doubleSpinBox_max_trailer_angle.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_trailer_len.blockSignals(True)
        self.doubleSpinBox_trailer_len.setValue(self.car.get_trailer_len())
        self.doubleSpinBox_trailer_len.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_xtrack_weight.blockSignals(True)
        self.doubleSpinBox_xtrack_weight.setValue(self.car.XTRACK_WEIGHT)
        self.doubleSpinBox_xtrack_weight.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_heading_weight.blockSignals(True)
        self.doubleSpinBox_heading_weight.setValue(self.car.HEADING_WEIGHT)
        self.doubleSpinBox_heading_weight.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_heading.blockSignals(True)
        self.doubleSpinBox_heading.setValue(self.car.rotation())
        self.doubleSpinBox_heading.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_trailer_angle.blockSignals(True)
        self.doubleSpinBox_trailer_angle.setValue(self.car.trailer.rotation())
        self.doubleSpinBox_trailer_angle.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_circle_radius.blockSignals(True)
        self.doubleSpinBox_circle_radius.setValue(
            self.circle.rect().width() / 2.)
        self.doubleSpinBox_circle_radius.blockSignals(prev_signal)

        # Timer to periodically update car movement
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_car)

        self.graphics_view.addAction(self.action_add_point)
        self.graphics_view.addAction(self.action_add_circle)
        self.graphics_view.addAction(self.action_add_polygon)
        separator = QtWidgets.QAction(self)
        separator.setSeparator(True)
        self.graphics_view.addAction(separator)
        self.graphics_view.addAction(self.action_edit_polygon)
        self.graphics_view.addAction(self.action_stop_edit_polygon)
        self.graphics_view.addAction(self.action_remove_selected)

        self.scene.selectionChanged.connect(self.selection_changed)

        self.edited_poligons = []

    def selection_changed(self):
        for item in self.scene.selectedItems():
            print(item)

    @QtCore.pyqtSlot()
    def on_action_add_point_triggered(self):
        pos = self.graphics_view.mapToScene(
            self.graphics_view.mapFromGlobal(QtGui.QCursor.pos()))

        self.scene.add_path_point(
            (pos.x(), pos.y()), True)

    @QtCore.pyqtSlot()
    def on_action_add_circle_triggered(self):
        pos = self.graphics_view.mapToScene(
            self.graphics_view.mapFromGlobal(QtGui.QCursor.pos()))
        self.scene.add_circle(
            (pos.x(), pos.y()),
            self.scene.CIRCLE_RADIUS, True)

    @QtCore.pyqtSlot()
    def on_action_remove_selected_triggered(self):
        self.scene.remove_selected()

    @QtCore.pyqtSlot()
    def on_action_edit_polygon_triggered(self):
        for polygon in self.scene.selectedItems():
            if isinstance(polygon, QtWidgets.QGraphicsPolygonItem):
                # creating of the Set of vertex editors for this polygon
                if not hasattr(polygon, "control_points"):
                    control_points = []
                    polygon.control_points = control_points
                    for index, point in enumerate(polygon.polygon()):
                        ve = vertex_editor.VertexEditor(index)
                        ve.setParentItem(polygon)
                        ve.setPos(point)
                        ve.bind(((polygon, index), self.poligon_changed))
                        control_points.append(ve)

    @QtCore.pyqtSlot()
    def on_action_stop_edit_polygon_triggered(self):
        for polygon in self.scene.selectedItems():
            if isinstance(polygon, QtWidgets.QGraphicsPolygonItem):
                if hasattr(polygon, "control_points"):
                    for item in polygon.control_points:
                        item.setParentItem(None)
                    delattr(polygon, "control_points")

    def poligon_changed(self, desc, pos):
        polygon_item, index = desc
        polygon = polygon_item.polygon()
        polygon[index] = pos
        polygon_item.setPolygon(polygon)

    @QtCore.pyqtSlot()
    def on_action_add_polygon_triggered(self):
        pos = self.graphics_view.mapToScene(
            self.graphics_view.mapFromGlobal(QtGui.QCursor.pos()))
        polygon = QtWidgets.QGraphicsPolygonItem(QtGui.QPolygonF(
            QtCore.QRectF(
                0,
                0,
                self.POLYGON_SIZE[0],
                self.POLYGON_SIZE[1])))
        polygon.setPos(pos)
        polygon.setPen(self.POLYGON_PEN)
        polygon.setBrush(self.POLYGON_BRUSH)
        polygon.setFlags(
            QtWidgets.QGraphicsItem.ItemIsMovable |
            QtWidgets.QGraphicsItem.ItemIsSelectable |
            QtWidgets.QGraphicsItem.ItemSendsGeometryChanges)
        self.scene.addItem(polygon)
        polygon.setZValue(0)

    def wheelEvent_handler(self, event):
        # Zoom in and out with the mouse wheel
        factor = 1.1
        if event.angleDelta().y() < 0:
            factor = 1 / factor
        self.graphics_view.scale(factor, factor)

    def update_car(self):
        # Move the car based on optimized steering angles
        self.scene.update_car()
        prev_signal = self.doubleSpinBox_heading.blockSignals(True)
        self.doubleSpinBox_heading.setValue(self.car.rotation())
        self.doubleSpinBox_heading.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_trailer_angle.blockSignals(True)
        self.doubleSpinBox_trailer_angle.setValue(self.car.trailer.rotation())
        self.doubleSpinBox_trailer_angle.blockSignals(prev_signal)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_heading_valueChanged(self, value):
        self.car.setRotation(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_trailer_angle_valueChanged(self, value):
        self.car.trailer.setRotation(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_circle_radius_valueChanged(self, value):
        self.circle.setRect(-value, -value, 2 * value, 2 * value)

    @QtCore.pyqtSlot(bool)
    def on_pushButton_start_clicked(self, _):
        if not self.timer.isActive():
            self.timer.start(int(1000 * self.DT))  # 10 FPS
            self.pushButton_start.setText("Stop")
        else:
            self.timer.stop()
            self.pushButton_start.setText("Start")

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_speed_valueChanged(self, value):
        self.car.speed = value

    @QtCore.pyqtSlot(int)
    def on_spinBox_prediction_horizon_valueChanged(self, value):
        self.scene.MPC_STEPS = value
        self.scene.rebuild_mpc()

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_wheel_base_valueChanged(self, value):
        self.car.set_wheel_base(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_max_rate_valueChanged(self, value):
        self.car.MAX_RATE = math.radians(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_max_angle_valueChanged(self, value):
        self.car.MAX_ANGLE = math.radians(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_max_trailer_angle_valueChanged(self, value):
        self.car.MAX_TRAILER_ANGLE = value

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_trailer_len_valueChanged(self, value):
        self.car.set_trailer_len(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_xtrack_weight_valueChanged(self, value):
        self.car.XTRACK_WEIGHT = value

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_heading_weight_valueChanged(self, value):
        self.car.HEADING_WEIGHT = value

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_dt_valueChanged(self, value):
        self.car.DT = value
