#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import math
from PyQt5 import QtWidgets, uic, QtCore
from mpc_view import scene


class MainWidget(QtWidgets.QMainWindow):
    DT = 0.1

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(
            os.path.join(os.path.split(__file__)[0], "main_widget.ui"),
            self)
        self.scene = scene.Scene()
        self.graphics_view.setScene(self.scene)

        self.car = self.scene.car
        self.circle = self.scene.circle

        # init values in UI
        prev_signal = self.doubleSpinBox_speed.blockSignals(True)
        self.doubleSpinBox_speed.setValue(self.car.speed)
        self.doubleSpinBox_speed.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_heading.blockSignals(True)
        self.doubleSpinBox_heading.setValue(self.car.rotation())
        self.doubleSpinBox_heading.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_trailer_angle.blockSignals(True)
        self.doubleSpinBox_trailer_angle.setValue(self.car.trailer.rotation())
        self.doubleSpinBox_trailer_angle.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_circle_x.blockSignals(True)
        self.doubleSpinBox_circle_x.setValue(self.circle.pos().x())
        self.doubleSpinBox_circle_x.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_circle_y.blockSignals(True)
        self.doubleSpinBox_circle_y.setValue(self.circle.pos().y())
        self.doubleSpinBox_circle_y.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_circle_radius.blockSignals(True)
        self.doubleSpinBox_circle_radius.setValue(
            self.circle.rect().width() / 2.)
        self.doubleSpinBox_circle_radius.blockSignals(prev_signal)

        # Timer to periodically update car movement
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_car)

    def update_car(self):
        # Move the car based on optimized steering angles
        self.scene.update_car()
        prev_signal = self.doubleSpinBox_heading.blockSignals(True)
        self.doubleSpinBox_heading.setValue(
            math.degrees(self.car.rotation()))
        self.doubleSpinBox_heading.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_trailer_angle.blockSignals(True)
        self.doubleSpinBox_trailer_angle.setValue(
            math.degrees(self.car.trailer.rotation()))
        self.doubleSpinBox_trailer_angle.blockSignals(prev_signal)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_heading_valueChanged(self, value):
        self.car.setRotation(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_trailer_angle_valueChanged(self, value):
        self.car.trailer.setRotation(value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_speed_valueChanged(self, value):
        self.car.speed = value

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_circle_x_valueChanged(self, value):
        pos = self.circle.pos()
        self.circle.setPos(value, pos.y())

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_circle_y_valueChanged(self, value):
        pos = self.circle.pos()
        self.circle.setPos(pos.x(), value)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_circle_radius_valueChanged(self, value):
        self.circle.setRect(-value, -value, 2 * value, 2 * value)

    @QtCore.pyqtSlot(bool)
    def on_pushButton_start_clicked(self, _):
        if not self.timer.isActive():
            self.timer.start(int(1000 * self.DT))  # 10 FPS
        else:
            self.timer.stop()
