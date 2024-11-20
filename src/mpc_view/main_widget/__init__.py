#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import math
from PyQt5 import QtWidgets, uic, QtCore


class MainWidget(QtWidgets.QMainWindow):
    DT = 0.1

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(
            os.path.join(os.path.split(__file__)[0], "main_widget.ui"),
            self)

        self.car = self.graphics_view.scene().car
        self.ellipse = self.graphics_view.scene().ellipse

        # init values in UI
        prev_signal = self.doubleSpinBox_speed.blockSignals(True)
        self.doubleSpinBox_speed.setValue(self.car.speed)
        self.doubleSpinBox_speed.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_heading.blockSignals(True)
        self.doubleSpinBox_heading.setValue(
            math.degrees(self.car.get_heading()))
        self.doubleSpinBox_heading.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_trailer_angle.blockSignals(True)
        self.doubleSpinBox_trailer_angle.setValue(
            math.degrees(self.car.get_trailer_angle()))
        self.doubleSpinBox_trailer_angle.blockSignals(prev_signal)

        # Timer to periodically update car movement
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateCar)

    def updateCar(self):
        # Move the car based on optimized steering angles
        self.car.move(self.DT)
        prev_signal = self.doubleSpinBox_heading.blockSignals(True)
        self.doubleSpinBox_heading.setValue(
            math.degrees(self.car.get_heading()))
        self.doubleSpinBox_heading.blockSignals(prev_signal)

        prev_signal = self.doubleSpinBox_trailer_angle.blockSignals(True)
        self.doubleSpinBox_trailer_angle.setValue(
            math.degrees(self.car.get_trailer_angle()))
        self.doubleSpinBox_trailer_angle.blockSignals(prev_signal)

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_heading_valueChanged(self, value):
        self.car.set_heading(math.radians(value))

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_trailer_angle_valueChanged(self, value):
        self.car.set_trailer_angle(math.radians(value))

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_speed_valueChanged(self, value):
        self.car.speed = value

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_circle_x_valueChanged(self, value):
        pos = self.ellipse.pos()
        pos.setX(value)
        self.ellipse.setPos(pos)
        self.car.circle_1 = [pos.x(), pos.y(), self.car.circle_1[2]]

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_circle_y_valueChanged(self, value):
        pos = self.ellipse.pos()
        pos.setY(value)
        self.ellipse.setPos(pos)
        self.car.circle_1 = [pos.x(), pos.y(), self.car.circle_1[2]]

    @QtCore.pyqtSlot(float)
    def on_doubleSpinBox_circle_radius_valueChanged(self, value):
        self.ellipse.setRect(-value, -value, 2*value, 2*value)
        self.car.circle_1 = [self.car.circle_1[0], self.car.circle_1[1], value]

    @QtCore.pyqtSlot(bool)
    def on_pushButton_start_clicked(self, checked):
        if not self.timer.isActive():
            self.timer.start(100)  # 10 FPS
        else:
            self.timer.stop()
