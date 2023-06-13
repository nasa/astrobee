#!/usr/bin/env python
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import argparse
import atexit
import functools
import os
import re
import select
import signal
import socket
import subprocess
import sys
import time
from math import asin, atan2, isnan, pi, sqrt

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

filepath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filepath + "/communications")

import com_manager as com
import plot_types

ARRAY_SIZE = plot_types.DISPLAY_TIME * 65

ASTROBEE_ROOT = os.path.dirname(os.path.realpath(__file__)) + "/../../../"

# Creating communication manager with default value
com_manager = com.ComManager()


def quat_to_eulers(quat):
    return [
        atan2(
            2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2 * (quat.x**2 + quat.y**2)
        ),
        asin(2 * (quat.w * quat.y - quat.z * quat.x)),
        atan2(
            2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y**2 + quat.z**2)
        ),
    ]


def mean(array):
    return sum(array) / len(array)


start_time = time.time()


class VisualizerCallback:
    def __init__(self, time_key, callbacks):
        self.time_key = time_key
        self.callbacks = callbacks


ekf_data = {
    "ekf_time": lambda x: time.time() - start_time,
    "ekf_position_x": lambda x: x.pose.position.x,
    "ekf_position_y": lambda x: x.pose.position.y,
    "ekf_position_z": lambda x: x.pose.position.z,
    "ekf_vel_x": lambda x: x.velocity.x,
    "ekf_vel_y": lambda x: x.velocity.y,
    "ekf_vel_z": lambda x: x.velocity.z,
    "ekf_accel_x": lambda x: x.accel.x,
    "ekf_accel_y": lambda x: x.accel.y,
    "ekf_accel_z": lambda x: x.accel.z,
    "ekf_accel_bias_x": lambda x: x.accel_bias.x,
    "ekf_accel_bias_y": lambda x: x.accel_bias.y,
    "ekf_accel_bias_z": lambda x: x.accel_bias.z,
    "ekf_rot_x": lambda x: quat_to_eulers(x.pose.orientation)[0] * 180 / pi,
    "ekf_rot_y": lambda x: quat_to_eulers(x.pose.orientation)[1] * 180 / pi,
    "ekf_rot_z": lambda x: quat_to_eulers(x.pose.orientation)[2] * 180 / pi,
    "ekf_omega_x": lambda x: x.omega.x,
    "ekf_omega_y": lambda x: x.omega.y,
    "ekf_omega_z": lambda x: x.omega.z,
    "ekf_gyro_bias_x": lambda x: x.gyro_bias.x,
    "ekf_gyro_bias_y": lambda x: x.gyro_bias.y,
    "ekf_gyro_bias_z": lambda x: x.gyro_bias.z,
    "ekf_confidence": lambda x: x.confidence,
    "ekf_cov_rot_x": lambda x: x.cov_diag[0],
    "ekf_cov_rot_y": lambda x: x.cov_diag[1],
    "ekf_cov_rot_z": lambda x: x.cov_diag[2],
    "ekf_cov_rot_m": lambda x: sqrt(
        x.cov_diag[0] ** 2 + x.cov_diag[1] ** 2 + x.cov_diag[2] ** 2
    ),
    "ekf_cov_gbias_x": lambda x: x.cov_diag[3],
    "ekf_cov_gbias_y": lambda x: x.cov_diag[4],
    "ekf_cov_gbias_z": lambda x: x.cov_diag[5],
    "ekf_cov_gbias_m": lambda x: sqrt(
        x.cov_diag[3] ** 2 + x.cov_diag[4] ** 2 + x.cov_diag[5] ** 2
    ),
    "ekf_cov_vel_x": lambda x: x.cov_diag[6],
    "ekf_cov_vel_y": lambda x: x.cov_diag[7],
    "ekf_cov_vel_z": lambda x: x.cov_diag[8],
    "ekf_cov_vel_m": lambda x: sqrt(
        x.cov_diag[6] ** 2 + x.cov_diag[7] ** 2 + x.cov_diag[8] ** 2
    ),
    "ekf_cov_abias_x": lambda x: x.cov_diag[9],
    "ekf_cov_abias_y": lambda x: x.cov_diag[10],
    "ekf_cov_abias_z": lambda x: x.cov_diag[11],
    "ekf_cov_abias_m": lambda x: sqrt(
        x.cov_diag[9] ** 2 + x.cov_diag[10] ** 2 + x.cov_diag[11] ** 2
    ),
    "ekf_cov_pos_x": lambda x: x.cov_diag[12],
    "ekf_cov_pos_y": lambda x: x.cov_diag[13],
    "ekf_cov_pos_z": lambda x: x.cov_diag[14],
    "ekf_cov_pos_m": lambda x: sqrt(
        x.cov_diag[12] ** 2 + x.cov_diag[13] ** 2 + x.cov_diag[14] ** 2
    ),
}
ekf_callbacks = VisualizerCallback("ekf_time", ekf_data)


def mahal_filter(dists):
    f = filter(lambda t: not isnan(t), dists)
    if len(f) == 0:
        return [-1]
    return f


ml_data = {
    "ml_time": lambda x: time.time() - start_time,
    "ml_landmarks": lambda x: x.ml_count,
    "ml_mahal_min": lambda x: min(mahal_filter(x.ml_mahal_dists)),
    "ml_mahal_mean": lambda x: mean(mahal_filter(x.ml_mahal_dists)),
    "ml_mahal_max": lambda x: max(mahal_filter(x.ml_mahal_dists)),
}
ml_callbacks = VisualizerCallback("ml_time", ml_data)

of_data = {
    "of_time": lambda x: time.time() - start_time,
    "of_landmarks": lambda x: x.of_count,
}
of_callbacks = VisualizerCallback("of_time", of_data)

truth_data = {
    "truth_time": lambda x: time.time() - start_time,
    "truth_position_x": lambda x: x.pose.position.x,
    "truth_position_y": lambda x: x.pose.position.y,
    "truth_position_z": lambda x: x.pose.position.z,
    "truth_rot_x": lambda x: quat_to_eulers(x.pose.orientation)[0] * 180 / pi,
    "truth_rot_y": lambda x: quat_to_eulers(x.pose.orientation)[1] * 180 / pi,
    "truth_rot_z": lambda x: quat_to_eulers(x.pose.orientation)[2] * 180 / pi,
}
truth_callbacks = VisualizerCallback("truth_time", truth_data)

ml_pose_data = {
    "ml_pose_time": lambda x: time.time() - start_time,
    "ml_pose_position_x": lambda x: x.pose.position.x,
    "ml_pose_position_y": lambda x: x.pose.position.y,
    "ml_pose_position_z": lambda x: x.pose.position.z,
    "ml_pose_rot_x": lambda x: quat_to_eulers(x.pose.orientation)[0] * 180 / pi,
    "ml_pose_rot_y": lambda x: quat_to_eulers(x.pose.orientation)[1] * 180 / pi,
    "ml_pose_rot_z": lambda x: quat_to_eulers(x.pose.orientation)[2] * 180 / pi,
}
ml_pose_callbacks = VisualizerCallback("ml_pose_time", ml_pose_data)

command_data = {
    "command_time": lambda x: time.time() - start_time,
    "command_status": lambda x: x.status,
    "command_control_mode": lambda x: x.control_mode,
    "command_force_x": lambda x: x.wrench.force.x,
    "command_force_y": lambda x: x.wrench.force.y,
    "command_force_z": lambda x: x.wrench.force.z,
    "command_torque_x": lambda x: x.wrench.torque.x,
    "command_torque_y": lambda x: x.wrench.torque.y,
    "command_torque_z": lambda x: x.wrench.torque.z,
    "command_pos_err_x": lambda x: x.position_error.x,
    "command_pos_err_y": lambda x: x.position_error.y,
    "command_pos_err_z": lambda x: x.position_error.z,
    "command_pos_err_int_x": lambda x: x.position_error_integrated.x,
    "command_pos_err_int_y": lambda x: x.position_error_integrated.y,
    "command_pos_err_int_z": lambda x: x.position_error_integrated.z,
    "command_att_err_x": lambda x: x.attitude_error.x,
    "command_att_err_y": lambda x: x.attitude_error.y,
    "command_att_err_z": lambda x: x.attitude_error.z,
    "command_att_err_int_x": lambda x: x.attitude_error_integrated.x,
    "command_att_err_int_y": lambda x: x.attitude_error_integrated.y,
    "command_att_err_int_z": lambda x: x.attitude_error_integrated.z,
}
command_callbacks = VisualizerCallback("command_time", command_data)

traj_data = {
    "traj_time": lambda x: time.time() - start_time,
    "traj_position_x": lambda x: x.pose.position.x,
    "traj_position_y": lambda x: x.pose.position.y,
    "traj_position_z": lambda x: x.pose.position.z,
    "traj_rot_x": lambda x: quat_to_eulers(x.pose.orientation)[0] * 180 / pi,
    "traj_rot_y": lambda x: quat_to_eulers(x.pose.orientation)[1] * 180 / pi,
    "traj_rot_z": lambda x: quat_to_eulers(x.pose.orientation)[2] * 180 / pi,
}
traj_callbacks = VisualizerCallback("traj_time", traj_data)

shaper_data = {
    "shaper_time": lambda x: time.time() - start_time,
    "shaper_position_x": lambda x: x.pose.position.x,
    "shaper_position_y": lambda x: x.pose.position.y,
    "shaper_position_z": lambda x: x.pose.position.z,
    "shaper_rot_x": lambda x: quat_to_eulers(x.pose.orientation)[0] * 180 / pi,
    "shaper_rot_y": lambda x: quat_to_eulers(x.pose.orientation)[1] * 180 / pi,
    "shaper_rot_z": lambda x: quat_to_eulers(x.pose.orientation)[2] * 180 / pi,
}
shaper_callbacks = VisualizerCallback("shaper_time", shaper_data)

pmc_data = {
    "pmc_time": lambda x: time.time() - start_time,
    "pmc_1_motor_speed": lambda x: x.goals[0].motor_speed,
    "pmc_1_nozzle_1": lambda x: ord(x.goals[0].nozzle_positions[0]),
    "pmc_1_nozzle_2": lambda x: ord(x.goals[0].nozzle_positions[1]),
    "pmc_1_nozzle_3": lambda x: ord(x.goals[0].nozzle_positions[2]),
    "pmc_1_nozzle_4": lambda x: ord(x.goals[0].nozzle_positions[3]),
    "pmc_1_nozzle_5": lambda x: ord(x.goals[0].nozzle_positions[4]),
    "pmc_1_nozzle_6": lambda x: ord(x.goals[0].nozzle_positions[5]),
    "pmc_2_motor_speed": lambda x: x.goals[1].motor_speed,
    "pmc_2_nozzle_1": lambda x: ord(x.goals[1].nozzle_positions[0]),
    "pmc_2_nozzle_2": lambda x: ord(x.goals[1].nozzle_positions[1]),
    "pmc_2_nozzle_3": lambda x: ord(x.goals[1].nozzle_positions[2]),
    "pmc_2_nozzle_4": lambda x: ord(x.goals[1].nozzle_positions[3]),
    "pmc_2_nozzle_5": lambda x: ord(x.goals[1].nozzle_positions[4]),
    "pmc_2_nozzle_6": lambda x: ord(x.goals[1].nozzle_positions[5]),
}
pmc_callbacks = VisualizerCallback("pmc_time", pmc_data)

callbacks_list = [
    ekf_callbacks,
    ml_callbacks,
    of_callbacks,
    truth_callbacks,
    ml_pose_callbacks,
    command_callbacks,
    traj_callbacks,
    shaper_callbacks,
    pmc_callbacks,
]


class TerminalView(QtGui.QGraphicsTextItem):
    def __init__(self, graphics_view):
        super(TerminalView, self).__init__("")
        self.graphics_view = graphics_view
        self.setZValue(1)
        self.setPos(10, 10)
        self.setDefaultTextColor(QtGui.QColor(255, 255, 255))

    def boundingRect(self):
        r = super(TerminalView, self).boundingRect()
        r.setWidth(self.graphics_view.width() - 50)
        r.setHeight(max(self.graphics_view.height() - 40, r.height()))
        return r

    def paint(self, painter, o, w):
        painter.setBrush(QtGui.QColor(0, 0, 0, 150))
        painter.drawRect(self.boundingRect())
        super(TerminalView, self).paint(painter, o, w)


class TerminalGraphicsView(QtGui.QGraphicsView):
    def __init__(self, parent):
        super(TerminalGraphicsView, self).__init__()
        self.connect(parent, QtCore.SIGNAL("resize"), self.resize)
        self.setScene(QtGui.QGraphicsScene())
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.viewport().setAutoFillBackground(False)
        self.setStyleSheet("border: none")
        self.setInteractive(False)

    def resize(self, event):
        self.move(20, 20)
        self.setFixedSize(event.size().width() - 40, event.size().height() - 40)


class ParentGraphicsView(pg.GraphicsView):
    def __init__(self):
        super(ParentGraphicsView, self).__init__()
        self.setAntialiasing(False)

    def resizeEvent(self, event):
        super(ParentGraphicsView, self).resizeEvent(event)
        if event != None:
            self.emit(QtCore.SIGNAL("resize"), event)


class Visualizer(QtGui.QMainWindow):
    def __init__(self, launch_command=None, plan=None, com_method=com.DDS_COM):
        super(Visualizer, self).__init__()
        self.com_method = com_method
        self.launch_command = launch_command
        self.plan = plan

        self.max_log_length = 500
        self.log_updated = False
        self.log_lines = 0
        self.log_text = ""
        self.data = dict()
        self.data_sizes = dict()
        for d in callbacks_list:
            self.data_sizes[d.time_key] = 0
            for k in d.callbacks.keys():
                self.data[k] = np.full(ARRAY_SIZE, 1e-10)

        self.columns = [
            [
                plot_types.CtlPosPlot,
                plot_types.PositionPlot,
                plot_types.FeatureCountPlot,
                plot_types.ConfidencePlot,
                plot_types.CommandStatusPlot,
            ],
            [plot_types.CtlRotPlot, plot_types.OrientationPlot, plot_types.CovPlot],
        ]

        self.graphics_view = ParentGraphicsView()
        self.terminal_graphics_view = TerminalGraphicsView(self.graphics_view)
        self.terminal_view = TerminalView(self.terminal_graphics_view)

        self.layout = pg.GraphicsLayout(border=(100, 100, 100))
        self.layout.setBorder(None)
        self.layout.setZValue(-1)
        self.graphics_view.setCentralItem(self.layout)
        self.graphics_view.scene().addWidget(self.terminal_graphics_view)
        self.log_shown = False
        self.pmc_enabled = True
        self.setCentralWidget(self.graphics_view)
        self.create_menu()

        row = 0
        for c in range(len(self.columns)):
            l = self.layout.addLayout(0, c)
            l.setBorder(pg.mkColor(150, 150, 150))
            for r in range(len(self.columns[c])):
                t = self.columns[c][r]
                new_item = t()  # it gets added to l in constructor
                l.addItem(new_item)
                if r != len(self.columns[c]) - 1:
                    l.nextRow()
                else:
                    new_item.show_x_axis(True)
            self.layout.nextColumn()

        self.setWindowTitle("GNC Visualizer")

        self.settings = QtCore.QSettings("NASA", "gnc_visualizer")
        self.restoreGeometry(self.settings.value("geometry", "").toByteArray())
        self.restoreState(self.settings.value("windowState", "").toByteArray())

        QtGui.qApp.installEventFilter(self)
        # make sure initial window size includes menubar
        QtCore.QTimer.singleShot(0, self.menuBar().hide)

        self.started = False
        self.paused = False
        self.proc = None
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(40)

    def closeEvent(self, event):
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        QtGui.QMainWindow.closeEvent(self, event)

    def startProcess(self):
        if self.launch_command != None:
            self.print_to_log("Starting process: %s" % (self.launch_command), "#FFB266")
            self.proc = subprocess.Popen(
                self.launch_command,
                preexec_fn=os.setsid,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.pmc_enabled = True

    def stopProcess(self):
        if self.proc != None:
            self.print_to_log("Stopping process.", "#FFB266")
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            # (output, error) = v.proc.communicate()
            self.proc.wait()
            self.proc = None

    def eventFilter(self, source, event):
        # do not hide menubar when menu shown
        if QtGui.qApp.activePopupWidget() is None:
            if event.type() == QtCore.QEvent.MouseMove:
                if self.menuBar().isHidden():
                    rect = self.geometry()
                    # set mouse-sensitive zone
                    rect.setHeight(25)
                    if rect.contains(event.globalPos()):
                        self.menuBar().show()
                else:
                    rect = QtCore.QRect(
                        self.menuBar().mapToGlobal(QtCore.QPoint(0, 0)),
                        self.menuBar().size(),
                    )
                    if not rect.contains(event.globalPos()):
                        self.menuBar().hide()
            elif event.type() == QtCore.QEvent.Leave and source is self:
                self.menuBar().hide()
        return QtGui.QMainWindow.eventFilter(self, source, event)

    def delete_plot(self, col, row):
        del self.columns[col][row]
        l = self.layout.getItem(0, col)

        self.layout.removeItem(l)
        new_layout = pg.GraphicsLayout()
        new_layout.setBorder(pg.mkColor(150, 150, 150))
        self.layout.addItem(new_layout, 0, col)

        # we recreate the entire column layout, since we can't
        # delete the new row...
        r = 0
        first = True
        while True:
            if r != row:
                item = l.getItem(r, 0)
                if item == None:
                    break
                if not first:
                    new_layout.nextRow()
                else:
                    first = False
                new_layout.addItem(item)
                item.show_x_axis(True if r == row - 1 else False)
            r += 1

        if len(self.columns[col]) == 0:
            self.delete_column(col)
        self.create_menu()

    def delete_column(self, col):
        del self.columns[col]
        self.create_menu()
        l = self.layout.getItem(0, col)
        self.layout.removeItem(l)

    def add_plot(self, plot, col):
        if len(self.columns) <= col:
            self.columns.append([])
            l = self.layout.addLayout(0, col)
            l.setBorder(pg.mkColor(150, 150, 150))
        self.columns[col].append(plot)
        column = self.layout.getItem(0, col)
        if len(self.columns[col]) > 1:
            column.getItem(len(self.columns[col]) - 2, 0).show_x_axis(False)
            column.nextRow()
        column.addItem(plot())
        column.getItem(len(self.columns[col]) - 1, 0).show_x_axis(True)
        self.create_menu()

    def generate_add_plot_menu(self, column, base, plot_types):
        for p in sorted(plot_types.keys()):
            v = plot_types[p]
            if type(v) == dict:
                self.generate_add_plot_menu(column, base.addMenu(p), v)
            else:
                base.addAction(p, functools.partial(self.add_plot, v, column))

    def create_menu(self):
        self.menuBar().clear()
        viewmenu = self.menuBar().addMenu("Visualizer")
        viewmenu.addAction("Pause / Resume (Space)", self.toggle_paused)
        viewmenu.addAction("Start / Stop Process (s)", self.start_stop_process)
        viewmenu.addAction("Show / Hide Log (l)", self.toggle_log_shown)
        viewmenu.addAction("Toggle Full Screen (f)", self.toggle_full_screen)
        viewmenu.addAction("Quit (q)", self.quit)
        commandmenu = self.menuBar().addMenu("Commands")
        commandmenu.addAction("Reset EKF (r)", self.reset_ekf)
        commandmenu.addAction("Initialize Bias (b)", self.initialize_bias)
        commandmenu.addAction("Choose Plan (c)", self.choose_plan)
        commandmenu.addAction("Run Plan (p)", self.run_plan)
        commandmenu.addAction("Undock (u)", self.undock)
        commandmenu.addAction("Enable / Disable PMC (m)", self.toggle_pmc)
        for i in range(len(self.columns) + 1):
            colmenu = self.menuBar().addMenu("Column " + str(i + 1))
            if i < len(self.columns):
                col = self.columns[i]
                for j in range(len(col)):
                    colmenu.addAction(
                        plot_types.plot_display_names[col[j]],
                        functools.partial(self.delete_plot, i, j),
                    )
            colmenu.addSeparator()
            add_menu = colmenu.addMenu("Add Plot")
            self.generate_add_plot_menu(i, add_menu, plot_types.plot_types)
            colmenu.addSeparator()
            if i < len(self.columns):
                colmenu.addAction(
                    "Delete Column", functools.partial(self.delete_column, i)
                )

        if self.com_method == com.DDS_COM:
            # Disable menu actions for commands
            for action in self.menuBar().actions():
                if action.menu() and action.text() == "Commands":
                    for act in action.menu().actions():
                        act.setEnabled(False)

    def quit(self):
        self.close()

    def toggle_paused(self):
        self.paused = not self.paused

    def toggle_full_screen(self):
        self.setWindowState(self.windowState() ^ QtCore.Qt.WindowFullScreen)

    def toggle_log_shown(self):
        if self.log_shown:
            self.terminal_graphics_view.scene().removeItem(self.terminal_view)
        else:
            self.terminal_graphics_view.scene().addItem(self.terminal_view)
        self.log_shown = not self.log_shown

    def initialize_bias(self):
        com_manager.initialize_bias()

    def reset_ekf(self):
        com_manager.reset_ekf()

    def start_stop_process(self):
        if self.launch_command != None:
            if self.proc == None:
                self.startProcess()
            else:
                self.stopProcess()

    def run_plan(self):
        if self.plan != None:
            ret = os.system(
                ASTROBEE_ROOT
                + "/scripts/teleop/run_plan.sh "
                + self.plan
                + " > /dev/null &"
            )
        else:
            self.print_to_log("No plan file specified.", "#FF0000")

    def undock(self):
        ret = os.system(
            "bash " + ASTROBEE_ROOT + "/scripts/teleop/run_undock.sh > /dev/null &"
        )

    def toggle_pmc(self):
        self.pmc_enabled = com_manager.toggle_pmc(self.pmc_enabled)

    def choose_plan(self):
        result = QtGui.QFileDialog.getOpenFileName(
            self,
            "Open Plan",
            ASTROBEE_ROOT,
            "Plans (*.fplan)",
        )
        if result != None:
            self.plan = str(result)

    def keyPressEvent(self, event):
        super(Visualizer, self).keyPressEvent(event)
        if event.key() == QtCore.Qt.Key_Escape or event.key() == QtCore.Qt.Key_Q:
            self.quit()
        if event.key() == QtCore.Qt.Key_Space:
            self.toggle_paused()
        if event.key() == QtCore.Qt.Key_F:
            self.toggle_full_screen()
        if event.key() == QtCore.Qt.Key_L:
            self.toggle_log_shown()
        if event.key() == QtCore.Qt.Key_B:
            self.initialize_bias()
        if event.key() == QtCore.Qt.Key_R:
            self.reset_ekf()
        if event.key() == QtCore.Qt.Key_S:
            self.start_stop_process()
        if event.key() == QtCore.Qt.Key_P:
            self.run_plan()
        if event.key() == QtCore.Qt.Key_U:
            self.undock()
        if event.key() == QtCore.Qt.Key_C:
            self.choose_plan()
        if event.key() == QtCore.Qt.Key_M:
            self.toggle_pmc()

    def remove_old_data(self):
        t = time.time() - start_time
        for c in callbacks_list:
            while (
                self.data_sizes[c.time_key] > 0
                and t - self.data[c.time_key][self.data_sizes[c.time_key] - 1]
                > plot_types.DISPLAY_TIME + 2
            ):
                self.data_sizes[c.time_key] -= 1

    def tick(self):
        if com_manager.was_shutdown():
            self.hide()
            return
        self.remove_old_data()
        if not self.paused and self.started:
            col = 0
            while True:
                column = self.layout.getItem(0, col)
                if column == None:
                    break
                row = 0
                while True:
                    item = column.getItem(row, 0)
                    if item == None:
                        break
                    item.update_plot((self.data, self.data_sizes))
                    row += 1
                col += 1
        if self.proc != None:
            ret = self.proc.poll()
            if ret != None:
                (out, err) = self.proc.communicate()
                self.proc = None
                ansi_escape = re.compile(r"\x1b[^m]*m")
                for line in err.splitlines():
                    self.print_to_log(ansi_escape.sub("", line), "#FF0000")
                self.print_to_log("Process terminated. Press 's' to re-run.", "#FFFFFF")
        if self.log_updated:
            self.terminal_view.setHtml(self.log_text)
            self.log_updated = False
        r = self.terminal_view.boundingRect()
        diff = min(self.terminal_graphics_view.height(), r.height())
        r.setY(r.height() - diff)
        r.setHeight(diff)
        r.setWidth(self.terminal_graphics_view.width() - 100)
        self.terminal_graphics_view.ensureVisible(r)

    def add_data(self, callbacks, data):
        self.data_sizes[callbacks.time_key] = min(
            self.data_sizes[callbacks.time_key] + 1, ARRAY_SIZE
        )
        for key in callbacks.callbacks:
            self.data[key] = np.roll(self.data[key], 1)
            self.data[key][0] = callbacks.callbacks[key](data)

    def ekf_callback(self, data):
        self.add_data(ekf_callbacks, data)
        if data.ml_count != 0:
            self.add_data(ml_callbacks, data)
        if data.of_count != 0:
            self.add_data(of_callbacks, data)
        self.started = True

    def ground_truth_callback(self, data):
        self.add_data(truth_callbacks, data)

    def ml_pose_callback(self, data):
        self.add_data(ml_pose_callbacks, data)

    def command_callback(self, data):
        self.add_data(command_callbacks, data)

    def traj_callback(self, data):
        self.add_data(traj_callbacks, data)

    def shaper_callback(self, data):
        self.add_data(shaper_callbacks, data)

    def pmc_callback(self, data):
        self.add_data(pmc_callbacks, data)

    def print_to_log(self, text, color):
        self.log_text += "<br /><font color='%s'>%s</font>" % (color, text)
        self.log_lines += 1
        self.log_updated = True
        if self.log_lines > self.max_log_length:
            self.log_text = self.log_text[self.log_text.find("<br />") + 1 :]

    def log_callback(self, data):
        if (
            data.name in ["/recorder"]
            or data.msg.startswith("waitForService")
            or data.msg.startswith("Initializing nodelet with")
            or data.msg.startswith("Loading nodelet")
        ):
            return
        log_colors = {
            1: "#C0C0C0",
            2: "#FFFFFF",
            4: "#FFB266",
            8: "#FF6666",
            16: "#FF0000",
        }
        self.print_to_log(data.msg, log_colors[data.level])


def sigint_handler(*args):
    QtGui.QApplication.quit()


def main():
    com_method = com.DDS_COM

    parser = argparse.ArgumentParser(description="Gnc visualization.")

    # ROS arguments only
    parser.add_argument(
        "--gantry",
        dest="launch_command",
        action="append_const",
        const="roslaunch astrobee proto4c.launch disable_fans:=true",
        help="Launch proto4.",
    )
    parser.add_argument(
        "--granite",
        dest="launch_command",
        action="append_const",
        const="roslaunch astrobee granite.launch",
        help="Launch proto4.",
    )
    parser.add_argument(
        "--bag",
        dest="launch_command",
        action="append_const",
        const="roslaunch astrobee play_bag.launch",
        help="Launch play_bag.launch.",
    )
    parser.add_argument(
        "--sim",
        dest="launch_command",
        action="append_const",
        const="roslaunch astrobee sim.launch",
        help="Launch simulator.launch.",
    )
    parser.add_argument(
        "--plan", dest="plan", action="store", help="The plan to execute."
    )
    parser.add_argument(
        "--disable_pmcs",
        dest="disable_pmcs",
        action="store_true",
        help="Disable the pmcs.",
    )

    # General argument
    parser.add_argument(
        "--comm",
        dest="com_method",
        action="store",
        help="Communication method to the robot: dds or ros",
    )

    # DDS arguments only
    parser.add_argument(
        "--use_ip",
        dest="use_ip",
        action="store",
        help="Type an IP to be treated as initial peer for DDS.",
    )
    parser.add_argument(
        "--robot_name",
        dest="robot_name",
        action="store",
        help="Type the name of the robot you want to hear telemetry of over DDS.",
    )
    parser.add_argument(
        "--domain", dest="domain", action="store", help="DDS Domain ID. Default 37 "
    )
    parser.add_argument(
        "--public_ip",
        dest="public_ip",
        action="store",
        help="TReK IP. Only use if needed",
    )

    args, unknown = parser.parse_known_args()

    com_method = com.ROS_COM

    if args.com_method != None:
        if args.com_method in (com.DDS_COM, com.ROS_COM):
            com_method = args.com_method
        else:
            print >> sys.stderr, "Invalid communication method. Must be dds or ros"
            return

    launch_command = None

    # Exclude ROS arguments when using DDS communication
    if com_method == com.DDS_COM and (
        args.launch_command != None or args.disable_pmcs or args.plan != None
    ):
        print >> sys.stderr, (
            "\n###\n"
            + "\nAdditional arguments (--gantry --granite --bag --sim --plan --disable_pmcs) "
            + 'will not be processed when using DDS mode. You may use "--comm ros" or do not include this '
            + "argument at all in order to use additional arguments.\n\n###\n"
        )
        return
    # Exclude DDS commands when using ROS communication
    elif com_method == com.ROS_COM and (
        args.use_ip != None
        or args.robot_name != None
        or args.public_ip != None
        or args.domain != None
    ):
        print >> sys.stderr, (
            "\n###\n"
            + "\nAdditional arguments (--use_ip --robot_name --public_ip) "
            + 'will not be processed when using ROS mode. You may include "--comm dds" '
            + "argument in order to use these additional arguments.\n\n###\n"
        )
        return
    else:
        if args.launch_command == None:
            args.launch_command = []
        if len(args.launch_command) > 1:
            print >> sys.stderr, "Can only specify one launch command."
            return
        if len(args.launch_command) == 1:
            launch_command = args.launch_command[0]
        if args.disable_pmcs:
            if launch_command != None:
                launch_command += " disable_fans:=true"

    # Setting communication method
    dds_args = dict(
        partition_name=args.robot_name,
        given_peer=args.use_ip,
        domain=args.domain,
        public_ip=args.public_ip,
    )
    if not com_manager.set_com_method(com_method, dds_args):
        return

    app = QtGui.QApplication([])
    signal.signal(signal.SIGINT, sigint_handler)
    v = Visualizer(launch_command, args.plan, com_manager.current_com_method)

    try:
        # TODO(rgarciar): Catch errors from Threads started in com_manager
        # Starting communications
        com_manager.start_communications(v)
        v.show()
        v.startProcess()
        app.exec_()
    except (KeyboardInterrupt, SystemExit):
        print("GVIZ will exit")
    except Exception as e:
        print(e)
    finally:
        v.hide()
        v.stopProcess()
        com_manager.stop_communications()
        if com_manager.current_com_method == com.DDS_COM:
            com_manager.config.destroy_dom()


if __name__ == "__main__":
    main()
