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

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

import time

DISPLAY_TIME = 10

colors = [pg.mkColor(65, 105, 225),
          pg.mkColor(255, 100, 100),
          pg.mkColor(255, 255, 100),
          pg.mkColor(32, 178, 170),
          pg.mkColor(153, 50, 204),
          pg.mkColor(255, 250, 250)
          ]

colors2 = [pg.mkColor(135, 206, 235),
           pg.mkColor(255, 160, 122),
           pg.mkColor(255, 255, 150)]

start_time = time.time()

class VisualizerPlot(pg.PlotItem):
    def __init__(self):
        super(VisualizerPlot, self).__init__()
    def update_plot(self, data):
        pass

class GraphPlot(VisualizerPlot):
    def __init__(self, title, y_label, show_x_axis=False):
        super(GraphPlot, self).__init__()
        self.setTitle(title)
        self.setLabel('left', y_label)
        self.setLabel('bottom', 'Time', 's')
        self.hideButtons()
        self.setDownsampling(mode='subsample')
        #self.setClipToView(True)
        self.show_x_axis(False)

    def show_x_axis(self, show):
        self.showAxis('bottom', show)

    def update_plot(self, data):
        super(GraphPlot, self).update_plot(data)
        t = time.time() - start_time
        self.setXRange(max(0, t - DISPLAY_TIME), t)
        self.setLimits(xMin=max(0, t - DISPLAY_TIME), xMax=t)

class PathItem(pg.QtGui.QGraphicsPathItem):
    def __init__(self, pen):
        self.path = pg.arrayToQPath(np.zeros(0), np.zeros(0), 'all')
        pg.QtGui.QGraphicsPathItem.__init__(self, self.path)
        self.setPen(pen)
        self.last_time = None
    def shape(self): # override because QGraphicsPathItem.shape is too expensive.
        return pg.QtGui.QGraphicsItem.shape(self)
    def boundingRect(self):
        return self.path.boundingRect()
    def setData(self, x_values, y_values):
        self.path = pg.arrayToQPath(x_values, y_values, 'all')
        self.setPath(self.path)

class VectorPlot(GraphPlot):
    def __init__(self, name, y_axis, time_names, y_names, legend_names=None, pens=None):
        a = time.time()
        super(VectorPlot, self).__init__(name, y_axis)
        self.y_names = y_names
        self.time_names = time_names
        if type(self.time_names) != list: # if all the same, can just specify one
            self.time_names = [time_names] * len(y_names)
        self.plot_data = []
        if legend_names != None:
            self.addLegend(offset=(30, 10))
        if pens == None:
            pens = colors
        for i in range(len(self.y_names)):
            name = None if legend_names == None or len(legend_names) <= i else legend_names[i]
            self.plot_data.append(PathItem(pens[i]))
            self.addItem(self.plot_data[-1], name=name)
            if name != None:
                style = pg.PlotDataItem(pen=pens[i], name=name)
                self.legend.addItem(style, name)

    def update_plot(self, data):
        super(VectorPlot, self).update_plot(data)
        for i in range(len(self.y_names)):
            s = data[1][self.time_names[i]]
            self.plot_data[i].setData(data[0][self.time_names[i]][:s], data[0][self.y_names[i]][:s])

class PositionPlot(GraphPlot):
    def __init__(self):
        super(PositionPlot, self).__init__('Position', 'Position (m)')
        self.pos_x = self.plot(pen=colors[0])
        self.pos_y = self.plot(pen=colors[1])
        self.pos_z = self.plot(pen=colors[2])
        self.truth_x = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[0], symbolSize=5)
        self.truth_y = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[1], symbolSize=5)
        self.truth_z = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[2], symbolSize=5)
        self.ml_pose_x = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[0], symbolSize=5)
        self.ml_pose_y = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[1], symbolSize=5)
        self.ml_pose_z = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[2], symbolSize=5)


    def update_plot(self, data):
        super(PositionPlot, self).update_plot(data)
        s = data[1]['ekf_time']
        self.pos_x.setData(data[0]['ekf_time'][:s], data[0]['ekf_position_x'][:s])
        self.pos_y.setData(data[0]['ekf_time'][:s], data[0]['ekf_position_y'][:s])
        self.pos_z.setData(data[0]['ekf_time'][:s], data[0]['ekf_position_z'][:s])
        s = data[1]['truth_time']
        self.truth_x.setData(data[0]['truth_time'][:s], data[0]['truth_position_x'][:s])
        self.truth_y.setData(data[0]['truth_time'][:s], data[0]['truth_position_y'][:s])
        self.truth_z.setData(data[0]['truth_time'][:s], data[0]['truth_position_z'][:s])
        s = data[1]['ml_pose_time']
        self.ml_pose_x.setData(data[0]['ml_pose_time'][:s], data[0]['ml_pose_position_x'][:s])
        self.ml_pose_y.setData(data[0]['ml_pose_time'][:s], data[0]['ml_pose_position_y'][:s])
        self.ml_pose_z.setData(data[0]['ml_pose_time'][:s], data[0]['ml_pose_position_z'][:s])

class VelocityPlot(VectorPlot):
    def __init__(self):
        super(VelocityPlot, self).__init__('Velocity', 'Velocity (m/s)',
                'ekf_time', ['ekf_vel_x', 'ekf_vel_y', 'ekf_vel_z'])

class AccelerationPlot(VectorPlot):
    def __init__(self):
        super(AccelerationPlot, self).__init__('Acceleration', 'Acceleration (m/s^2)',
                'ekf_time', ['ekf_accel_x', 'ekf_accel_y', 'ekf_accel_z'])

class AccelerationBiasPlot(VectorPlot):
    def __init__(self):
        super(AccelerationBiasPlot, self).__init__('Accel Bias', 'Accel Bias (m/s^2)',
                'ekf_time', ['ekf_accel_bias_x', 'ekf_accel_bias_y', 'ekf_accel_bias_z'])

class OrientationPlot(GraphPlot):
    def __init__(self):
        super(OrientationPlot, self).__init__('Orientation', 'Euler Angles (degrees)')
        self.rot_x = self.plot(pen=colors[0])
        self.rot_y = self.plot(pen=colors[1])
        self.rot_z = self.plot(pen=colors[2])
        self.truth_x = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[0], symbolSize=5)
        self.truth_y = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[1], symbolSize=5)
        self.truth_z = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[2], symbolSize=5)
        self.ml_pose_x = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[0], symbolSize=5)
        self.ml_pose_y = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[1], symbolSize=5)
        self.ml_pose_z = self.plot(symbol='d', pen=None, symbolPen=None, symbolBrush=colors[2], symbolSize=5)


    def update_plot(self, data):
        super(OrientationPlot, self).update_plot(data)
        s = data[1]['ekf_time']
        self.rot_x.setData(data[0]['ekf_time'][:s], data[0]['ekf_rot_x'][:s])
        self.rot_y.setData(data[0]['ekf_time'][:s], data[0]['ekf_rot_y'][:s])
        self.rot_z.setData(data[0]['ekf_time'][:s], data[0]['ekf_rot_z'][:s])
        s = data[1]['truth_time']
        self.truth_x.setData(data[0]['truth_time'][:s], data[0]['truth_rot_x'][:s])
        self.truth_y.setData(data[0]['truth_time'][:s], data[0]['truth_rot_y'][:s])
        self.truth_z.setData(data[0]['truth_time'][:s], data[0]['truth_rot_z'][:s])
        s = data[1]['ml_pose_time']
        self.ml_pose_x.setData(data[0]['ml_pose_time'][:s], data[0]['ml_pose_rot_x'][:s])
        self.ml_pose_y.setData(data[0]['ml_pose_time'][:s], data[0]['ml_pose_rot_y'][:s])
        self.ml_pose_z.setData(data[0]['ml_pose_time'][:s], data[0]['ml_pose_rot_z'][:s])


class OmegaPlot(VectorPlot):
    def __init__(self):
        super(OmegaPlot, self).__init__('Angular Velocity', 'Omega (degrees)',
                'ekf_time', ['ekf_omega_x', 'ekf_omega_y', 'ekf_omega_z'])

class GyroBiasPlot(VectorPlot):
    def __init__(self):
        super(GyroBiasPlot, self).__init__('Gyro Bias', 'Gyro Bias (degrees)',
                'ekf_time', ['ekf_gyro_bias_x', 'ekf_gyro_bias_y', 'ekf_gyro_bias_z'])

class ModePlot(GraphPlot):
    def __init__(self, name, x_value, y_value, colormap):
        super(ModePlot, self).__init__(name, "")
        self.setMaximumSize(5000, 100)
        self.confidence_rects = []
        self.last_value = None
        self.x_value = x_value
        self.y_value = y_value
        self.colormap = colormap

    def update_plot(self, data):
        super(ModePlot, self).update_plot(data)
        t = time.time() - start_time
        for r in self.confidence_rects:
            if r.rect().x() + r.rect().width() < t - DISPLAY_TIME:
                self.removeItem(r)
                del r
            else:
                break
        new_items = 0
        if len(self.confidence_rects) == 0:
            new_items = len(data[0][self.x_value])
        else:
            for r in data[0][self.x_value]:
                if r > self.confidence_rects[-1].rect().x() + self.confidence_rects[-1].rect().width():
                    new_items += 1
                else:
                    break
        for j in range(new_items):
            i = new_items - 1 - j
            val = data[0][self.y_value][i]
            if val == self.last_value:
                continue
            self.last_value = val
            if len(self.confidence_rects) > 0:
                r = self.confidence_rects[-1].rect()
                self.confidence_rects[-1].setRect(r.x(), 0, data[0][self.x_value][i] - r.x(), 1)
            start_t = data[0][self.x_value][i]
            r = QtGui.QGraphicsRectItem(start_t, 0, 0, 1)
            r.setBrush(self.colormap[int(val)])
            self.addItem(r)
            self.confidence_rects.append(r)
        r = self.confidence_rects[-1].rect()
        self.confidence_rects[-1].setRect(r.x(), 0, t - r.x(), 1)
        self.setYRange(0, 1)
        self.setLimits(yMin=0, yMax=1)

class ConfidencePlot(ModePlot):
    def __init__(self):
        super(ConfidencePlot, self).__init__('Confidence', 'ekf_time', 'ekf_confidence',
            [pg.mkColor(0, 255, 0), pg.mkColor(255, 255, 0), pg.mkColor(255, 0, 0)])

class CovPlot(VectorPlot):
    def __init__(self):
        super(CovPlot, self).__init__('Covariance Magnitudes', 'Covariance',
                'ekf_time', ['ekf_cov_rot_m', 'ekf_cov_gbias_m', 'ekf_cov_vel_m', 'ekf_cov_abias_m', 'ekf_cov_pos_m'],
                ['Rotation', 'Gyro Bias', 'Velocity', 'Accel Bias', 'Position'])
    def update_plot(self, data):
        super(CovPlot, self).update_plot(data)
        #self.plot.setLogMode(y=True)

class CovRotPlot(VectorPlot):
    def __init__(self):
        super(CovRotPlot, self).__init__('Rotation Covariance', 'Rotation Covariance',
                'ekf_time', ['ekf_cov_rot_x', 'ekf_cov_rot_y', 'ekf_cov_rot_z'])

class CovGBiasPlot(VectorPlot):
    def __init__(self):
        super(CovGBiasPlot, self).__init__('Gyro Bias Covariance', 'Gyro Bias Covariance',
                'ekf_time', ['ekf_cov_gbias_x', 'ekf_cov_gbias_y', 'ekf_cov_gbias_z'])

class CovVelPlot(VectorPlot):
    def __init__(self):
        super(CovVelPlot, self).__init__('Velocity Covariance', 'Velocity Covariance',
                'ekf_time', ['ekf_cov_vel_x', 'ekf_cov_vel_y', 'ekf_cov_vel_z'])

class CovABiasPlot(VectorPlot):
    def __init__(self):
        super(CovABiasPlot, self).__init__('Accel Bias Covariance', 'Accel Bias Covariance',
                'ekf_time', ['ekf_cov_abias_x', 'ekf_cov_abias_y', 'ekf_cov_abias_z'])
        
class CovPosPlot(VectorPlot):
    def __init__(self):
        super(CovPosPlot, self).__init__('Position Covariance', 'Position Covariance',
                'ekf_time', ['ekf_cov_pos_x', 'ekf_cov_pos_y', 'ekf_cov_pos_z'])

class FeatureCountPlot(GraphPlot):
    def __init__(self):
        super(FeatureCountPlot, self).__init__('Observations', 'Number of Features')
        self.setMaximumSize(5000, 200)
        self.features = []

    def update_plot(self, data):
        super(FeatureCountPlot, self).update_plot(data)
        t = time.time() - start_time
        for r in self.features:
            if r.x() < t - DISPLAY_TIME - 1:
                self.removeItem(r)
                del r
            else:
                break
        new_ml = 0
        if len(self.features) == 0:
            new_ml = len(filter(lambda x: x > 1e-2, data[0]['ml_time']))
        else:
            for r in data[0]['ml_time']:
                if r > self.features[-1].x() + 1e-5 and r > 1e-2:
                    new_ml += 1
                else:
                    break
        new_of = 0
        if len(self.features) == 0:
            new_of = len(filter(lambda x: x > 1e-2, data[0]['of_time']))
        else:
            for r in data[0]['of_time']:
                if r > self.features[-1].x() + 1e-5 and r > 1e-2:
                    new_of += 1
                else:
                    break
        cur_ml = new_ml - 1
        cur_of = new_of - 1
        while cur_ml >= 0 or cur_of >= 0:
            add_ml = True
            if cur_ml < 0:
                add_ml = False
            elif cur_of >= 0 and data[0]['of_time'][cur_of] < data[0]['ml_time'][cur_ml]:
                add_ml = False
            t = data[0]['ml_time'][cur_ml] if add_ml else data[0]['of_time'][cur_of]
            v = data[0]['ml_landmarks'][cur_ml] if add_ml else data[0]['of_landmarks'][cur_of]
            r = QtGui.QGraphicsEllipseItem(-5, -5, 10, 10) if add_ml else QtGui.QGraphicsEllipseItem(-3, -3, 6, 6)
            r.setPos(t, v)
            r.setBrush(colors[1] if add_ml else colors[0])
            r.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)
            self.addItem(r)
            self.features.append(r)
            if add_ml:
                cur_ml -= 1
            else:
                cur_of -= 1

class MLMahalPlot(GraphPlot):
    def __init__(self):
        super(MLMahalPlot, self).__init__('Mahalanobis Distance', 'Mahalanobis Distance')
        self.mahal_min  = pg.PlotDataItem([], [], pen=None, symbol='t')
        self.mahal_max  = pg.PlotDataItem([], [], pen=None, symbol='t')
        self.mahal_mean = pg.PlotDataItem([], [], pen=None, symbol='o', symbolBrush=colors[1])
        self.addItem(self.mahal_min)
        self.addItem(self.mahal_max)
        self.addItem(self.mahal_mean)

    def update_plot(self, data):
        super(MLMahalPlot, self).update_plot(data)
        s = data[1]['ml_time']
        self.mahal_min.setData( data[0]['ml_time'][:s], data[0]['ml_mahal_min'][:s])
        self.mahal_max.setData( data[0]['ml_time'][:s], data[0]['ml_mahal_max'][:s])
        self.mahal_mean.setData(data[0]['ml_time'][:s], data[0]['ml_mahal_mean'][:s])

class CommandForcePlot(VectorPlot):
    def __init__(self):
        super(CommandForcePlot, self).__init__('CTL Force', 'Force (N)',
                'command_time', ['command_force_x', 'command_force_y', 'command_force_z'])

class CommandTorquePlot(VectorPlot):
    def __init__(self):
        super(CommandTorquePlot, self).__init__('CTL Torque', 'Torque(N m)',
                'command_time', ['command_torque_x', 'command_torque_y', 'command_torque_z'])

class CommandPosErrPlot(VectorPlot):
    def __init__(self):
        super(CommandPosErrPlot, self).__init__('CTL Position Error', 'Position Error (m)',
                'command_time', ['command_pos_err_x', 'command_pos_err_y', 'command_pos_err_z'])

class CommandPosErrIntPlot(VectorPlot):
    def __init__(self):
        super(CommandPosErrIntPlot, self).__init__('CTL Position Error Integrated', 'Position Error Integrated',
                'command_time', ['command_pos_err_int_x', 'command_pos_err_int_y', 'command_pos_err_int_z'])

class CommandAttErrPlot(VectorPlot):
    def __init__(self):
        super(CommandAttErrPlot, self).__init__('CTL Attitude Error', 'Attitude Error',
                'command_time', ['command_att_err_x', 'command_att_err_y', 'command_att_err_z'])

class CommandAttErrIntPlot(VectorPlot):
    def __init__(self):
        super(CommandAttErrIntPlot, self).__init__('CTL Attitude Error Integrated', 'Attitude Error Integrated',
                'command_time', ['command_att_err_int_x', 'command_att_err_int_y', 'command_att_err_int_z'])

class CommandStatusPlot(ModePlot):
    def __init__(self):
        super(CommandStatusPlot, self).__init__('CTL Status', 'command_time', 'command_status',
            [pg.mkColor(0, 0, 255), pg.mkColor(255, 255, 0), pg.mkColor(0, 255, 0), pg.mkColor(255, 0, 0)])

class CtlPosPlot(VectorPlot):
    def __init__(self):
        super(CtlPosPlot, self).__init__('Control Position', 'Position (m)',
           ['ekf_time',       'truth_time',       'traj_time',       'shaper_time',
            'ekf_time',       'truth_time',       'traj_time',       'shaper_time',
            'ekf_time',       'truth_time',       'traj_time',       'shaper_time'],
           ['ekf_position_x', 'truth_position_x', 'traj_position_x', 'shaper_position_x',
            'ekf_position_y', 'truth_position_y', 'traj_position_y', 'shaper_position_y',
            'ekf_position_z', 'truth_position_z', 'traj_position_z', 'shaper_position_z'],
           legend_names=['EKF X', 'Truth X', 'Trajectory X', 'Shaper X'],
           pens=[pg.mkPen(colors[0]), pg.mkPen(colors[0], style=QtCore.Qt.DotLine, width=2), pg.mkPen(colors[0], style=QtCore.Qt.DotLine), pg.mkPen(colors[0], style=QtCore.Qt.DashLine),
                 pg.mkPen(colors[1]), pg.mkPen(colors[1], style=QtCore.Qt.DotLine, width=2), pg.mkPen(colors[1], style=QtCore.Qt.DotLine), pg.mkPen(colors[1], style=QtCore.Qt.DashLine),
                 pg.mkPen(colors[2]), pg.mkPen(colors[2], style=QtCore.Qt.DotLine, width=2), pg.mkPen(colors[2], style=QtCore.Qt.DotLine), pg.mkPen(colors[2], style=QtCore.Qt.DashLine)]
               )

class CtlRotPlot(VectorPlot):
    def __init__(self):
        super(CtlRotPlot, self).__init__('Control Rotation', 'Euler Angle (deg)',
           ['ekf_time',  'truth_time',  'traj_time',  'shaper_time',
            'ekf_time',  'truth_time',  'traj_time',  'shaper_time',
            'ekf_time',  'truth_time',  'traj_time',  'shaper_time'],
           ['ekf_rot_x', 'truth_rot_x', 'traj_rot_x', 'shaper_rot_x',
            'ekf_rot_y', 'truth_rot_y', 'traj_rot_y', 'shaper_rot_y',
            'ekf_rot_z', 'truth_rot_z', 'traj_rot_z', 'shaper_rot_z'],
           legend_names=['EKF X Rot', 'Truth X Rot', 'Trajectory X Rot', 'Shaper X Rot'],
           pens=[pg.mkPen(colors[0]), pg.mkPen(colors[0], style=QtCore.Qt.DotLine, width=2), pg.mkPen(colors[0], style=QtCore.Qt.DotLine), pg.mkPen(colors[0], style=QtCore.Qt.DashLine),
                 pg.mkPen(colors[1]), pg.mkPen(colors[1], style=QtCore.Qt.DotLine, width=2), pg.mkPen(colors[1], style=QtCore.Qt.DotLine), pg.mkPen(colors[1], style=QtCore.Qt.DashLine),
                 pg.mkPen(colors[2]), pg.mkPen(colors[2], style=QtCore.Qt.DotLine, width=2), pg.mkPen(colors[2], style=QtCore.Qt.DotLine), pg.mkPen(colors[2], style=QtCore.Qt.DashLine)]
               )

class Pmc1NozzlePlot(VectorPlot):
    def __init__(self):
        super(Pmc1NozzlePlot, self).__init__('PMC1 Nozzle Command', 'Ticks',
            'pmc_time', ['pmc_1_nozzle_1', 'pmc_1_nozzle_2', 'pmc_1_nozzle_3', 'pmc_1_nozzle_4', 'pmc_1_nozzle_5', 'pmc_1_nozzle_6' ])

class Pmc2NozzlePlot(VectorPlot):
    def __init__(self):
        super(Pmc2NozzlePlot, self).__init__('PMC2 Nozzle Command', 'Ticks', 
            'pmc_time', ['pmc_2_nozzle_1', 'pmc_2_nozzle_2', 'pmc_2_nozzle_3', 'pmc_2_nozzle_4', 'pmc_2_nozzle_5', 'pmc_2_nozzle_6' ])

class Pmc1BlowerPlot(VectorPlot):
    def __init__(self):
        super(Pmc1BlowerPlot, self).__init__('PMC1 Blower speed', 'Ticks',
            'pmc_time', ['pmc_1_motor_speed'])

class Pmc2BlowerPlot(VectorPlot):
    def __init__(self):
        super(Pmc2BlowerPlot, self).__init__('PMC2 Blower speed', 'Ticks',
            'pmc_time', ['pmc_2_motor_speed'])

plot_types = {
    'EKF' : {
        'Position'              : PositionPlot,
        'Velocity'              : VelocityPlot,
        'Acceleration'          : AccelerationPlot,
        'Accel Bias'            : AccelerationBiasPlot,
        'Orientation'           : OrientationPlot,
        'Angular Velocity'      : OmegaPlot,
        'Gyro Bias'             : GyroBiasPlot,
        'Feature Counts'        : FeatureCountPlot,
        'Confidence'            : ConfidencePlot,
        'Mahalanobis Distances' : MLMahalPlot,
        'Covariance' : {
            'All'               : CovPlot,
            'Rotation'          : CovRotPlot,
            'Gyro Bias'         : CovGBiasPlot,
            'Velocity'          : CovVelPlot,
            'Accel Bias'        : CovABiasPlot,
            'Position'          : CovPosPlot
        }
    },
    'CTL' : {
        'Force'                 : CommandForcePlot,
        'Torque'                : CommandTorquePlot,
        'Position'              : CtlPosPlot,
        'Position Error'        : CommandPosErrPlot,
        'Position Error Integ.' : CommandPosErrIntPlot,
        'Attitude Error'        : CommandAttErrPlot,
        'Attitude Error Integ.' : CommandAttErrIntPlot,
        'Rotation'              : CtlRotPlot,
        'Status'                : CommandStatusPlot
    },
    'PMC' : {
        'PMC1:Nozzle'           : Pmc1NozzlePlot,
        'PMC1:Blower'           : Pmc1BlowerPlot,
        'PMC2:Nozzle'           : Pmc2NozzlePlot,
        'PMC2:Blower'           : Pmc2BlowerPlot
    }
}

plot_display_names = dict()
def __generate_display_names(plot_types, base_name):
    for n in plot_types.keys():
        v = plot_types[n]
        name = base_name + ' ' + n
        if type(v) == dict:
            __generate_display_names(v, name)
        else:
            plot_display_names[v] = name
__generate_display_names(plot_types, "")

