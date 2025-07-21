import sys
import json
import os
import numpy as np
from typing import Tuple
from pathlib import Path
from datetime import datetime
import traceback

from PyQt5.QtWidgets import (
    QWidget, QGraphicsScene, QGraphicsView, QApplication, QGridLayout,
    QGraphicsEllipseItem, QGraphicsItem, QTableWidget, QHeaderView, QTableWidgetItem,
    QHBoxLayout, QPushButton, QVBoxLayout, QTabWidget, QLabel, QTextEdit, QComboBox, QCheckBox,
)
from PyQt5.QtGui import QPen, QBrush, QColor, QPolygonF, QPainterPath, QPainter
from PyQt5.QtCore import QPointF, Qt, pyqtSlot, QLineF, QRectF, pyqtSignal
import yaml

# Additional imports for dynamic scaling and velocity graphs
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from widgets.raceline_tuner.utils import *
from styles import *


class SmoothingWidget(QWidget):
    def __init__(self, parent_tuner):
        super(SmoothingWidget, self).__init__()
        self.smooth_results = {}
        self.smoothing_iterations = [0,1,2, 4, 8]
        self.max_smooth_iters = max(self.smoothing_iterations)
        self.selected_smooth_iter = self.smoothing_iterations[0]
        self.smooth_factor = 0.1
        self.filter_size_ratio = 0.1
        self.filter_size_iter_scale = 1.0
        self.smooth_offset = 20
        # Removed fixed draw_scale; will use parent's dynamic draw_scale

        self._last_computed_raceline = {}
        self._last_save_path = None
        self.parent_tuner = parent_tuner
        layout = QVBoxLayout()

        header_widget = QWidget()
        header_layout = QHBoxLayout(header_widget)
        header_layout.addWidget(QLabel("Smoothing"))
        compute_button = QPushButton("Compute")
        compute_button.clicked.connect(self.compute)
        compute_button.setStyleSheet(button_stylesheet)
        header_layout.addWidget(compute_button)
        accept_button = QPushButton("Accept")
        accept_button.setStyleSheet(button_stylesheet)
        accept_button.clicked.connect(self.deploy_action)
        header_layout.addWidget(accept_button)
        layout.addWidget(header_widget)

        
        smoothing_grid = QGridLayout()
        smoothing_header_widget = QWidget()
        

        label_raceline = QLabel("Original")
        smoothing_grid.addWidget(label_raceline, 0, 0)

        smoothing_header_layout = QHBoxLayout(smoothing_header_widget)
        label_smoothing = QLabel("Smoothing")
        smoothing_header_layout.addWidget(label_smoothing)
        self.iteration_combobox = QComboBox()
        for it in self.smoothing_iterations:
            self.iteration_combobox.addItem(str(it))
        self.iteration_combobox.currentIndexChanged.connect(self.on_iteration_changed)
        smoothing_header_layout.addWidget(self.iteration_combobox)
        smoothing_header_layout.setContentsMargins(0, 0, 0, 0)
        smoothing_grid.addWidget(smoothing_header_widget, 0, 1)

        self.smoothing_view_left = QGraphicsView(QGraphicsScene(self))
        self.smoothing_view_right = QGraphicsView(QGraphicsScene(self))
        smoothing_grid.addWidget(self.smoothing_view_left, 1, 0)
        smoothing_grid.addWidget(self.smoothing_view_right, 1, 1)

        layout.addLayout(smoothing_grid)
        self.setLayout(layout)
        self.start_index = None
        self.end_index = None

    
    def deploy_action(self):
        self.compute()
        self.submit_points()

    def submit_points(self):
        self.parent_tuner.raceline.insert_points(self.smooth_results[self.selected_smooth_iter])
        self.parent_tuner.update_scene()

    def on_iteration_changed(self, index):
        self.selected_smooth_iter = self.smoothing_iterations[index]
        raceline = self.smooth_results[self.selected_smooth_iter]
        scene = self.smoothing_view_right.scene()
        scene.clear()
        self.update_scene(scene, raceline)

    def setInterval(self, start_index, end_index):
        self.start_index = start_index
        self.end_index = end_index


    def smooth_points(self):
        raceline = self.parent_tuner.raceline()[:, 2:4]
        if self.parent_tuner.current_sector is not None:
            start_index, end_index = self.parent_tuner.current_sector
            start_index = max(0, start_index - self.smooth_offset)
            end_index = max(raceline.shape[0], start_index + self.smooth_offset)
        else:
            start_index, end_index = 0, raceline.shape[0]
        filter_size = max(40, int(raceline.shape[0] * self.filter_size_ratio))
        # TODO filter_size = self.parameters["filter_size_"]
        self.smooth_results = {}
        self.smooth_results[0] = raceline
        for i in range(self.max_smooth_iters):
            raceline_new = smooth_centerline(raceline, force_filter_length=filter_size)
            raceline[start_index:end_index] = raceline_new[start_index:end_index]
            filter_size = int(filter_size * self.filter_size_iter_scale)
            if i+1 in self.smoothing_iterations: 
                self.smooth_results[i+1] = raceline
        raceline = self.smooth_results[self.selected_smooth_iter]
        scene = self.smoothing_view_right.scene()
        scene.clear()
        self.update_scene(scene, raceline)


    def update_scene(self, scene, raceline):
        scene.clear()
        scale = self.parent_tuner.draw_scale * 0.3
        self.parent_tuner._print_tb(scene, self.parent_tuner.raceline.inner_trackbounds, scale)
        self.parent_tuner._print_tb(scene, self.parent_tuner.raceline.outer_trackbounds, scale)
        path = QPainterPath()
        ref_line = raceline
        start_x, start_y = scale * ref_line[0][0], scale * ref_line[0][1]
        path.moveTo(start_x, start_y)
        for point in ref_line[1:]:
            x, y = scale * point[0], scale * point[1]
            path.lineTo(x, y)
        pen = QPen(QColor(200, 200, 200, 200), 2)
        scene.addPath(path, pen)

    def update_initial_scene(self):
        raceline = self.parent_tuner.raceline()[:, 2:4]
        self.update_scene(self.smoothing_view_left.scene(), raceline)

    @pyqtSlot()
    def compute(self):
        self.smooth_points()