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

from widgets.raceline_tuner.structs import ControlPoint
from widgets.raceline_tuner.custom import GridGraphicsView



class BezierCurveWidget(QWidget):
    def __init__(self, parent_tuner):
        """
        This widget displays and allows editing of a Bezier curve.
        It owns 4 draggable control points and a table listing the computed curve points.
        When the control points change or the compute button is pressed, the new curve is computed and sent to the parent tuner.
        """
        super(BezierCurveWidget, self).__init__()
        self.parent_tuner = parent_tuner
        self.n_curve_points = 20
        self.control_points = []  # List of ControlPoint objects
        self.computed_points = []  # Computed new raceline points
        self.sector_points = []
        self.sector_indices = []  # Added to track the original indices
        self._is_shown = False
        self.is_wrapped_sector = False  # Added flag for wrapped sectors

        self.scene = QGraphicsScene(self)
        self.view = GridGraphicsView(self.scene, self)

        layout = QVBoxLayout(self)
        header_widget = QWidget()
        header_layout = QHBoxLayout(header_widget)
        header_layout.addWidget(QLabel("Bezier Curve Editor"))
        show_button = QPushButton("Show")
        show_button.setStyleSheet(button_stylesheet)
        show_button.clicked.connect(self.show)
        header_layout.addWidget(show_button)
        accept_button = QPushButton("Accept")
        accept_button.setStyleSheet(button_stylesheet)
        accept_button.clicked.connect(self.accept)
        header_layout.addWidget(accept_button)
        layout.addWidget(header_widget)
        layout.addWidget(self.view)
        self.setLayout(layout)

    def reset(self):
        """Reset the Bezier widget state."""
        self.scene.clear()
        self.control_points = []
        self.computed_points = []
        self.sector_points = []
        self.sector_indices = []
        self.is_wrapped_sector = False

    def setSectorPoints(self, raceline_points: list, indices=None):
        self.sector_points = raceline_points
        if indices is not None:
            self.sector_indices = indices
        self.compute()

    def compute(self, *args, **kwargs):
        #import ipdb; ipdb.set_trace()
        raceline_points = kwargs.get('raceline_points', self.sector_points)
        if len(raceline_points) < 4:
            print("Not enough points in sector to fit a Bezier curve.")
            return
        pts_np = np.stack([np.array((pt.scenePos().x(), pt.scenePos().y())) for pt in raceline_points])
        bezier_params = self.fit_cubic_bezier(pts_np)
        P0 = QPointF(bezier_params[0,0], bezier_params[0,1])
        P1 = QPointF(bezier_params[1,0], bezier_params[1,1])
        P2 = QPointF(bezier_params[2,0], bezier_params[2,1])
        P3 = QPointF(bezier_params[3,0], bezier_params[3,1])

        self.control_points = []
        self.scene.clear()
        for i, pt in enumerate([P0, P1, P2, P3]):
            cp = ControlPoint(pt.x(), pt.y(), i, bezier_widget=self, is_movable=(i in [1,2]))
            self.control_points.append(cp)
            self.scene.addItem(cp)
        self.update_curve()
    
    def show(self):
        if self._is_shown:
            self._is_shown = False
            self.parent_tuner.update_scene()
        else:
            self._is_shown = True
            self.update_curve()


    def fit_cubic_bezier(self, points):
        def chord_length_parameterize(points):
            distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
            cumulative = np.insert(np.cumsum(distances), 0, 0)
            return cumulative / cumulative[-1]
        n_points = len(points)
        if n_points < 4:
            raise ValueError("At least 4 points are needed to fit a cubic Bézier curve.")
        t = chord_length_parameterize(points)
        P0 = points[0]
        P3 = points[-1]
        b1 = 3 * t * (1 - t)**2
        b2 = 3 * t**2 * (1 - t)
        B0 = (1 - t)**3
        B3 = t**3
        Q = points - np.outer(B0, P0) - np.outer(B3, P3)
        M = np.column_stack((b1, b2))
        P1x, P2x = np.linalg.lstsq(M, Q[:, 0], rcond=None)[0]
        P1y, P2y = np.linalg.lstsq(M, Q[:, 1], rcond=None)[0]
        P1 = np.array([P1x, P1y])
        P2 = np.array([P2x, P2y])
        return np.array([P0, P1, P2, P3])

    def update_curve(self):
        if len(self.control_points) != 4:
            return

        P0 = np.array([self.control_points[0].scenePos().x(), self.control_points[0].scenePos().y()])
        P1 = np.array([self.control_points[1].scenePos().x(), self.control_points[1].scenePos().y()])
        P2 = np.array([self.control_points[2].scenePos().x(), self.control_points[2].scenePos().y()])
        P3 = np.array([self.control_points[3].scenePos().x(), self.control_points[3].scenePos().y()])

        t_values = np.linspace(0, 1, self.n_curve_points)
        new_points = []
        for t in t_values:
            point = ((1-t)**3)*P0 + 3*((1-t)**2)*t*P1 + 3*(1-t)*(t**2)*P2 + (t**3)*P3
            new_points.append(point)
        self.computed_points = np.array(new_points)

        for item in self.scene.items():
            if not isinstance(item, ControlPoint):
                self.scene.removeItem(item)
        path = QPainterPath()
        path.moveTo(self.computed_points[0][0], self.computed_points[0][1])
        for pt in self.computed_points[1:]:
            path.lineTo(pt[0], pt[1])
        self.scene.addPath(path, QPen(QColor(0, 255, 0), 2))

        if self._is_shown:
            self.parent_tuner.updateBezierRacelinePoints(self.computed_points)

    def changeSector(self):
        if self.parent_tuner.current_sector is None:
            return
        start_idx, end_idx = self.parent_tuner.current_sector
        points = self.parent_tuner.raceline_points
        
        # Check if the sector wraps around the end of the raceline
        self.is_wrapped_sector = start_idx > end_idx
        
        if start_idx <= end_idx:
            indices = np.arange(start_idx, end_idx + 1)
        else:
            indices = np.concatenate([np.arange(start_idx, len(points)), np.arange(0, end_idx + 1)])
        sector_pts = [points[i] for i in indices]
        self.setSectorPoints(sector_pts, indices)

    def compute_points(self, num_points: int):
        if len(self.control_points) != 4:
            return

        P0 = np.array([self.control_points[0].scenePos().x(), self.control_points[0].scenePos().y()])
        P1 = np.array([self.control_points[1].scenePos().x(), self.control_points[1].scenePos().y()])
        P2 = np.array([self.control_points[2].scenePos().x(), self.control_points[2].scenePos().y()])
        P3 = np.array([self.control_points[3].scenePos().x(), self.control_points[3].scenePos().y()])

        t_values = np.linspace(0, 1, num_points)
        new_points = []
        for t in t_values:
            point = ((1-t)**3)*P0 + 3*((1-t)**2)*t*P1 + 3*(1-t)*(t**2)*P2 + (t**3)*P3
            new_points.append(point)
        computed_points = np.array(new_points)
        return computed_points

    def accept(self):
        if self.computed_points is None or len(self.computed_points) == 0:
            print("No computed points available to accept.")
            return
        
        computed_points = self.compute_points(len(self.sector_points))
        # Pass the is_wrapped_sector flag to applyBezierCurve
        self.parent_tuner.applyBezierCurve(computed_points)