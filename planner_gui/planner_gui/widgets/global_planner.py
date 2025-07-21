from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QFormLayout, QLineEdit, QScrollArea,
                             QComboBox, QSpacerItem, QSizePolicy, QFrame, QSlider)
from PyQt5.QtGui import QPixmap, QImage, QFont
from PyQt5.QtCore import Qt, pyqtSignal
import sip
from functools import partial
import yaml
import numpy as np
from PIL import Image
from typing import Any
from pathlib import Path
import cv2 as cv

from global_planner.wrapper import GlobalTrajectoryOptimizer
from widgets.planners import MinimumCurvatureWidget, MinimumTimeWidget, CenterlineWidget

from styles import *

class GlobalPlannerWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignTop)  # Align all widgets to the top

        # Internal parameters
        self.map_path = ""
        self.opt_alg = ""
        self.map_image = None
        self.map_image_edit = None

        # Map display
        self.map_label = QLabel()
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setMinimumSize(400, 300)  # Set a minimum size for the map
        self.map_label.setStyleSheet("background-color: lightgray;")
        self.map_label.setText("Map will be displayed here")
        main_layout.addWidget(self.map_label)

        # Optimization algorithm dropdown
        algo_layout = QHBoxLayout()
        algo_layout.addWidget(QLabel("Optimization Algorithm:"))
        self.algo_dropdown = QComboBox()
        self.algo_dropdown.setStyleSheet(combobox_stylesheet)
        self.algo_dropdown.addItem("Minimum Curvature")
        self.algo_dropdown.currentTextChanged.connect(self.open_optimization)
        self.algo_dropdown.addItem("Minimum Time")
        self.algo_dropdown.currentTextChanged.connect(self.open_optimization)
        self.algo_dropdown.addItem("Centerline")
        self.algo_dropdown.currentTextChanged.connect(self.open_optimization)
        algo_layout.addWidget(self.algo_dropdown)
        main_layout.addLayout(algo_layout)

        # Buttons layout for Generate and Interpolate
        button_layout = QHBoxLayout()
        self.generate_button = QPushButton("Generate")
        self.generate_button.setStyleSheet(button_stylesheet)
        button_layout.addWidget(self.generate_button)

        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

        # Initialize with Minimum Curvature widget
        self.open_optimization("Minimum Curvature")

        self.centerline_image = None
        self.raceline_image = None
        self.current_opt_widget = None

        self._centerline_selected_point = None

    def set_current_centerline_index(self, index: int):
        self._centerline_selected_point = int(index)
        self.draw_image()

    def set_map(self, map_path: str):
        self.map_path = map_path
        self.open_map()
        self.open_optimization()

    def draw_image(self):
        self.map_image_edit = np.copy(self.map_image)
        if self.centerline_image is not None:
            for i, wp in enumerate(self.centerline_image):
                wp = np.round(wp).astype(np.int64)
                self.map_image_edit = cv.circle(self.map_image_edit, (wp[0], wp[1]), 1, (128, 128, 128), thickness=-1)
            self.map_image_edit = cv.arrowedLine(self.map_image_edit, self.centerline_image[0].astype(np.int64), 
                                                 self.centerline_image[10].astype(np.int64), (0,255,0), 2)

        if self.raceline_image is not None:
            for i, wp in enumerate(self.raceline_image):
                wp = np.round(wp).astype(np.int64)
                self.map_image_edit = cv.circle(self.map_image_edit, (wp[0], wp[1]), 1, (50, 25, 196), thickness=-1)

        if self._centerline_selected_point is not None:
            wp = np.round(self.centerline_image[self._centerline_selected_point]).astype(np.int64)
            self.map_image_edit = cv.circle(self.map_image_edit, wp, 5, (255, 0, 0), thickness=-1)
        self.display_map()

    def display_map(self):
        if self.map_image is not None:
            height, width, _ = self.map_image_edit.shape
            bytes_per_line = 3 * width
            qimage = QImage(self.map_image_edit.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimage)
            self.map_label.setPixmap(pixmap.scaled(self.map_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def open_map(self):
        map_info_file = Path(self.map_path) / f"{Path(self.map_path).stem}.yaml"
        try:
            with open(map_info_file, 'r') as file:
                map_info = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"Map info file not found: {map_info_file}")
            return
        self.map_file_path = Path(self.map_path) / (map_info["planner_image"] if "planner_image" in map_info else map_info["image"])
        self.map_resolution = map_info["resolution"]
        self.map_origin = np.array(map_info["origin"])[:2]

        try:
            # Load image into numpy array
            self.map_image = cv.flip(cv.imread(self.map_file_path, 0), 0)
            self.map_image = self.map_image[..., np.newaxis].repeat(3, axis=-1)
            self.map_image_edit = np.copy(self.map_image)
            self.display_map()
        except Exception as e:
            print(f"Error opening map image: {e}")
            return

    def centerline_receive(self):
        if self.current_opt_widget is not None and self.current_opt_widget.centerline_computed is not None:
            self.centerline_image = self.current_opt_widget.centerline_computed
            self.draw_image()

    def raceline_receive(self):
        if self.current_opt_widget is not None and self.current_opt_widget.raceline_computed is not None:
            self.raceline_image = self.current_opt_widget.raceline_computed
            self.draw_image()

    def open_optimization(self, alg: str = None):
        if self.map_path is None or self.map_path == "":
            return
        algo_name = self.algo_dropdown.currentText() if alg is None else alg

        # Remove the current algorithm widget if it exists
        if hasattr(self, 'algo_widget'):
            self.layout().removeWidget(self.algo_widget)
            self.algo_widget.deleteLater()

        # Create a new QWidget for the algorithm
        self.algo_widget = QWidget()
        algo_layout = QVBoxLayout()
        algo_layout.setAlignment(Qt.AlignTop)  # Align widgets to the top

        if algo_name == "Minimum Curvature":
            min_curv_widget = MinimumCurvatureWidget(self.map_path)
            min_curv_widget.centerlineComputed.connect(self.centerline_receive)
            min_curv_widget.racelineComputed.connect(self.raceline_receive)
            self.current_opt_widget = min_curv_widget
            self.generate_button.clicked.connect(min_curv_widget.generate)
            algo_layout.addWidget(min_curv_widget)
        elif algo_name == "Minimum Time":
            min_time_widget = MinimumTimeWidget(self.map_path)
            min_time_widget.centerlineComputed.connect(self.centerline_receive)
            min_time_widget.racelineComputed.connect(self.raceline_receive)
            self.current_opt_widget = min_time_widget
            self.generate_button.clicked.connect(min_time_widget.generate)
            algo_layout.addWidget(min_time_widget)
        elif algo_name == "Centerline":
            centerline_widget = CenterlineWidget(self.map_path)
            centerline_widget.centerlineComputed.connect(self.centerline_receive)
            centerline_widget.racelineComputed.connect(self.raceline_receive)
            self.current_opt_widget = centerline_widget
            self.generate_button.clicked.connect(centerline_widget.generate)
            algo_layout.addWidget(centerline_widget)
        # Add more conditions here for other optimization algorithms

        self.algo_widget.setLayout(algo_layout)

        # Add the new algorithm widget to the main layout
        main_layout = self.layout()
        main_layout.insertWidget(main_layout.count() - 1, self.algo_widget)

    def update_images(self, image_paths):
        # Clear existing images
        for i in reversed(range(self.image_layout.count())): 
            self.image_layout.itemAt(i).widget().setParent(None)

        # Add new images
        for path in image_paths:
            image_label = QLabel()
            pixmap = QPixmap(path)
            if not pixmap.isNull():
                image_label.setPixmap(pixmap.scaled(200, 150, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            else:
                image_label.setText("Failed to load image")
            self.image_layout.addWidget(image_label)


