from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QFormLayout, QLineEdit, QScrollArea,
                             QComboBox, QSpacerItem, QSizePolicy, QFrame, QSlider)
from PyQt5.QtGui import QPixmap, QImage, QFont
from PyQt5.QtCore import Qt, pyqtSignal
import sip
import yaml
import numpy as np
from PIL import Image
from pathlib import Path
from typing import Any
from functools import partial

from global_planner.wrapper import GlobalTrajectoryOptimizer

from styles import *

class MinimumCurvatureWidget(QWidget):
    centerlineComputed = pyqtSignal()
    racelineComputed = pyqtSignal()

    def __init__(self, map_path: str):
        super().__init__()

        header_font = QFont("Arial", 14, QFont.Bold)

        self.global_optimizer = GlobalTrajectoryOptimizer(map_path)

        # Create a scroll area
        scroll_area = QScrollArea()
        scroll_area.setStyleSheet(scrollarea_stylesheet)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)

        content_widget = QWidget()
        
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)

        form_layout = QFormLayout()

        # Add parameters from MinCurveOptimizer
        self.param_widgets = {}
        
        self.setStyleSheet(tab_stylesheet)


        self._set_parameter("centerline_length", form_layout, lambda x: float(x))
        self._set_parameter("safety_width", form_layout, lambda x: float(x))
        self._set_parameter("occ_grid_threshold", form_layout, lambda x: float(x))
        self._set_parameter("map_editor_mode", form_layout, lambda x: x.lower()=="true")
        self._set_parameter("recompute_velocity", form_layout, lambda x: x.lower()=="true")
        self._set_parameter("border_normal_dist", form_layout, lambda x: x.lower()=="true")
        self._set_parameter("max_iters", form_layout, lambda x: int(x))
        self._set_parameter("curvature_limit", form_layout, lambda x: float(x))


        param_name = "init_pose_ind"
        label = QLabel(param_name)
        label.setStyleSheet(container_label_stylesheet)
        self.init_pose_slider = QSlider(Qt.Horizontal)
        self.init_pose_slider.setMinimum(0)
        self.init_pose_slider.setMaximum(0)
        self.init_pose_slider.setValue(int(self.global_optimizer.parameters[param_name]))
        self.init_pose_slider.setTickPosition(QSlider.TicksBelow)
        self.init_pose_slider.setTickInterval(1)
        self.init_pose_slider.setStyleSheet(slider_stylesheet)
        # Create a QLabel to display the current value
        value_label = QLabel(str(self.init_pose_slider.value()))
        value_label.setStyleSheet(container_label_stylesheet)
        # Connect the slider's valueChanged signal to update the parameter and the value label
        self.init_pose_slider.valueChanged.connect(lambda value: self.set_parameter(value, param_name, int))
        self.init_pose_slider.valueChanged.connect(lambda value: value_label.setText(str(value)))
        slider_layout = QHBoxLayout()
        slider_layout.addWidget(self.init_pose_slider)
        slider_layout.addWidget(value_label)
        form_layout.addRow(label, slider_layout)
        self.param_widgets[param_name] = self.init_pose_slider

        self._set_parameter("reverse_mapping", form_layout, lambda x: x.lower()=="true")
        self._set_parameter("show_plots", form_layout, lambda x: x.lower()=="true")
        self._set_parameter("config_path", form_layout, lambda x: str(x))

        layout.addLayout(form_layout)

        layout.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Fixed))
        # Skeletonization
        skeleton_layout = QHBoxLayout()
        self.skeleton_label = QLabel("Skeletonization")
        self.skeleton_label.setStyleSheet(heading_label_stylesheet)
        skeleton_layout.addWidget(self.skeleton_label)
        
        skeleton_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        self.skeletonize_button = QPushButton("Generate")
        self.skeletonize_button.setStyleSheet(button_stylesheet)
        self.skeletonize_button.clicked.connect(self.generate_skeleton)
        skeleton_layout.addWidget(self.skeletonize_button)
        
        layout.addLayout(skeleton_layout)

        self.skeleton_image = QLabel("Skeleton not computed yet")
        self.skeleton_image.setStyleSheet(container_label_stylesheet)
        self.skeleton_image.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        layout.addWidget(self.skeleton_image)

        # Centerline
        centerline_layout = QHBoxLayout()
        self.centerline_label = QLabel("Centerline")
        self.centerline_label.setStyleSheet(heading_label_stylesheet)
        centerline_layout.addWidget(self.centerline_label)
        centerline_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        self.centerline_button = QPushButton("Generate")
        self.centerline_button.setStyleSheet(button_stylesheet)
        self.centerline_button.clicked.connect(self.generate_centerline)
        centerline_layout.addWidget(self.centerline_button)
        
        layout.addLayout(centerline_layout)

        self.centerline_image = QLabel("Centerline not computed yet")
        self.centerline_image.setStyleSheet(container_label_stylesheet)
        self.centerline_image.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        layout.addWidget(self.centerline_image)

        # Track Boundaries
        tb_layout = QHBoxLayout()
        self.tb_label = QLabel("Track Boundaries")
        self.tb_label.setStyleSheet(heading_label_stylesheet)
        tb_layout.addWidget(self.tb_label)
        tb_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        self.tb_button = QPushButton("Generate")
        self.tb_button.setStyleSheet(button_stylesheet)
        self.tb_button.clicked.connect(self.generate_track_boundaries)
        tb_layout.addWidget(self.tb_button)
        
        layout.addLayout(tb_layout)

        self.tb_image = QLabel("Track boundaries not computed yet")
        self.tb_image.setStyleSheet(container_label_stylesheet)
        self.tb_image.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        layout.addWidget(self.tb_image)

        # Global Trajectory
        glb_traj_layout = QHBoxLayout()
        self.glb_traj_label = QLabel("Global Trajectory")
        self.glb_traj_label.setStyleSheet(heading_label_stylesheet)
        glb_traj_layout.addWidget(self.glb_traj_label)
        glb_traj_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        
        self.glb_traj_button = QPushButton("Generate")
        self.glb_traj_button.setStyleSheet(button_stylesheet)
        self.glb_traj_button.clicked.connect(self.generate_global_trajectory)
        glb_traj_layout.addWidget(self.glb_traj_button)
        
        layout.addLayout(glb_traj_layout)

        self.glb_traj_image = QLabel("Global trajectory not computed yet")
        self.glb_traj_image.setStyleSheet(container_label_stylesheet)
        self.glb_traj_image.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        layout.addWidget(self.glb_traj_image)

        layout.addStretch(1)

        content_widget.setLayout(layout)
        content_widget.setAutoFillBackground(True)
        palette = content_widget.palette()
        palette.setColor(content_widget.backgroundRole(), Qt.white)
        content_widget.setPalette(palette)

        scroll_area.setWidget(content_widget)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(scroll_area)
        self.setLayout(main_layout)

        # Set the widget to expand to fill available space
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.centerline_computed = []
        self.raceline_computed = []

    def _set_parameter(self, name, layout, parser=None):
        label = QLabel(name)
        label.setStyleSheet(container_label_stylesheet)
        line_edit = QLineEdit(str(self.global_optimizer.parameters[name]))
        line_edit.setReadOnly(False)  # Make it read-only for now
        line_edit.textChanged.connect(partial(self.set_parameter, name=name, parser=parser))
        line_edit.setStyleSheet(line_edit_stylesheet)
        layout.addRow(label, line_edit)
        self.param_widgets[name] = line_edit

    def generate(self):
        if not self.generate_skeleton():
            return False
        if not self.generate_centerline():
            return False
        if not self.generate_track_boundaries():
            return False
        if not self.generate_global_trajectory():
            return False

    def generate_skeleton(self):
        if not self.global_optimizer.skeletonize():
            self.skeleton_image.setText("Failed to compute skeleton")
            self.skeleton_image.setStyleSheet("color: red;")
            return False
        
        skeleton_vis = self.global_optimizer.skeleton_vis

        height, width, channel = skeleton_vis.shape
        bytes_per_line = 3 * width
        qimage = QImage(skeleton_vis.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)

        # Set a fixed size for the QLabel
        desired_width = 500  # You can adjust this value
        aspect_ratio = height / width
        desired_height = int(desired_width * aspect_ratio)

        self.skeleton_image.setFixedSize(desired_width, desired_height)
        self.skeleton_image.setPixmap(pixmap.scaled(desired_width, desired_height, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.skeleton_image.setAlignment(Qt.AlignCenter)
        
        # Allow the label to expand if needed
        self.skeleton_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        return True
    
    def generate_centerline(self):
        if not self.global_optimizer.define_centerline():
            self.centerline_image.setText("Failed to compute centerline")
            self.centerline_image.setStyleSheet("color: red;")
            return False
        
        centerline_vis = self.global_optimizer.centerline_vis

        height, width, channel = centerline_vis.shape
        bytes_per_line = 3 * width
        qimage = QImage(centerline_vis.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)

        # Set a fixed size for the QLabel
        desired_width = 500  # You can adjust this value
        aspect_ratio = height / width
        desired_height = int(desired_width * aspect_ratio)

        self.centerline_image.setFixedSize(desired_width, desired_height)
        self.centerline_image.setPixmap(pixmap.scaled(desired_width, desired_height, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.centerline_image.setAlignment(Qt.AlignCenter)
        
        # Allow the label to expand if needed
        self.centerline_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.centerline_computed = self.global_optimizer.centerline_smooth
        self.init_pose_slider.setMinimum(0)
        self.init_pose_slider.setMaximum(self.centerline_computed.shape[0]-1)
        self.centerlineComputed.emit()
        return True
    
    def generate_track_boundaries(self):
        if not self.global_optimizer.extract_track_bounds():
            self.tb_image.setText("Failed to compute track boundaries")
            self.tb_image.setStyleSheet("color: red;")
            return False
        
        track_bounds_vis = self.global_optimizer.track_bounds_vis


        height, width, channel = track_bounds_vis.shape
        bytes_per_line = 3 * width
        qimage = QImage(track_bounds_vis.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)

        # Set a fixed size for the QLabel
        desired_width = 500  # You can adjust this value
        aspect_ratio = height / width
        desired_height = int(desired_width * aspect_ratio)

        self.tb_image.setFixedSize(desired_width, desired_height)
        self.tb_image.setPixmap(pixmap.scaled(desired_width, desired_height, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.tb_image.setAlignment(Qt.AlignCenter)
        
        # Allow the label to expand if needed
        self.tb_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        return True
    
    def generate_global_trajectory(self):
        if not self.global_optimizer.trajectory_optimization():
            self.glb_traj_image.setText("Failed to compute track boundaries")
            self.glb_traj_image.setStyleSheet("color: red;")
            return False
        
        glb_traj_vis = self.global_optimizer.glb_traj_vis
        self.raceline_computed = self.global_optimizer.global_trajectory_pixel
        
        height, width, channel = glb_traj_vis.shape
        bytes_per_line = 3 * width
        qimage = QImage(glb_traj_vis.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)

        # Set a fixed size for the QLabel
        desired_width = 800  # You can adjust this value
        aspect_ratio = height / width
        desired_height = int(desired_width * aspect_ratio)

        self.glb_traj_image.setFixedSize(desired_width, desired_height)
        self.glb_traj_image.setPixmap(pixmap.scaled(desired_width, desired_height, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.glb_traj_image.setAlignment(Qt.AlignCenter)
        
        # Allow the label to expand if needed
        self.glb_traj_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.racelineComputed.emit()
        return True

    def update_params(self):
        """Update the displayed parameters if they change in the optimizer"""
        for param_name, widget in self.param_widgets.items():
            dtype = type(self.global_optimizer.parameters[param_name])
            self.global_optimizer.parameters[param_name] = type(widget.text())

    def set_parameter(self, value: Any, name: str, parser=lambda x:x):
        if value is None or value == "":
            return
        if name in self.global_optimizer.parameters:
            try:
                self.global_optimizer.parameters[name] = parser(value)
                print(f"Set parameter {name} to {self.global_optimizer.parameters[name]}")
            except Exception as e:
                print(f"Failed to set parameter {name}: {str(e)}")

