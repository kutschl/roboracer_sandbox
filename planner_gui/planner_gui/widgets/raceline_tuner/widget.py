from PyQt5.QtWidgets import (
    QWidget, QGraphicsScene, QGraphicsView, QApplication, QGridLayout,
    QGraphicsEllipseItem, QGraphicsItem, QTableWidget, QHeaderView, QTableWidgetItem,
    QHBoxLayout, QPushButton, QVBoxLayout, QTabWidget, QLabel, QComboBox, QCheckBox,
    QFileDialog, QMessageBox, QSlider, QGraphicsPixmapItem, QInputDialog, QGraphicsTextItem,
    QGraphicsRectItem
)
from PyQt5.QtGui import QPen, QBrush, QColor, QPolygonF, QPainterPath, QPainter, QPixmap, QTransform, QFont
from PyQt5.QtCore import QPointF, Qt, pyqtSlot, QLineF, QRectF, pyqtSignal, QTimer
import yaml
import pyqtgraph as pg
import sip
from functools import partial
import yaml
import numpy as np
from PIL import Image
from typing import Any
from pathlib import Path
import cv2 as cv
import datetime
import os

from styles import *
from widgets.raceline_tuner.tools import BezierCurveWidget, VelocityScalingWidget, SmoothingWidget, SectorTableWidget
from widgets.raceline_tuner.structs import RacelinePoint, ControlPoint

from global_planner.utils import load_waypoints, array_to_trackbounds

from global_planner.raceline import RacelineContainer

import rclpy

import time

class ToggleableQTableWidget(QTableWidget):
    def mousePressEvent(self, event):
        index = self.indexAt(event.pos())
        if index.isValid():
            if self.selectionModel().isSelected(index):
                self.clearSelection()
                # Do not call the base event if deselecting.
                return
        super().mousePressEvent(event)

class RacelineTuner(QWidget):

    sectorChanged = pyqtSignal()

    def __init__(self, map_path: str, node=None):
        super().__init__()
        self.ros_interface = node
        self.map_path = map_path
        self.draw_scale = 90

        self.map_image = None
        self.map_meta_data = None
        self.map_img_path = None
        self._raceline_cache = {}
        self._trackbound_cache = {}


        self.__dividerStr = "────────────────────"

        self.raceline = None
        self.current_sector = None
        self.raceline_points = []
        self.selecting_sector = False
        self.selected_sector_points = []
        self.sectors = []
        self.initUI()
        #import ipdb; ipdb.set_trace()
        self.set_map(self.map_path)

        #self.activate_ros_interface()
        #self.update_scene()
        if self.ros_interface is not None:
            self.initialize_ros_interface()
        self.sectorChanged.emit()


    def initUI(self):
        self.main_layout = QGridLayout(self)
        self.setLayout(self.main_layout)
        self.setStyleSheet(tab_stylesheet)


        # --- Create two plot widgets for velocity and steering ---
        self.vx_plot_widget = pg.GraphicsLayoutWidget()
        self.vx_plot_widget.setStyleSheet(plot_stylesheet)
        self.vx_plot_widget.setBackground(container_bg_color)

        self.vx_plot = self.vx_plot_widget.addPlot()
        self.vx_plot.setLabel('bottom', 'Track Length [m]')
        self.vx_plot.setLabel('left', 'Speed [m/s]')
        self.vx_plot.showGrid(x=True, y=True)
        self.vx_plot_ref_line = self.vx_plot.plot([], [], pen=pg.mkPen(color=ref_line_color, width=1))
        self.vx_plot_curr_lap = self.vx_plot.plot([], [], pen=pg.mkPen(color=curr_lap_color, width=2))
        self.vx_plot_change = self.vx_plot.plot([], [], pen=pg.mkPen(color=change_plot_color, width=2))
        # New red line for scaled velocity profile
        self.vx_plot_scaled = self.vx_plot.plot([], [], pen=pg.mkPen('r', width=2))

        self.ax_plot_widget = pg.GraphicsLayoutWidget()
        self.ax_plot_widget.setStyleSheet(plot_stylesheet)
        self.ax_plot_widget.setBackground(container_bg_color)

        self.ax_plot = self.ax_plot_widget.addPlot()
        self.ax_plot.setLabel('bottom', 'Track Length [m]')
        self.ax_plot.setLabel('left', 'Acc. [m/s^2]')
        self.ax_plot.showGrid(x=True, y=True)
        self.ax_plot_ref_line = self.ax_plot.plot([], [], pen=pg.mkPen(color=ref_line_color, width=1))
        self.ax_plot_curr_lap = self.ax_plot.plot([], [], pen=pg.mkPen(color=curr_lap_color, width=2))
        # New red line for scaled velocity profile
        self.ax_plot_scaled = self.ax_plot.plot([], [], pen=pg.mkPen('r', width=2))

        self.sa_plot_widget = pg.GraphicsLayoutWidget()
        self.sa_plot_widget.setStyleSheet(plot_stylesheet)
        self.sa_plot_widget.setBackground(container_bg_color)

        self.sa_plot = self.sa_plot_widget.addPlot()
        self.sa_plot.setLabel('bottom', 'Track Length [m]')
        self.sa_plot.setLabel('left', 'Steering Angle [rad]')
        self.sa_plot.showGrid(x=True, y=True)
        self.sa_plot_ref_line = self.sa_plot.plot([], [], pen=pg.mkPen(color=ref_line_color, width=1))
        self.sa_plot_curr_lap = self.sa_plot.plot([], [], pen=pg.mkPen(color=curr_lap_color, width=2))

        # --- Create the sector table using the toggleable version ---
        # Modified: now 8 columns including L1 Min and L1 Max.
        self.sector_table = ToggleableQTableWidget(0, 3)
        self.sector_table.setHorizontalHeaderLabels(["ID", "Start", "End"])
        self.sector_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.sector_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.sector_table.setSelectionMode(QTableWidget.SingleSelection)
        self.sector_table.setStyleSheet("""
            QTableWidget {
                background-color: rgb(30,30,30);
                color: white;
                font-size: 10pt;
            }
            QTableWidget::item {
                border: none;
                padding: 4px;
            }
            QTableWidget::item:selected {
                background-color: rgb(50,50,50);
                color: rgb(250,250,250);
            }
            QHeaderView::section {
                background-color: rgb(30,30,30);
                color: white;
                padding: 4px;
                border: none;
            }
        """)
        self.sector_table.itemChanged.connect(self.sector_table_item_changed)
        self.sector_table.itemSelectionChanged.connect(self.sector_table_selection_changed)

        # --- Arrange the left container: speed plot, global speed scale, then steering plot ---
        self.left_container = QWidget(self)
        left_layout = QVBoxLayout(self.left_container)
        left_layout.addWidget(self.vx_plot_widget)
        left_layout.addWidget(self.ax_plot_widget)
        left_layout.addWidget(self.sa_plot_widget)
        self.left_container.setLayout(left_layout)

        self.scene = QGraphicsScene(self)
        self.scene.setBackgroundBrush(QBrush(QColor(*container_bg_color)))
        self.view = QGraphicsView(self.scene, self)
        self.view.setRenderHints(self.view.renderHints())

        self.tool_bar = QWidget(self)
        tool_layout = QHBoxLayout(self.tool_bar)
        self.tool_bar.setLayout(tool_layout)

        self.select_button = QPushButton("Select", self.tool_bar)
        self.select_button.setStyleSheet(button_stylesheet)
        self.select_button.clicked.connect(self.start_sector_selection)
        tool_layout.addWidget(self.select_button)

        self.racelineComboBox = QComboBox(self.tool_bar)
        self.racelineComboBox.setStyleSheet(combobox_stylesheet)
        self.racelineComboBox.currentIndexChanged.connect(self.racelineChanged)
        tool_layout.addWidget(self.racelineComboBox)


        self.velocity_checkbox = QCheckBox("Velocity Profile", self.tool_bar)
        self.velocity_checkbox.setStyleSheet(checkbox_stylesheet)
        self.velocity_checkbox.stateChanged.connect(self.toggleVelocityProfile)
        self.velocity_checkbox.setChecked(True)
        tool_layout.addWidget(self.velocity_checkbox)

        self.use_ros_checkbox = QCheckBox("ROS Active", self.tool_bar)
        self.use_ros_checkbox.setStyleSheet(checkbox_stylesheet)
        #self.use_ros_checkbox.stateChanged.connect(self.toggleROSInterface)
        self.use_ros_checkbox.setCheckable(False)
        self.use_ros_checkbox.setChecked(self.ros_interface is not None)
        tool_layout.addWidget(self.use_ros_checkbox)

        self.deploy_button = QPushButton("Deploy", self.tool_bar)
        self.deploy_button.setStyleSheet(button_stylesheet)
        self.deploy_button.clicked.connect(lambda *_: self.deploy())
        tool_layout.addWidget(self.deploy_button)

        self.saveas_button = QPushButton("Save as", self.tool_bar)
        self.saveas_button.setStyleSheet(button_stylesheet)
        self.saveas_button.clicked.connect(lambda *_: self.saveas())
        tool_layout.addWidget(self.saveas_button)

        self.save_button = QPushButton("Save", self.tool_bar)
        self.save_button.setStyleSheet(button_stylesheet)
        self.save_button.clicked.connect(lambda *_: self.save())
        tool_layout.addWidget(self.save_button)

        self.view_container = QWidget(self)
        view_vlayout = QVBoxLayout(self.view_container)
        view_vlayout.addWidget(self.tool_bar)
        view_vlayout.addWidget(self.view)
        self.view_container.setLayout(view_vlayout)


        self.toolbox = QTabWidget(self)
        self.toolbox.setStyleSheet(data_stylesheet)
        self.toolbox.setMinimumHeight(150)
        self.bezier_widget = BezierCurveWidget(self)
        self.sectorChanged.connect(self.bezier_widget.changeSector)
        self.velocity_scaling_widget = VelocityScalingWidget(self)
        #self.smoothing_widget = SmoothingWidget(self)
        self.toolbox.addTab(self.velocity_scaling_widget, "Velocity Scaling")
        self.toolbox.addTab(self.bezier_widget, "Bezier Curve")

        self.sector_table_widget = SectorTableWidget(self)
        self.toolbox.addTab(self.sector_table_widget, "Sectors")
        #self.toolbox.addTab(self.smoothing_widget, "Smoothing")
        self.toolbox.currentChanged.connect(self.onTabChanged)

        # --- Layout: left container at top left, sector table at bottom left ---
        self.main_layout.addWidget(self.left_container, 0, 0)
        self.main_layout.addWidget(self.view_container, 0, 1)
        self.main_layout.addWidget(self.sector_table, 1, 0)
        self.main_layout.addWidget(self.toolbox, 1, 1)
        self.main_layout.setRowStretch(0, 5)
        self.main_layout.setRowStretch(1, 3)
        self.main_layout.setColumnStretch(0, 1)
        self.main_layout.setColumnStretch(1, 3)

    #################################################
    ##################### Scene #####################
    #################################################

    def update_scene(self):
        self.clear_scene()

        pix = QPixmap(self.map_img_path)
        #import ipdb; ipdb.set_trace()
        transform = QTransform().scale(1, -1)
        pix = pix.transformed(transform)  
        self.map_pixmap_item = QGraphicsPixmapItem(pix)
        self.map_pixmap_item.setZValue(-1)  
        self.scene.addItem(self.map_pixmap_item)
        self.scene.setSceneRect(self.map_pixmap_item.boundingRect())  # so view bounds match

        if self.current_raceline == '':
            print("No raceline found to visualize!")
            return

        raceline = self.raceline()
        #tb_left, tb_right = self._trackbound_cache[self.current_raceline]
        #scale = self.draw_scale
        #import ipdb; ipdb.set_trace()
        #tb_left, tb_right = self.raceline.inverse_trackbounds
        tb_left, tb_right = self.raceline.trackbounds
        self._print_tb(self.scene, tb_left)
        self._print_tb(self.scene, tb_right)

        path = QPainterPath()
        pixel = self.point_to_pixel_coords(raceline[0, 1:3])
        path.moveTo(pixel[0], pixel[1])
        for point in raceline[1:]:
            pixel = self.point_to_pixel_coords(point[1:3])
            path.lineTo(pixel[0], pixel[1])
        pen = QPen(QColor(200, 200, 200, 200), 2)
        self.scene.addPath(path, pen)
        for i, point in enumerate(raceline):
            pixel = self.point_to_pixel_coords(point[1:3])
            ellipse_item = RacelinePoint(pixel[0], pixel[1], i, radius=3, tuner=self)
            self.scene.addItem(ellipse_item)
            self.raceline_points.append(ellipse_item)

        self.add_arclength_labels()

        if self.current_sector is not None:
            self.highlight_sector(self.current_sector)

        if self.velocity_checkbox.isChecked():
            self.toggleVelocityProfile(Qt.Checked)

        if self.raceline_points:
            # unite all individual point bounding rects
            rect = None
            for pt in self.raceline_points:
                br = pt.sceneBoundingRect()
                rect = br if rect is None else rect.united(br)

            # add pixel padding around the raceline
            padding = 100  # adjust as needed
            rect.adjust(-padding, -padding, padding, padding)

            # fit the view to this rect, keeping aspect ratio
            self.view.fitInView(rect, Qt.KeepAspectRatio)

            # optional: disable scrollbars now that it’s zoomed
            self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

    def toggleVelocityProfile(self, state):
        if self.current_raceline == '':
            return
        raceline = self.raceline()
        if state == Qt.Checked:
            velocities = raceline[:, 7].astype(float)
            v_min = np.min(velocities)
            v_max = np.max(velocities)
            for i, pt in enumerate(self.raceline_points):
                v = float(raceline[i, 7])
                norm = (v - v_min) / (v_max - v_min) if (v_max - v_min) > 0 else 0
                g = int(255 * (1 - norm))
                b = int(255 * (1 - norm))
                pt.setBrush(QBrush(QColor(255, g, b)))
        else:
            for pt in self.raceline_points:
                pt.setBrush(QBrush(QColor(255, 255, 255)))

    def toggleROSInterface(self, state):
        if state == Qt.Checked:
            self.activate_ros_interface()
        else:
            self.deactivate_ros_interface()

    def _print_tb(self, scene, bound: np.ndarray, scale: float = None):
        pen = QPen(QColor(150, 150, 150), 1)
        #scale = scale or self.draw_scale
        path = QPainterPath()
        for i, tb in enumerate(bound):
            pixel = self.point_to_pixel_coords(tb)
            point = QPointF(pixel[0], pixel[1])
            if i == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)
        scene.addPath(path, pen)
        
    def clear_scene(self):
        self.scene.clear()
        self.bezier_raceline_items = []
        self.raceline_points = []

    def highlight_sector(self, sector):
        start_idx, end_idx = sector
        for pt in self.raceline_points:
            if self.point_in_sector(pt.id, start_idx, end_idx):
                pt.setOpacity(1.0)
            else:
                pt.setOpacity(0.3)

    def mark_point(self, index):
        """
        Highlights the raceline point corresponding to the given index.
        Here, we set the selected point’s color to red and reset others to white.
        """
        for pt in self.raceline_points:
            if pt.id == index:
                pt.setBrush(QBrush(QColor(255, 0, 0)))  # Mark the selected point in red.
            else:
                pt.setBrush(QBrush(QColor(255, 255, 255)))  # Reset others to white.

    def point_in_sector(self, pt_id, start_idx, end_idx):
        # This function already handles the wrapping case (when start_idx > end_idx)
        if start_idx <= end_idx:
            return start_idx <= pt_id <= end_idx
        else:
            return pt_id >= start_idx or pt_id <= end_idx
        
    def set_sector_points_movable(self, movable: bool):
        if self.current_sector is None:
            return
        start_idx, end_idx = self.current_sector
        for pt in self.raceline_points:
            if self.point_in_sector(pt.id, start_idx, end_idx):
                pt.setMovable(movable)
            else:
                pt.setMovable(False)

    def point_to_pixel_coords(self, point: np.ndarray):

        pixel = point - np.array(self.map_meta_data["origin"][:-1])
        pixel /= self.map_meta_data["resolution"]
        return pixel
    
    def pixel_to_point_coords(self, pixel: np.ndarray):

        point = pixel * self.map_meta_data["resolution"] + np.array(self.map_meta_data["origin"][:-1])
        return point
    
    def updateBezierRacelinePoints(self, new_points: np.ndarray):
        # Only update if Bezier Curve tab is selected
        for item in self.bezier_raceline_items:
            self.scene.removeItem(item)
        self.bezier_raceline_items = []
        for pt in new_points:
            item = QGraphicsEllipseItem(-3, -3, 6, 6)
            item.setPos(pt[0], pt[1])
            item.setBrush(QBrush(QColor(0, 0, 255)))  # blue
            item.setPen(QPen(QColor(0, 0, 0)))
            item.setZValue(2)
            self.scene.addItem(item)
            self.bezier_raceline_items.append(item)
        for pt in self.raceline_points:
            pt.setOpacity(0.3)

    def applyBezierCurve(self, bezier_points: np.ndarray):
        if self.current_sector is None:
            print("No current sector selected.")
            return
        if self.raceline is None:
            print("Currently no raceline is loaded.")
            return
        points = self.pixel_to_point_coords(bezier_points)
        start_idx, end_idx = self.current_sector
        self.raceline.set_position(points, start_idx, end_idx)
        self.save()
        self.update_scene()

    def add_arclength_labels(self):
        """
        Add labels showing arclength (s) values at regular intervals along the raceline.
        """
        # Get arclength values from raceline
        raceline = self.raceline()
        arc_lengths = raceline[:, self.raceline.S_INDEX]
        total_length = arc_lengths[-1]

        d_left = raceline[:, self.raceline.D_LEFT_INDEX]
        d_right = raceline[:, self.raceline.D_RIGHT_INDEX]

        left_larger = d_left > d_right

        left_normals, right_normals = self.raceline.normals
        # Determine label interval (place ~10-15 labels along the track)
        interval = max(1.0, round(total_length / 10, 0))
        # Create labels at regular intervals
        for s_target in np.arange(0, total_length, interval):
            # Find closest point on raceline to desired arclength
            idx = np.argmin(np.abs(arc_lengths - s_target))

            if left_larger[idx] == True:
                point = raceline[idx, 1:3] + left_normals[idx] * 0.7
            else:
                point = raceline[idx, 1:3] + right_normals[idx] * 0.7
            s_value = arc_lengths[idx]
            
            # Convert to pixel coordinates
            pixel = self.point_to_pixel_coords(point)
            
            # Create label with background box
            text_item = QGraphicsTextItem(f"{s_value:.0f}m")
            text_item.setPos(pixel[0], pixel[1])  # Offset from point
            text_item.setDefaultTextColor(QColor(0, 0, 0))
            text_item.setFont(QFont("NotoMono", 8))
            
            # Add marker dot to show exact position
            pixel = self.point_to_pixel_coords(raceline[idx,1:3])
            marker = QGraphicsEllipseItem(-3, -3, 6, 6)
            marker.setBrush(QBrush(QColor(255, 255, 0, 100)))  # Yellow dot
            marker.setPen(QPen(QColor(0, 0, 0, 100)))
            marker.setPos(pixel[0], pixel[1])
            marker.setZValue(2)
            
            # Add items to scene (background first, then text)
            self.scene.addItem(text_item)
            self.scene.addItem(marker)

    #################################################
    ################### Racelines ###################
    #################################################

    def set_map(self, map_path: str):

        self.map_path = map_path

        meta_path = Path(self.map_path) / f"{str(Path(self.map_path).stem)}.yaml"
        with open(meta_path, 'r') as f:
            meta_data = yaml.load(f)

        self.map_meta_data = meta_data
        map_img_path = meta_data["planner_image"]
        self.map_img_path = str(Path(self.map_path) / map_img_path)
        self.map_image = np.asarray(Image.open(self.map_img_path))
        
        self.load_raceline()

    
    def load_raceline(self, selected: str = None):
        raceline_path = Path(self.map_path) / "plans"
        self._raceline_cache = {}
        self._trackbound_cache = {}

        normal_files = []
        timestamp_files = []

        # Separate files into normal names and timestamp-based names
        for p in raceline_path.iterdir():
            if p.is_file() and p.suffix == ".csv" and str(p.stem) != 'iqp_centerline':
                raceline_name = str(p.stem)
                raceline = load_waypoints(str(p))
                left_tb, right_tb = array_to_trackbounds(raceline, True)
                self._trackbound_cache[raceline_name] = (left_tb, right_tb)
                self._raceline_cache[raceline_name] = raceline

                # Check if the file name is a timestamp
                try:
                    datetime.datetime.strptime(raceline_name, "%d_%m_%Y_%H_%M_%S")
                    timestamp_files.append(raceline_name)
                except ValueError:
                    normal_files.append(raceline_name)

        # Sort the files
        normal_files.sort()  # Alphabetical order
        timestamp_files.sort(reverse=True)  # Latest to earliest

        # Clear and populate the combobox
        self.racelineComboBox.clear()

        # Add normal files
        self.racelineComboBox.addItems(normal_files)

        # Add a divider
        if normal_files and timestamp_files:
            self.racelineComboBox.addItem(self.__dividerStr)  # Divider

        # Add timestamp-based files
        self.racelineComboBox.addItems(timestamp_files)

        # Add the "ROS" option
        self.racelineComboBox.addItem("ROS")

        # Select the default raceline
        if selected is None and 'min_curve' in self._raceline_cache.keys():
            selected = "min_curve"

        num_racelines = len(self._raceline_cache)
        if selected is None:
            self.racelineComboBox.setCurrentIndex(0)
        else:
            if selected in self._raceline_cache.keys():
                self.racelineComboBox.setCurrentText(selected)
            else:
                print(f"Warning: {selected} not found in raceline cache. Defaulting to first raceline.")
                self.racelineComboBox.setCurrentIndex(0)

    def racelineChanged(self):

        curr_raceline = self.current_raceline
        print(f"Selected raceline: {curr_raceline}")
        if curr_raceline == '':
            return
        elif curr_raceline == self.__dividerStr:
            if 'min_curve' in self._raceline_cache.keys():
                curr_raceline = "min_curve"
                self.racelineComboBox.setCurrentText("min_curve")
            else:
                return
        elif curr_raceline == "ROS":
            if self.ros_interface is None:
                self.activate_ros_interface()
            self.update_raceline_from_ros()
        else:
            #if self.ros_interface is not None:
            #    self.deactivate_ros_interface()
            self.raceline = RacelineContainer.from_array(self._raceline_cache[curr_raceline])

        sector_file = Path(self.map_path) / "plans" / f"{curr_raceline}_sectors.json"
        #import ipdb; ipdb.set_trace()
        if os.path.exists(sector_file):
            self.raceline.set_sectors_from_file(sector_file)
            self.sector_table_widget.update_table(self.raceline.sectors)

        self.update_scene()
        self.update_plots()
        self.velocity_scaling_widget.clear_plot()
        self.clear_sectors()
        self.clear_changed_vx_plot()
        self.bezier_widget.reset()

    def save(self, path: str = None):
        if path is None:
            file_name = datetime.datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
            path = str(Path(self.map_path) / "plans" / f"{file_name}.csv")
        self.raceline.save(path)
        curr_raceline = str(Path(path).stem)

        if self.current_raceline != "ROS":
            self.load_raceline(curr_raceline)

    def deploy(self):
        if self.ros_interface is None:
            print("Can only deploy when connected to ROS!")
            return

        self.publish_raceline()

    def saveas(self):
        # Prompt the user for a filename
        name, ok = QInputDialog.getText(
            self,
            'Save Raceline As',
            'Enter raceline filename:'
        )
        if ok and name:
            # Ensure the filename ends with .csv
            filename = name if name.lower().endswith('.csv') else f"{name}.csv"
            # Construct full save path
            save_path = Path(self.map_path) / 'plans' / filename
            # Delegate to the existing save() method
            self.save(str(save_path))
    
    @property
    def current_raceline(self):
        return self.racelineComboBox.currentText()
    
    #################################################
    ################# ROS Interface##################
    #################################################

    def initialize_ros_interface(self):
        if self.ros_interface is None:
            return
        from f110_msgs.msg import WpntArray, SectorArray

        self.ros_interface.subscribe('/global_waypoints', WpntArray)
        self.ros_interface.publisher('/global_waypoints/set', WpntArray)
        self.ros_interface.publisher('/sectors/set', SectorArray)
        print("ROS interface initialized on RacelineTuner!")

    def destroy_ros_interface(self):
        if self.ros_interface is None: return

        self.ros_interface.unsubscribe('/global_waypoints')
        self.ros_interface.delete_publisher('/global_waypoints/set')
        self.ros_interface.delete_publisher('/sectors/set')


    def publish_raceline(self):
        if self.ros_interface is None:
            print("ROS interface not activated.")
            return
        print("Publishing raceline...")
        msg = self.raceline.to_wpnt_msg()
        wpnt_name = datetime.datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        msg.name = wpnt_name
        self.ros_interface.publish('/global_waypoints/set', msg)

        if self.raceline.has_sectors:
            sector_msg = self.raceline.to_sector_msg()
            self.ros_interface.publish('/sectors/set', sector_msg)

    
    #################################################
    ##################### Plots #####################
    #################################################
    def update_plots(self):
        if self.raceline is None:
            return
        try:
            # Update speed and steering plots
            raceline = self.raceline()
            arc_lengths = raceline[:, self.raceline.S_INDEX]
            velocity = raceline[:, self.raceline.VX_INDEX]
            acc = raceline[:, self.raceline.AX_INDEX]
            #self.vx_plot_curr_lap.setData(arc_lengths, self.node.speed[1])
            self.vx_plot_ref_line.setData(arc_lengths, velocity)
            self.vx_plot.setYRange(0., 12.)

            self.ax_plot_ref_line.setData(arc_lengths, acc)
            self.ax_plot.setYRange(-5, 5)

            #self.sa_plot_curr_lap.setData(self.node.steering_angle[0], self.node.steering_angle[1])
            psi = raceline[:, self.raceline.PSI_INDEX]
            self.sa_plot_ref_line.setData(arc_lengths, psi)
            self.sa_plot.setYRange(-np.pi/2, np.pi/2)

        except Exception as e:
            return
            #print("Error updating plots:", e)

    def set_changed_vx_plot(self, arc_lengths, velocity):
        try:
            #self.vx_plot_curr_lap.setData(arc_lengths, self.node.speed[1])
            self.vx_plot_change.clear()
            self.vx_plot_change.setData(arc_lengths, velocity)
            self.vx_plot_change.setYRange(0., 12.)

            #self.sa_plot_curr_lap.setData(self.node.steering_angle[0], self.node.steering_angle[1])
            #self.sa_plot_ref_line.setData(self.node.ref_line[:, 0], np.arctan(self.node.ref_line[:, 5] * self.node.wheelbase))
            #self.sa_plot.setYRange(-0.5, 0.5)

        except Exception as e:
            return
        
    def clear_changed_vx_plot(self):
        try:
            self.vx_plot_change.clear()
            self.vx_plot_change.setYRange(0., 12.)
        except Exception as e:
            return

    #################################################
    #################### Toolbox ####################
    #################################################

    def onTabChanged(self, index):
        current_widget = self.toolbox.currentWidget()
        # If not in Bezier Curve tab, reset the Bezier widget and remove its visualization
        if current_widget != self.bezier_widget:
            try:
                self.bezier_widget.reset()
                for item in self.bezier_raceline_items:
                    self.scene.removeItem(item)
                    del item
                for pt in self.raceline_points:
                    pt.setOpacity(1.0)
            except:
                print("Warning: Points already have been deleted")
            self.bezier_raceline_items = []
        #if current_widget == self.smoothing_widget:
        #    self.smoothing_widget.update_initial_scene()
        # If Velocity Scaling tab is selected, update its plots


    
    #################################################
    #################### Sectors ####################
    #################################################

    def clear_sectors(self):
        self.sectors = []
        self.sector_table.clear()
        self.sector_table.setRowCount(0)
        self.sector_table.setHorizontalHeaderLabels(["ID", "Start", "End"])
        self.current_sector = None

        self.sectorChanged.emit()
    
    def start_sector_selection(self):
        self.selecting_sector = True
        self.selected_sector_points = []
        self.view.setCursor(Qt.CrossCursor)
        print("Sector selection mode activated. Please select two raceline points.")


    def sector_table_item_changed(self, item):
        try:
            row = item.row()
            # Read updated values from the table (expecting 8 columns):
            sector_id = int(self.sector_table.item(row, 0).text())
            start_idx = int(self.sector_table.item(row, 1).text())
            end_idx = int(self.sector_table.item(row, 2).text())

            # Update the corresponding sector tuple in self.sectors
            self.sectors[row] = (sector_id, start_idx, end_idx)
        except Exception as e:
            return
            #print("Error updating self.sectors from table:", e)

    def add_sector(self, idx1, idx2):
        # Determine total number of points either from self.raceline_points or the raceline array
        self.view.setCursor(Qt.ArrowCursor)

        if self.raceline_points:
            N = len(self.raceline_points)
        elif self.raceline is not None:
            N = len(self.raceline())
        else:
            print("No raceline loaded; cannot add sector.")
            return

        # Compute forward difference (handle wrapping)
        diff = (idx2 - idx1) if idx2 >= idx1 else (idx2 + N - idx1)
        # Choose the arc with fewer points
        if diff > N / 2:
            start_idx, end_idx = idx2, idx1
        else:
            start_idx, end_idx = idx1, idx2

        #if self.save_sectors_checkbox.isChecked():
        sector_id = len(self.sectors) + 1
        # --- New: Append default values for velocity_scaling, l1_scaling, l1_min, l1_max and overtaking ---
        sector = (sector_id, start_idx, end_idx)
        self.sectors.append(sector)
        self.add_sector_to_table(sector)
        print(f"Sector {sector_id} added: start index {start_idx}, end index {end_idx}")

        self.current_sector = (start_idx, end_idx)
        self.highlight_sector((start_idx, end_idx))

        #if not self.save_sectors_checkbox.isChecked(): 
        self.sectorChanged.emit()

        print(f"Sector selected (not saved): start index {start_idx}, end index {end_idx}")


    def sector_table_selection_changed(self):
        selected_items = self.sector_table.selectedItems()
        if selected_items:
            row = self.sector_table.currentRow()
            start_idx = int(self.sector_table.item(row, 1).text())
            end_idx = int(self.sector_table.item(row, 2).text())
            self.start_index, self.end_index = start_idx, end_idx
            self.current_sector = (start_idx, end_idx)
            self.highlight_sector((start_idx, end_idx))
        else:
            self.current_sector = None
            for pt in self.raceline_points:
                pt.setOpacity(1.0)
            self.set_sector_points_movable(False)
            self.sector_table.clearSelection()
        self.sectorChanged.emit()
        print("sector change emitted!")

    def add_sector_to_table(self, sector):
        # Updated to include velocity_scaling, l1_scaling, l1_min, l1_max, and overtaking columns.
        sector_id, start_idx, end_idx = sector
        row = self.sector_table.rowCount()
        #print(row)
        self.sector_table.insertRow(row)
        self.sector_table.setItem(row, 0, QTableWidgetItem(str(sector_id)))
        self.sector_table.setItem(row, 1, QTableWidgetItem(str(start_idx)))
        self.sector_table.setItem(row, 2, QTableWidgetItem(str(end_idx)))
        self.sector_table.selectRow(row)
