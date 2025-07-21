from PyQt5.QtCore import QDir, QUrl, pyqtSignal, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
                             QLabel, QSlider, QTableWidget, QTableWidgetItem, QHeaderView,
                             QPushButton, QFileDialog, QMessageBox, QLineEdit, QFormLayout)
from PyQt5.QtGui import QPixmap, QImage, QDoubleValidator, QColor

import os
from pathlib import Path
import yaml
import csv
import numpy as np
from PIL import Image
import cv2 as cv
import colorsys
import random

class OTSectorTunerWidget(QWidget):
    def __init__(self, map_path: str = "", traj_type: str = ""):
        super().__init__()
        self.setWindowTitle("OT Sector Tuner")

        self.map_path = map_path
        self.map_file_path = ""
        self.traj_type = traj_type
        self.map_image = None
        self.map_image_edit = None

        self.waypoints_image = None
        self.waypoints_real = None
        self._selected_wp = None
        self._selected_start_point = None

        # Main layout
        main_layout = QHBoxLayout()

        # Left side (map and slider)
        left_layout = QVBoxLayout()
        
        # Map image label
        self.map_label = QLabel()
        left_layout.addWidget(self.map_label)

        # Controls layout
        controls_layout = QVBoxLayout()

        # Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.valueChanged.connect(self.update_slider_value)
        controls_layout.addWidget(self.slider)

        # Slider value label
        self.slider_value_label = QLabel("Current Waypoint: 0")
        controls_layout.addWidget(self.slider_value_label)

        # Current start index label
        self.start_index_label = QLabel("Current Start Index: N/A")
        controls_layout.addWidget(self.start_index_label)

        # Total waypoints label
        self.total_waypoints_label = QLabel("Total Waypoints: N/A")
        controls_layout.addWidget(self.total_waypoints_label)

        # Create a form layout for the parameters
        form_layout = QFormLayout()

        self.yeet_factor_edit = QLineEdit("1.25")
        form_layout.addRow("Yeet Factor:", self.yeet_factor_edit)

        self.spline_len_edit = QLineEdit("30")
        form_layout.addRow("Spline Length:", self.spline_len_edit)

        self.sector_begin_edit = QLineEdit("0.5")
        form_layout.addRow("Sector Begin:", self.sector_begin_edit)

        controls_layout.addLayout(form_layout)

        # Add buttons
        button_layout = QHBoxLayout()
        
        self.start_sector_button = QPushButton("Start Sector")
        self.start_sector_button.clicked.connect(self.start_sector)
        button_layout.addWidget(self.start_sector_button)

        self.end_sector_button = QPushButton("End Sector")
        self.end_sector_button.clicked.connect(self.end_sector)
        button_layout.addWidget(self.end_sector_button)
        # Add Save button
        self.remove_button = QPushButton("Remove")
        self.remove_button.clicked.connect(self.reset)
        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.save)
        action_buttons_layout = QHBoxLayout()
        action_buttons_layout.addWidget(self.remove_button)
        action_buttons_layout.addWidget(self.save_button)
        controls_layout.addLayout(action_buttons_layout)

        # Add controls to left layout
        left_layout.addLayout(button_layout)
        left_layout.addLayout(controls_layout)

        # Set alignment to top
        left_layout.setAlignment(Qt.AlignTop)

        main_layout.addLayout(left_layout)

        # Modify the table setup
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Sector", "Start", "End", "Overtaking"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.itemChanged.connect(self.on_table_item_changed)
        main_layout.addWidget(self.table)

        self.setLayout(main_layout)

        self.colors = self.generate_colors()
        random.shuffle(self.colors)
        self.slider.setValue(0)

        # Initialize sectors
        self.sectors = []

        if self.map_path is not None and self.map_path!="":
            self.open_map()
        if self.traj_type is not None and self.traj_type!="":
            self.load_trajectory()

        self.load()

    def update_slider_value(self, value):
        self.slider_value_label.setText(f"Slider Value: {value}")
        self._selected_wp = value
        self.update_start_index_label()
        self.draw_image()

    def update_start_index_label(self):
        if self._selected_start_point is not None:
            self.start_index_label.setText(f"Current Start Index: {self._selected_start_point}")
        else:
            self.start_index_label.setText("Current Start Index: N/A")

    def update_total_waypoints_label(self):
        if self.waypoints_image is not None:
            self.total_waypoints_label.setText(f"Total Waypoints: {len(self.waypoints_image)}")
        else:
            self.total_waypoints_label.setText("Total Waypoints: N/A")

    def start_sector(self):
        self._selected_start_point = self._selected_wp
        self.update_start_index_label()
        print(f"Started sector at waypoint {self._selected_start_point}")

    def end_sector(self):
        if self._selected_start_point is not None:
            end_point = self._selected_wp
            self.add_sector(self._selected_start_point, end_point)  # Default scale is 1.0
            print(f"Ended sector from waypoint {self._selected_start_point} to {end_point}")
            self.increment_slider()
            self.start_sector()
            self.draw_image()
        else:
            print("Please select a start point first")

    def increment_slider(self):
        self._selected_wp += 1
        self._selected_wp %= self.slider.maximum()
        self.slider.setValue(self._selected_wp)

    def set_map(self, map_path: str):
        self.map_path = map_path
        self.open_map()
        self.load()

    def reset(self):
        self.yeet_factor_edit.setText("1.25")
        self.spline_len_edit.setText("30")
        self.sector_begin_edit.setText("0.5")
        self.sectors = []
        self._selected_wp = 0
        self.update_start_index_label()
        self.update_total_waypoints_label()
        self.update_table()

    def set_trajectory(self, traj_type: str):
        self.traj_type = traj_type
        self.load_trajectory()
        self.load()

    def open_map(self):

        map_info_file = Path(self.map_path) / f"{Path(self.map_path).stem}.yaml"
        try:
            with open(map_info_file, 'r') as file:
                map_info = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"Map info file not found: {map_info_file}")
            return
        self.map_file_path = Path(self.map_path) / map_info["planner_image"]
        self.map_resolution = map_info["resolution"]
        self.map_origin = np.array(map_info["origin"])[:2]

        try:
            # Load image into numpy array
            with Image.open(self.map_file_path) as img:
                self.map_image = np.array(img)
                self.map_image = self.map_image[...,np.newaxis].repeat(3,axis=-1)
                self.map_image_edit = np.copy(self.map_image)

            self.display_map()
        except Exception as e:
            print(f"Error opening map image: {e}")
            return
        
    def load_trajectory(self):
        if self.traj_type is None or self.traj_type == "":
            return
        glb_wpnts_file = Path(self.map_path) / "plans" / self.traj_type / "global_waypoints.csv"
        with open(glb_wpnts_file, 'r') as csvfile:
            reader = csv.reader(csvfile)
            headers = next(reader)  # Read the header row
            data = np.array([list(map(float, row)) for row in reader])

        #import ipdb; ipdb.set_trace()
        self.waypoints_real = data[:, 1:3]
        self.waypoints_image = np.copy(self.waypoints_real)
        self.waypoints_image = self.waypoints_image-self.map_origin
        #self.waypoints_image[:,0] -= self.map_image.shape[0]//2
        #self.waypoints_image[:,1] -= self.map_image.shape[1]//2
        self.waypoints_image /= self.map_resolution
        self.waypoints_image[:, 1] = self.map_image.shape[0] - self.waypoints_image[:, 1]
        self.waypoints_image = np.round(self.waypoints_image).astype(np.int64)

        self._selected_wp = 0
        self.start_sector()
        self.slider.setMaximum(self.waypoints_image.shape[0] - 1)

        self.update_total_waypoints_label()
        self.draw_image()
       
    def draw_image(self):
        
        drawn_wps = set()

        self.map_image_edit = np.copy(self.map_image)

        for sector in self.sectors:
            start = sector["start"]
            end = sector["end"]
            color = sector["color"]
            for i in range(start, end):
                drawn_wps.add(i)
                wp = self.waypoints_image[i]
                self.map_image_edit = cv.circle(self.map_image_edit, (wp[0], wp[1]), 1, color, thickness=-1)

        if self._selected_start_point is not None:
            for i in range(self._selected_start_point, self._selected_wp):
                wp = self.waypoints_image[i]
                drawn_wps.add(i)
                self.map_image_edit = cv.circle(self.map_image_edit, (wp[0], wp[1]), 1, (255, 0, 0), thickness=-1)
        if self._selected_wp is not None:
            drawn_wps.add(self._selected_wp)
            wp = self.waypoints_image[self._selected_wp]
            self.map_image_edit = cv.circle(self.map_image_edit, (wp[0], wp[1]), 2, (190, 25, 25), thickness=-1)

        for i, wp in enumerate(self.waypoints_image):
            # draw red circles in the waypoints image
            if i in drawn_wps:
                continue
            else:
                self.map_image_edit = cv.circle(self.map_image_edit, (wp[0], wp[1]), 1, (128, 128, 128), thickness=-1)
        
        self.display_map()

    

    def display_map(self):
        if self.map_image is not None:
            height, width, _ = self.map_image_edit.shape
            bytes_per_line = 3 * width
            qimage = QImage(self.map_image_edit.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimage)
            self.map_label.setPixmap(pixmap.scaled(self.map_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def update_slider_value(self, value):
        self.slider_value_label.setText(f"Slider Value: {value}")
        self._selected_wp = value
        if self.waypoints_real is not None:
            self.draw_image()  # Update the map with the selected waypoint
        # Here you can add logic to update sectors based on slider value

    def add_sector(self, start, end):
        sector_number = len(self.sectors) + 1
        self.sectors.append({"start": start, "end": end, "overtaking": False, "color": self.colors[sector_number - 1]})
        self.update_table()

    def update_table(self):
        self.table.setRowCount(len(self.sectors))
        for i, sector in enumerate(self.sectors):
            sector_number = QTableWidgetItem(str(i + 1))
            start = QTableWidgetItem(str(sector["start"]))
            end = QTableWidgetItem(str(sector["end"]))
            
            overtaking_checkbox = QTableWidgetItem()
            overtaking_checkbox.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            overtaking_checkbox.setCheckState(Qt.Checked if sector["overtaking"] else Qt.Unchecked)

            color = sector["color"]
            qt_color = QColor(*color)
            
            for item in [sector_number, start, end, overtaking_checkbox]:
                item.setBackground(qt_color)
                if sum(color) > 382:
                    item.setForeground(QColor(0, 0, 0))
                else:
                    item.setForeground(QColor(255, 255, 255))

            self.table.setItem(i, 0, sector_number)
            self.table.setItem(i, 1, start)
            self.table.setItem(i, 2, end)
            self.table.setItem(i, 3, overtaking_checkbox)

        for row in range(self.table.rowCount()):
            self.table.setRowHeight(row, 30)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.display_map()  # Redisplay the map to fit the new size

    def generate_colors(self, n=20):
        colors = []
        for i in range(n):
            hue = i / n
            saturation = 0.7 + (i % 3) * 0.1  # Vary saturation slightly
            value = 0.8 + (i % 2) * 0.1  # Vary value slightly
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            # Convert to 8-bit RGB values
            color = tuple(int(x * 255) for x in rgb)
            colors.append(color)
        return colors
    def on_table_item_changed(self, item):
        row = item.row()
        col = item.column()
        new_value = item.text()

        if col == 0:  # Sector number (read-only)
            return

        try:
            if col in [1, 2]:  # Start or End
                new_value = int(new_value)
            elif col == 3:  # Overtaking
                new_value = item.checkState() == Qt.Checked
        except ValueError:
            print(f"Invalid input: {new_value}")
            self.update_table()  # Revert changes
            return

        if 0 <= row < len(self.sectors):
            if col == 1:
                self.sectors[row]["start"] = new_value
            elif col == 2:
                self.sectors[row]["end"] = new_value
            elif col == 3:
                self.sectors[row]["overtaking"] = new_value

            self.draw_image()  # Update the map display

    def save(self):
        yeet_factor = float(self.yeet_factor_edit.text())
        spline_len = int(self.spline_len_edit.text())
        sector_begin = float(self.sector_begin_edit.text())

        # Create the YAML structure
        save_path = Path(self.map_path) / "plans" / self.traj_type / "ot_sectors.yaml"

        sector_dict = [
            {
                "start": sector["start"],
                "end": sector["end"],
                "ot_flag": sector["overtaking"],
                "force_fallback": False
            }
        for sector in self.sectors]

        save_dict = {
            "yeet_factor": yeet_factor,
            "spline_len": spline_len,
            "ot_sector_begin": sector_begin,
            "n_sectors": len(self.sectors),
            "sectors": sector_dict
        }

        try:
            with open(save_path, "w") as file:
                yaml.dump(save_dict, file)
            print(f"Sectors saved to {save_path}")
        except Exception as e:
            print(f"Error saving sectors: {e}")

    def load(self):
        load_path = Path(self.map_path) / "plans" / self.traj_type / "ot_sectors.yaml"
        if not os.path.exists(load_path):
            return

        try:
            with open(load_path, "r") as file:
                loaded_data = yaml.safe_load(file)
            
            # Set the values in the widget
            self.yeet_factor_edit.setText(str(loaded_data.get("yeet_factor", 1.25)))
            self.spline_len_edit.setText(str(loaded_data.get("spline_len", 30)))
            self.sector_begin_edit.setText(str(loaded_data.get("ot_sector_begin", 0.5)))
            
            # Clear existing sectors
            self.sectors.clear()
            
            # Load sectors
            for sector in loaded_data.get("sectors", []):
                self.sectors.append({
                    "start": sector["start"],
                    "end": sector["end"],
                    "overtaking": sector["ot_flag"],
                    "color": self.colors[len(self.sectors)]  # Assign a color from the existing color list
                })
            
            # Update the table and redraw the image
            self.update_table()
            self.draw_image()
            
            print(f"Sectors loaded from {load_path}")
        except FileNotFoundError:
            print(f"File not found: {load_path}")
        except Exception as e:
            print(f"Error loading sectors: {e}")