from PyQt5.QtCore import QDir, QUrl, pyqtSignal, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
                             QLabel, QSlider, QTableWidget, QTableWidgetItem, QHeaderView,
                             QPushButton, QFileDialog, QMessageBox, QLineEdit, QFormLayout, QCheckBox)
from PyQt5.QtGui import QPixmap, QImage, QDoubleValidator, QColor

from pathlib import Path
import json
import yaml
import csv
import numpy as np
from PIL import Image
import cv2 as cv
import colorsys
import random

from rosidl_runtime_py.convert import message_to_ordereddict
from f110_msgs.msg import SectorArray, Sector 

from styles import *

class SectorTunerWidget(QWidget):
    def __init__(self, map_path: str = "", traj_type: str = ""):
        super().__init__()
        self.setWindowTitle("Sector Tuner")
        
        # Optionally set a background if not already set by the main window style
        self.setStyleSheet("background-color: rgb(30,30,30);")
        
        self.map_path = map_path
        print("Sector tuner map path:", map_path)
        self.map_file_path = ""
        self.traj_type = traj_type
        self.map_image = None
        self.map_image_edit = None

        self.waypoints_image = None
        self.arc_lengths = None
        self.waypoints_real = None
        self._selected_wp = 0
        self._selected_start_point = None

        # Main layout: left side (controls and map) and right side (table)
        main_layout = QHBoxLayout()

        # Left side layout
        left_layout = QVBoxLayout()

        # Map image label (centered and bigger)
        self.map_label = QLabel()
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setMinimumSize(800, 600)
        left_layout.addWidget(self.map_label)

        # Controls layout (slider, labels, parameters, buttons)
        controls_layout = QVBoxLayout()

        # Slider and labels
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)  # Will be updated when trajectory is loaded
        self.slider.valueChanged.connect(self.update_slider_value)
        self.slider.setStyleSheet(slider_stylesheet)  # Apply slider style
        controls_layout.addWidget(self.slider)

        self.slider_value_label = QLabel("Slider Value: 0")
        controls_layout.addWidget(self.slider_value_label)

        self.start_index_label = QLabel("Current Start Index: N/A")
        controls_layout.addWidget(self.start_index_label)

        self.total_waypoints_label = QLabel("Total Waypoints: N/A")
        controls_layout.addWidget(self.total_waypoints_label)

        # Horizontal layout for default Vmax and global limit
        scale_layout = QHBoxLayout()
        self.default_v_max_label = QLabel("Default Vmax:")
        self.default_v_max_edit = QLineEdit("8.0")
        # Allow higher range for v_max if needed
        self.default_v_max_edit.setValidator(QDoubleValidator(0.0, 100.0, 2))
        self.default_v_max_edit.setStyleSheet(line_edit_stylesheet)
        
        scale_layout.addWidget(self.default_v_max_label)
        scale_layout.addWidget(self.default_v_max_edit)
        controls_layout.addLayout(scale_layout)

        # New default boolean fields for overtaking and fallback
        default_bool_layout = QHBoxLayout()
        self.default_overtaking_label = QLabel("Default Overtaking:")
        self.default_overtaking_checkbox = QCheckBox()
        self.default_overtaking_checkbox.setStyleSheet(checkbox_stylesheet)
        self.default_overtaking_checkbox.setChecked(True)
        self.default_fallback_label = QLabel("Default Fallback:")
        self.default_fallback_checkbox = QCheckBox()
        self.default_fallback_checkbox.setStyleSheet(checkbox_stylesheet)
        self.default_fallback_checkbox.setChecked(True)
        default_bool_layout.addWidget(self.default_overtaking_label)
        default_bool_layout.addWidget(self.default_overtaking_checkbox)
        default_bool_layout.addWidget(self.default_fallback_label)
        default_bool_layout.addWidget(self.default_fallback_checkbox)
        controls_layout.addLayout(default_bool_layout)

        # --- New default L1 parameters ---
        l1_layout = QFormLayout()
        self.default_l1_scale_edit = QLineEdit("1.0")
        self.default_l1_shift_edit = QLineEdit("0.0")
        self.default_l1_min_edit   = QLineEdit("0.5")
        self.default_l1_max_edit   = QLineEdit("1.5")
        for w in (self.default_l1_scale_edit,
                  self.default_l1_shift_edit,
                  self.default_l1_min_edit,
                  self.default_l1_max_edit):
            w.setValidator(QDoubleValidator(0.0, 100.0, 3))
            w.setStyleSheet(line_edit_stylesheet)
        l1_layout.addRow("Default L1 Scale:", self.default_l1_scale_edit)
        l1_layout.addRow("Default L1 Shift:", self.default_l1_shift_edit)
        l1_layout.addRow("Default L1 Min:",   self.default_l1_min_edit)
        l1_layout.addRow("Default L1 Max:",   self.default_l1_max_edit)
        controls_layout.addLayout(l1_layout)
        # --- end L1 defaults ---

        # Buttons for sector selection
        button_layout = QHBoxLayout()
        self.start_sector_button = QPushButton("Start Sector")
        self.start_sector_button.setStyleSheet(button_stylesheet)
        self.start_sector_button.clicked.connect(self.start_sector)
        button_layout.addWidget(self.start_sector_button)
        
        self.end_sector_button = QPushButton("End Sector")
        self.end_sector_button.setStyleSheet(button_stylesheet)
        self.end_sector_button.clicked.connect(self.end_sector)
        button_layout.addWidget(self.end_sector_button)
        controls_layout.addLayout(button_layout)

        # Action buttons: Remove and Save
        action_buttons_layout = QHBoxLayout()
        self.remove_button = QPushButton("Remove")
        self.remove_button.setStyleSheet(button_stylesheet)
        self.remove_button.clicked.connect(self.reset)
        self.save_button = QPushButton("Save")
        self.save_button.setStyleSheet(button_stylesheet)
        self.save_button.clicked.connect(self.save)
        action_buttons_layout.addWidget(self.remove_button)
        action_buttons_layout.addWidget(self.save_button)
        controls_layout.addLayout(action_buttons_layout)

        left_layout.addLayout(controls_layout)
        left_layout.setAlignment(Qt.AlignTop)
        main_layout.addLayout(left_layout)

        # Table with now 12 columns (new column: Bang Bang)
        self.table = QTableWidget(0, 12)
        self.table.setHorizontalHeaderLabels([
            "Sector","Start","End","Vmax","Fallback","Overtaking",
            "L1 Scale","L1 Shift","L1 Min","L1 Max","Force Fallback", "Bang Bang"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.itemChanged.connect(self.on_table_item_changed)
        self.table.setStyleSheet(table_stylesheet)  # Apply table style
        
        # Force header fonts to white
        self.table.horizontalHeader().setStyleSheet("color: white;")
        self.table.verticalHeader().setStyleSheet("color: white;")
        
        main_layout.addWidget(self.table)

        self.setLayout(main_layout)

        self.colors = self.generate_colors()
        random.shuffle(self.colors)
        self.slider.setValue(0)

        self.sectors = []
        if self.map_path:
            self.open_map()
        if self.traj_type:
            self.load_trajectory()

        self.load()

    def update_slider_value(self, value):
        self.slider_value_label.setText(f"Slider Value: {value}")
        self._selected_wp = value
        self.update_start_index_label()
        if self.waypoints_real is not None:
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
            v_max = float(self.default_v_max_edit.text())
            default_overtaking = self.default_overtaking_checkbox.isChecked()
            default_fallback   = self.default_fallback_checkbox.isChecked()
            # --- grab L1 defaults ---
            l1_scale = float(self.default_l1_scale_edit.text())
            l1_shift = float(self.default_l1_shift_edit.text())
            l1_min   = float(self.default_l1_min_edit.text())
            l1_max   = float(self.default_l1_max_edit.text())
            self.add_sector(self._selected_start_point, end_point,
                            v_max, default_fallback, default_overtaking,
                            l1_scale, l1_shift, l1_min, l1_max)
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
        self.default_v_max_edit.setText("8.0")
        self.default_overtaking_checkbox.setChecked(True)
        self.default_fallback_checkbox.setChecked(True)
        self.default_l1_scale_edit.setText("1.0")
        self.default_l1_shift_edit.setText("0.0")
        self.default_l1_min_edit.setText("0.5")
        self.default_l1_max_edit.setText("1.5")
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
            with Image.open(self.map_file_path) as img:
                self.map_image = np.array(img)
                self.map_image = self.map_image[..., np.newaxis].repeat(3, axis=-1)
                self.map_image_edit = np.copy(self.map_image)
            self.display_map()
        except Exception as e:
            print(f"Error opening map image: {e}")
            return

    def load_trajectory(self):
        if not self.traj_type:
            return
        glb_wpnts_file = Path(self.map_path) / "plans" / f"{self.traj_type}.csv"
        with open(glb_wpnts_file, 'r') as csvfile:
            reader = csv.reader(csvfile)
            headers = next(reader)
            data = np.array([list(map(float, row)) for row in reader])
        self.waypoints_real = data[:, 1:3]
        self.arc_lengths = data[:,0]
        self.waypoints_image = np.copy(self.waypoints_real)
        self.waypoints_image = self.waypoints_image - self.map_origin
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

    def add_sector(self, start, end, v_max, fallback, overtaking,
                   l1_scale, l1_shift, l1_min, l1_max):
        sector_number = len(self.sectors) + 1
        self.sectors.append({
            "start": start, "end": end,
            "v_max": v_max, "fallback": fallback, "overtaking": overtaking,
            "l1_scale": l1_scale, "l1_shift": l1_shift,
            "l1_min":   l1_min,   "l1_max":   l1_max,
            "force_fallback": False,              # <-- new flag already added
            "bang_bang": False,                   # <-- new flag added
            "color": self.colors[sector_number-1]
        })
        self.update_table()

    def update_table(self):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.sectors))
        for i, sector in enumerate(self.sectors):
            sector_number_item = QTableWidgetItem(str(i + 1))
            start_item = QTableWidgetItem(str(sector["start"]))
            end_item = QTableWidgetItem(str(sector["end"]))
            v_max_item = QTableWidgetItem(str(sector["v_max"]))
            # Create checkable items for fallback and overtaking
            fallback_item = QTableWidgetItem()
            fallback_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            fallback_item.setCheckState(Qt.Checked if sector["fallback"] else Qt.Unchecked)
            overtaking_item = QTableWidgetItem()
            overtaking_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            overtaking_item.setCheckState(Qt.Checked if sector["overtaking"] else Qt.Unchecked)
            # --- new L1 items ---
            l1_scale_item = QTableWidgetItem(str(sector["l1_scale"]))
            l1_shift_item = QTableWidgetItem(str(sector["l1_shift"]))
            l1_min_item   = QTableWidgetItem(str(sector["l1_min"]))
            l1_max_item   = QTableWidgetItem(str(sector["l1_max"]))
            # --- new force_fallback item ---
            force_fallback_item = QTableWidgetItem()
            force_fallback_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            force_fallback_item.setCheckState(Qt.Checked if sector.get("force_fallback", False) else Qt.Unchecked)
            # --- new bang_bang item ---
            bang_bang_item = QTableWidgetItem()
            bang_bang_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            bang_bang_item.setCheckState(Qt.Checked if sector.get("bang_bang", False) else Qt.Unchecked)

            color = sector["color"]
            qt_color = QColor(*color)
            
            # Set background and force font color white for non-checkable items
            for item in [sector_number_item, start_item, end_item, v_max_item,
                         l1_scale_item, l1_shift_item, l1_min_item, l1_max_item]:
                item.setBackground(qt_color)
                item.setForeground(QColor(255, 255, 255))
            
            # Set the background for checkable items and force font color white
            fallback_item.setBackground(qt_color)
            fallback_item.setForeground(QColor(255, 255, 255))
            overtaking_item.setBackground(qt_color)
            overtaking_item.setForeground(QColor(255, 255, 255))
            force_fallback_item.setBackground(qt_color)
            force_fallback_item.setForeground(QColor(255, 255, 255))
            bang_bang_item.setBackground(qt_color)
            bang_bang_item.setForeground(QColor(255, 255, 255))
            
            self.table.setItem(i, 0, sector_number_item)
            self.table.setItem(i, 1, start_item)
            self.table.setItem(i, 2, end_item)
            self.table.setItem(i, 3, v_max_item)
            self.table.setItem(i, 4, fallback_item)
            self.table.setItem(i, 5, overtaking_item)
            self.table.setItem(i, 6, l1_scale_item)
            self.table.setItem(i, 7, l1_shift_item)
            self.table.setItem(i, 8, l1_min_item)
            self.table.setItem(i, 9, l1_max_item)
            self.table.setItem(i, 10, force_fallback_item)
            self.table.setItem(i, 11, bang_bang_item)

        for row in range(self.table.rowCount()):
            self.table.setRowHeight(row, 30)
        self.table.blockSignals(False)

    def on_table_item_changed(self, item):
        row = item.row()
        col = item.column()
        if col == 0:
            return
        if row < 0 or row >= len(self.sectors):
            return

        if col in [1, 2]:
            try:
                new_value = int(item.text())
            except ValueError:
                print(f"Invalid input: {item.text()}")
                self.update_table()
                return
            if col == 1:
                self.sectors[row]["start"] = new_value
            elif col == 2:
                self.sectors[row]["end"] = new_value
        elif col == 3:
            try:
                new_value = float(item.text())
            except ValueError:
                print(f"Invalid input: {item.text()}")
                self.update_table()
                return
            self.sectors[row]["v_max"] = new_value
        elif col == 4:
            new_value = item.checkState() == Qt.Checked
            self.sectors[row]["fallback"] = new_value
        elif col == 5:
            new_value = item.checkState() == Qt.Checked
            self.sectors[row]["overtaking"] = new_value
        elif col in (6, 7, 8, 9):
            try:
                new_val = float(item.text())
            except ValueError:
                self.update_table()
                return
            key = {6: "l1_scale", 7: "l1_shift", 8: "l1_min", 9: "l1_max"}[col]
            self.sectors[row][key] = new_val
        elif col == 10:
            new_value = item.checkState() == Qt.Checked
            self.sectors[row]["force_fallback"] = new_value
        elif col == 11:
            new_value = item.checkState() == Qt.Checked
            self.sectors[row]["bang_bang"] = new_value

        self.draw_image()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.display_map()

    def generate_colors(self, n=20):
        colors = []
        for i in range(n):
            hue = i / n
            saturation = 0.7 + (i % 3) * 0.1  # Slight variation in saturation
            value = 0.8 + (i % 2) * 0.1       # Slight variation in value
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            color = tuple(int(x * 255) for x in rgb)
            colors.append(color)
        return colors

    def save(self):
        save_path = Path(self.map_path) / "plans" / f"{self.traj_type}_sectors.json"
   
        try:
            default_v_max = float(self.default_v_max_edit.text())
        except ValueError:
            default_v_max = 1.0

        default_overtaking = self.default_overtaking_checkbox.isChecked()
        default_fallback = self.default_fallback_checkbox.isChecked()

        default_l1_scale = float(self.default_l1_scale_edit.text())
        default_l1_shift = float(self.default_l1_shift_edit.text())
        default_l1_min   = float(self.default_l1_min_edit.text())
        default_l1_max   = float(self.default_l1_max_edit.text())

        sector_dict = [
            {
                "start_ind": s["start"], "end_ind": s["end"],
                "start_s": self.arc_lengths[int(s["start"])], "end_s": self.arc_lengths[min(int(s["end"])+1, len(self.arc_lengths)-1)],
                "v_max": s["v_max"], "fallback": s["fallback"],
                "overtaking": s["overtaking"],
                "l1_scale": s["l1_scale"], "l1_shift": s["l1_shift"],
                "l1_min":   s["l1_min"],   "l1_max":   s["l1_max"],
                "force_fallback": s["force_fallback"],
                "bang_bang": s["bang_bang"]       # <-- new flag saved
            } for s in self.sectors
        ]

        save_dict = {
            "default_v_max": default_v_max,
            "default_overtaking": default_overtaking,
            "default_fallback": default_fallback,
            "default_l1_scale": default_l1_scale,
            "default_l1_shift": default_l1_shift,
            "default_l1_min":   default_l1_min,
            "default_l1_max":   default_l1_max,
            "n_sectors": len(self.sectors),
            "sectors": sector_dict,
        }
        try:
            with open(save_path, "w") as file:
                json.dump(save_dict, file, indent=4)
            print(f"Sectors saved to {save_path}")
        except Exception as e:
            print(f"Error saving sectors: {e}")

    def load(self):
        load_path = Path(self.map_path) / "plans" / f"{self.traj_type}_sectors.json"
        if not load_path.exists():
            print(f"No saved sectors file found at {load_path}")
            return
        try:
            with open(load_path, "r") as file:
                loaded_data = json.load(file)
            self.default_v_max_edit.setText(str(loaded_data.get("default_v_max", 1.0)))
            self.default_overtaking_checkbox.setChecked(bool(loaded_data.get("default_overtaking", False)))
            self.default_fallback_checkbox.setChecked(bool(loaded_data.get("default_fallback", False)))
            self.default_l1_scale_edit.setText(str(loaded_data.get("default_l1_scale", 1.0)))
            self.default_l1_shift_edit.setText(str(loaded_data.get("default_l1_shift", 0.0)))
            self.default_l1_min_edit.setText(str(loaded_data.get("default_l1_min", 0.5)))
            self.default_l1_max_edit.setText(str(loaded_data.get("default_l1_max", 1.5)))
            self.sectors.clear()
            for sector in loaded_data.get("sectors", []):
                self.sectors.append({
                    "start": sector["start_ind"],
                    "end":   sector["end_ind"],
                    "v_max": sector["v_max"],
                    "fallback":   sector["fallback"],
                    "overtaking": sector["overtaking"],
                    "l1_scale": sector.get("l1_scale", 1.0),
                    "l1_shift": sector.get("l1_shift", 0.0),
                    "l1_min":   sector.get("l1_min", 0.5),
                    "l1_max":   sector.get("l1_max", 1.5),
                    "force_fallback": sector.get("force_fallback", False),
                    "bang_bang": sector.get("bang_bang", False),   # <-- new flag loaded
                    "color": self.colors[len(self.sectors) % len(self.colors)]
                })
            self.update_table()
            self.draw_image()
            print(f"Sectors loaded from {load_path}")
        except Exception as e:
            print(f"Error loading sectors: {e}")
            
# To test the widget standalone, you can uncomment and run the following:
# if __name__ == "__main__":
#     import sys
#     app = QApplication(sys.argv)
#     widget = SectorTunerWidget(map_path="path/to/map_directory", traj_type="trajectory_name")
#     widget.show()
#     sys.exit(app.exec_())