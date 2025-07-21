import sys, os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, 
                             QListWidget, QTabWidget, QVBoxLayout, QAction, QMenuBar,
                             QLabel, QLineEdit, QFormLayout, QGroupBox, QComboBox,
                             QTreeView, QFileSystemModel)
from PyQt5.QtCore import pyqtSignal, QDir, QUrl, Qt
from PyQt5.QtGui import QFont, QIcon

import argparse
from pathlib import Path
import yaml
import csv
import numpy as np
import random
import subprocess

import signal

from widgets import SectorTunerWidget, OTSectorTunerWidget, GlobalPlannerWidget, RacelineTuner, TextEditorWidget, ImageViewWidget, ParameterTunerWidget
from styles import *
from node import SubscriberNode, ROS2Thread

import threading
import webbrowser

class CustomWidget(QWidget):
    def __init__(self, name):
        super().__init__()
        layout = QVBoxLayout()
        layout.addWidget(QWidget())  # Placeholder for future content
        self.setLayout(layout)
        self.setWindowTitle(name)

class MainWindow(QMainWindow):
    map_changed = pyqtSignal(str)
    traj_type_changed = pyqtSignal(str)

    def __init__(self, map_name: str = None, traj_type: str = None):
        super().__init__()
        self.setWindowTitle("LamaRacing GUI")
        self.setGeometry(20, 20, 1280, 720)

        icon_path = str(Path(os.environ["LARACE_ROOT"]) / "src" / "planner"/ "planner_gui" / "misc" / "icon.png") 
        self.setWindowIcon(QIcon(icon_path))

        # Apply a global dark stylesheet for consistency
        self.setStyleSheet("""
        QMainWindow {
            background-color: rgb(30,30,30);
        }
        QLabel {
            color: rgb(250,250,250);
        }
        QListWidget {
            background-color: rgb(50,50,50);
            color: rgb(250,250,250);
            border: 1px solid rgb(50,50,50);
            padding: 4px;
        }
        QGroupBox {
            border: 1px solid rgb(250,250,250);
            border-radius: 5px;
            margin-top: 10px;
            background-color: rgb(40,40,40);
            color: rgb(250,250,250);
        }
        QTreeView {
            background-color: rgb(30,30,30);
            color: rgb(250,250,250);
            border: 1px solid rgb(50,50,50);
        }
        """)
                
        self.map_name = map_name
        self.traj_type = traj_type

        self.config_path = str(Path(os.environ["LARACE_ROOT"]) / "src" / "core" / "config")
        self.map_import_path = str(Path(os.environ["LARACE_ROOT"]) / "src" / "core" / "maps")
        self.map_path = str(Path(os.environ["LARACE_ROOT"]) / "src" / "core" / "maps" / map_name) if map_name is not None else ""

        # Create central widget and main layout
        central_widget = QWidget()
        main_layout = QHBoxLayout()

        # --- Left panel ---
        left_panel_layout = QVBoxLayout()
        # Create map info view and other left-panel widgets as before...
        self.map_info_group = QGroupBox("Map Information")
        map_info_layout = QFormLayout()
        self.map_name_combo = QComboBox()
        self.map_name_combo.setStyleSheet(combobox_stylesheet)
        self.map_name_combo.addItems(self.get_map_names())
        if map_name is not None:
            index = self.map_name_combo.findText(map_name)
            if index != -1:
                self.map_name_combo.setCurrentIndex(index)
        self.map_name_combo.currentIndexChanged.connect(self.on_map_name_changed)
        map_info_layout.addRow("Map Name:", self.map_name_combo)
        self.traj_type_edit = QLineEdit()
        self.traj_type_edit.setStyleSheet(line_edit_stylesheet)
        self.traj_type_edit.editingFinished.connect(self.on_traj_type_changed)
        map_info_layout.addRow("Raceline:", self.traj_type_edit)
        self.resolution_edit = QLabel("")
        map_info_layout.addRow("Resolution", self.resolution_edit)
        self.origin_edit = QLabel("")
        map_info_layout.addRow("Origin", self.origin_edit)
        self.map_info_group.setLayout(map_info_layout)
        left_panel_layout.addWidget(self.map_info_group)

        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(listwidget_stylesheet)
        self.list_widget.addItems(["Global Planner", "Sector Tuner", "Raceline Tuner", "Parameter Tuner", "RVIZ"])
        self.list_widget.itemClicked.connect(self.on_item_clicked)
        self.tools_label = QLabel("Tools")
        left_panel_layout.addWidget(self.tools_label)
        left_panel_layout.addWidget(self.list_widget)

        self.map_dir_label = QLabel("Map Directory")
        left_panel_layout.addWidget(self.map_dir_label)
        self.map_file_explorer = QTreeView()
        self.map_file_explorer.setStyleSheet(file_explorer_stylesheet)
        self.map_fs_model = QFileSystemModel()
        self.map_fs_model.setRootPath(self.map_import_path)
        self.map_file_explorer.setModel(self.map_fs_model)
        self.map_file_explorer.setRootIndex(self.map_fs_model.index(self.map_import_path))
        self.map_file_explorer.setColumnWidth(0, 200)
        self.map_file_explorer.header().hide()  # This hides the header completely
        self.map_file_explorer.doubleClicked.connect(self.on_map_file_open)

        left_panel_layout.addWidget(self.map_file_explorer)

        self.config_dir_label = QLabel("Config Directory")
        left_panel_layout.addWidget(self.config_dir_label)
        self.config_file_explorer = QTreeView()
        self.config_file_explorer.setStyleSheet(file_explorer_stylesheet)
        self.config_fs_model = QFileSystemModel()
        self.config_fs_model.setRootPath(self.config_path)
        self.config_file_explorer.setModel(self.config_fs_model)
        self.config_file_explorer.setRootIndex(self.config_fs_model.index(self.config_path))
        self.config_file_explorer.setColumnWidth(0, 200)
        self.config_file_explorer.header().hide()  # This hides the header completely
        self.config_file_explorer.doubleClicked.connect(self.on_config_file_open)
        left_panel_layout.addWidget(self.config_file_explorer)
        self.config_file_explorer.update

        # Wrap left layout in a widget and make it thinner
        left_panel_widget = QWidget()
        left_panel_widget.setLayout(left_panel_layout)
        left_panel_widget.setFixedWidth(250)  # Adjust width as needed

        # --- Right panel (tabs) ---
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet(tab_widget_stylesheet)
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.tabCloseRequested.connect(self.close_tab)

        main_layout.addWidget(left_panel_widget)
        main_layout.addWidget(self.tab_widget, 1)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Create menu bar and rest of the initialization as before...
        self.create_menu_bar()
        if self.map_name is not None:
            index = self.map_name_combo.findText(self.map_name)
            if index != -1:
                self.map_name_combo.setCurrentIndex(index)
            self.on_map_name_changed()
        if self.traj_type is not None:
            self.traj_type_edit.setText(self.traj_type)
            self.on_traj_type_changed()

        # Initialize widgets
        self.sector_tuner = None
        self.ot_sector_tuner = None
        self.global_planner = None
        self.raceline_tuner = None
        self.parameter_tuner = None

        # Settings

        # Parameters
        self.waypoints_real = None
        self.waypoints_image = None

        self.map_resolution = ""
        self.map_origin = ""
        self.map_negate = ""
        self.occ_thr = ""
        self.free_thr = ""

        self.ros_interface = None
        self.ros_thread = None

        self.toggle_ros_interface()


    def on_map_name_changed(self):
        new_map_name = self.map_name_combo.currentText()
        self.map_path = str(Path(os.environ["LARACE_ROOT"]) / "src" / "core" / "maps" / new_map_name)
        self.map_changed.emit(self.map_path)

        map_info_file = Path(self.map_path) / f"{Path(self.map_path).stem}.yaml"
        try:
            with open(map_info_file, 'r') as file:
                map_info = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"Map info file not found: {map_info_file}")
            return
        self.map_resolution = map_info["resolution"]
        self.map_origin = map_info["origin"]
        self.resolution_edit.setText(str(self.map_resolution))
        self.origin_edit.setText(str(self.map_origin[:2]))
        print(f"Map name changed to: {new_map_name}")

    def get_map_names(self):
        map_names = []
        for p in Path(self.map_import_path).iterdir():
            if p.is_dir() and os.path.exists(p / f"{p.stem}.yaml"):
                map_names.append(str(p.stem))
        return map_names

    def on_traj_type_changed(self):
        self.traj_type = self.traj_type_edit.text()
        self.traj_type_changed.emit(self.traj_type)
        print(f"Trajectory type changed to: {self.traj_type}")

    def create_menu_bar(self):
        menu_bar = self.menuBar()
        menu_bar.setStyleSheet(menu_bar_stylesheet)
        settings_menu = menu_bar.addMenu("Settings")
        
        # Create a checkable action for the ROS interface.
        self.toggle_ros_action = QAction("ROS Interface", self)
        self.toggle_ros_action.setCheckable(True)
        self.toggle_ros_action.setChecked(True)
        self.toggle_ros_action.triggered.connect(self.toggle_ros_interface)
        settings_menu.addAction(self.toggle_ros_action)

        help_menu = menu_bar.addMenu("Help")
        self.help_wiki_action = QAction("Wiki", self)
        self.help_wiki_action.triggered.connect(self.open_wiki)
        help_menu.addAction(self.help_wiki_action)
        self.more_help_action = QAction("More Help", self)
        self.more_help_action.triggered.connect(self.open_more_help)
        help_menu.addAction(self.more_help_action)


    def on_item_clicked(self, item):
        widget_name = item.text()
        if widget_name == "Sector Tuner":
            self.open_sector_tuner()
        elif widget_name == "Global Planner":
            self.open_global_planner()
        elif widget_name == "Raceline Tuner":
            self.open_raceline_tuner()
        elif widget_name == "Parameter Tuner":
            self.open_parameter_tuner()
        elif widget_name == "RVIZ":
            rviz_config_file = os.path.join(os.environ["LARACE_ROOT"], "src", "core", "rviz", "base.rviz")
            print(f"Opening RVIZ with config file: {rviz_config_file}")
            subprocess.Popen(
                ["rviz2", "-d", rviz_config_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
                start_new_session=True
            )

    def open_wiki(self):
        wiki_url = "https://gitlab.igg.uni-bonn.de/f1_tenth/race_stack/-/wikis/home"
        subprocess.Popen(
            ["xdg-open", f"{wiki_url}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL,
            start_new_session=True
        )

    def open_more_help(self):
        wiki_url = "https://chatgpt.com"
        subprocess.Popen(
            ["xdg-open", f"{wiki_url}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL,
            start_new_session=True
        )


    def open_parameter_tuner(self):
        if self.map_path == "":
            print("No map loaded. Please load a map first.")
            return
        if self.parameter_tuner is None and self.ros_interface is not None:
            self.parameter_tuner = ParameterTunerWidget(self.ros_interface)
            #self.map_changed.connect(self.global_planner.set_map)
            #self.global_planner.set_map(str(self.map_path))
            self.tab_widget.addTab(self.parameter_tuner, "Parameter Tuner")

        self.tab_widget.setCurrentWidget(self.parameter_tuner)

    def open_raceline_tuner(self):
        if self.map_path == "":
            print("No map loaded. Please load a map first.")
            return
        if self.raceline_tuner is None:
            self.raceline_tuner = RacelineTuner(str(self.map_path), self.ros_interface)
            self.map_changed.connect(self.raceline_tuner.set_map)
            #self.map_changed.connect(self.global_planner.set_map)
            #self.global_planner.set_map(str(self.map_path))
            self.tab_widget.addTab(self.raceline_tuner, "Raceline Tuner")
            self.raceline_tuner.racelineChanged()

        self.tab_widget.setCurrentWidget(self.raceline_tuner)

    def open_sector_tuner(self):
        if self.map_path == "":
            print("No map loaded. Please load a map first.")
            return
        if self.sector_tuner is None:
            try:
                self.sector_tuner = SectorTunerWidget(self.map_path, self.traj_type if self.traj_type is not None else "")
            except Exception as e:
                print(f"Error creating sector tuner widget: {e}")
                return
            self.map_changed.connect(self.sector_tuner.set_map)
            self.sector_tuner.set_map(str(self.map_path))
            if self.traj_type is not None:
                self.sector_tuner.set_trajectory(self.traj_type)
            self.tab_widget.addTab(self.sector_tuner, "Sector Tuner")
        self.tab_widget.setCurrentWidget(self.sector_tuner)

    def open_global_planner(self):
        if self.map_path == "":
            print("No map loaded. Please load a map first.")
            return
        if self.global_planner is None:
            self.global_planner = GlobalPlannerWidget()
            self.map_changed.connect(self.global_planner.set_map)
            self.global_planner.set_map(str(self.map_path))
            self.tab_widget.addTab(self.global_planner, "Global Planner")
        self.tab_widget.setCurrentWidget(self.global_planner)

    def close_tab(self, index):
        widget = self.tab_widget.widget(index)
        if widget == self.sector_tuner:
            self.sector_tuner = None
        elif widget == self.ot_sector_tuner:
            self.ot_sector_tuner = None
        elif widget == self.global_planner:
            self.global_planner = None
        elif widget == self.raceline_tuner:
            self.raceline_tuner = None
        elif widget == self.parameter_tuner:
            self.parameter_tuner = None
        self.tab_widget.removeTab(index)

    def on_config_file_open(self, index):
        """Open the clicked file in a new tab with a plain-text editor."""
        path = self.config_file_explorer.model().filePath(index)
        if not os.path.isfile(path):
            return  # ignore directories

        # Prevent opening the same file twice
        for i in range(self.tab_widget.count()):
            w = self.tab_widget.widget(i)
            if hasattr(w, 'file_path') and w.file_path == path:
                self.tab_widget.setCurrentIndex(i)
                return

        # Create editor, load file
        editor = TextEditorWidget(path)

        # Add as a new tab
        dir_name  = os.path.basename(os.path.dirname(path))
        file_name = os.path.basename(path)
        title = f"{dir_name}/{file_name}"
        idx = self.tab_widget.addTab(editor, title)
        self.tab_widget.setCurrentIndex(idx)

    def on_map_file_open(self, index):
        """Open the clicked file in a new tab with a plain-text editor."""
        path = self.config_file_explorer.model().filePath(index)
        if not os.path.isfile(path):
            return  # ignore directories

        path = Path(path)
        if path.suffix in [".png", ".jpeg", ".jpg"]:
            # Prevent opening the same file twice
            for i in range(self.tab_widget.count()):
                w = self.tab_widget.widget(i)
                if hasattr(w, 'file_path') and w.file_path == path:
                    self.tab_widget.setCurrentIndex(i)
                    return

            # Create editor, load file
            image_view = ImageViewWidget(str(path))

            # Add as a new tab
            dir_name  = os.path.basename(os.path.dirname(path))
            file_name = os.path.basename(path)
            title = f"{dir_name}/{file_name}"
            idx = self.tab_widget.addTab(image_view, title)
            self.tab_widget.setCurrentIndex(idx)
        elif path.suffix in [".yaml", ".yml", ".json", ".csv"]:
            for i in range(self.tab_widget.count()):
                w = self.tab_widget.widget(i)
                if hasattr(w, 'file_path') and w.file_path == path:
                    self.tab_widget.setCurrentIndex(i)
                    return

            # Create editor, load file
            editor = TextEditorWidget(str(path))

            # Add as a new tab
            dir_name  = os.path.basename(os.path.dirname(path))
            file_name = os.path.basename(path)
            title = f"{dir_name}/{file_name}"
            idx = self.tab_widget.addTab(editor, title)
            self.tab_widget.setCurrentIndex(idx)

    def toggle_ros_interface(self):
        if getattr(self, 'ros_interface', None) is None:
            self.activate_ros_interface()
        else:
            self.deactivate_ros_interface()

    def activate_ros_interface(self):
        if self.ros_interface is not None:
            return

        print("Initializing ROS interface...")

        self.ros_interface = SubscriberNode()
        self.ros_thread = ROS2Thread(self.ros_interface)
        self.ros_thread.start()
        if self.raceline_tuner is not None:
            self.raceline_tuner.activate_ros_interface(self.ros_interface)
        


    def deactivate_ros_interface(self):
        if self.ros_interface is None:
            print("ROS interface not activated.")
            return
        print("Destroying ROS interface...")
        self.ros_thread.quit()
        self.ros_interface.destroy()
        del self.ros_interface
        self.ros_interface = None

        if self.raceline_tuner is not None:
            self.raceline_tuner.deactivate_ros_interface()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Parameter Tuner")
    parser.add_argument('--map', '-m', default=None, help="Map to load")
    parser.add_argument('--trajectory', '-t', default=None, help="Map to load")
    args = parser.parse_args()

    map = args.map or (os.environ["LARACE_CURRENT_MAP"] if "LARACE_CURRENT_MAP" in os.environ else None)
    trajectory = args.trajectory or "min_curve"

    app = QApplication(sys.argv)
    window = MainWindow(map, trajectory)

    # Define a signal handler for SIGINT and SIGTERM
    def signal_handler(signum, frame):
        window.close()  # This will close the GUI window
        sys.exit(0)

    # Install the signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    window.showMaximized()
    sys.exit(app.exec_())
