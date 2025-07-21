
import os
import subprocess
from pathlib import Path

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QScrollArea
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

from styles import button_thin_stylesheet, scrollarea_stylesheet

class ImageViewWidget(QWidget):
    def __init__(self, file_path=None, parent=None):
        super().__init__(parent)
        self.file_path = file_path

        # Create header with GIMP and Refresh buttons
        header_widget = QWidget()
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(5, 5, 0, 5)
        header_layout.setSpacing(5)

        self.gimp_btn = QPushButton("GIMP")
        self.gimp_btn.setStyleSheet(button_thin_stylesheet)
        self.gimp_btn.clicked.connect(self.open_in_gimp)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setStyleSheet(button_thin_stylesheet)
        self.refresh_btn.clicked.connect(self.refresh_image)

        header_layout.addWidget(self.gimp_btn)
        header_layout.addWidget(self.refresh_btn)
        header_layout.addStretch()
        header_widget.setLayout(header_layout)

        # Create image display area: a QLabel inside a QScrollArea
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: rgb(30,30,30);")

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet(scrollarea_stylesheet)
        self.scroll_area.setWidget(self.image_label)

        # Organize header and image display area in a vertical layout
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(header_widget)
        layout.addWidget(self.scroll_area)
        self.setLayout(layout)

        # If a file path is provided, load the image
        if self.file_path:
            self.load_image(self.file_path)

    def load_image(self, path: str):
        """Load the image from file and display it."""
        if os.path.exists(path):
            pix = QPixmap(path)
            if pix.isNull():
                self.image_label.setText("Failed to load image.")
            else:
                self.image_label.setPixmap(pix)
                self.file_path = path
        else:
            self.image_label.setText("File not found.")

    def refresh_image(self):
        """Reload the image from file."""
        if self.file_path:
            self.load_image(self.file_path)

    def open_in_gimp(self):
        """Open the currently loaded image in GIMP."""
        if self.file_path and os.path.isfile(self.file_path):
            subprocess.Popen(
                ["gimp", self.file_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
                start_new_session=True
            )