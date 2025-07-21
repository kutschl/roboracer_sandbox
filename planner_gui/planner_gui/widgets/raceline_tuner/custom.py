import sys
import json
import os
import numpy as np
from typing import Tuple
from pathlib import Path
from datetime import datetime
import traceback
import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QWidget, QGraphicsScene, QGraphicsView, QApplication, QGridLayout,
    QGraphicsEllipseItem, QGraphicsItem, QTableWidget, QHeaderView, QTableWidgetItem,
    QHBoxLayout, QPushButton, QVBoxLayout, QTabWidget, QLabel, QTextEdit, QComboBox, QCheckBox,
    QMenu
)
from PyQt5.QtGui import QPen, QBrush, QColor, QPolygonF, QPainterPath, QPainter
from PyQt5.QtCore import QPointF, Qt, pyqtSlot, QLineF, QRectF, pyqtSignal
import yaml

# Additional imports for dynamic scaling and velocity graphs
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from larace_pitwall.widgets.utils import *
from larace_pitwall.styles import *

from trajectory_planning_helpers.import_veh_dyn_info import import_veh_dyn_info 



class GridGraphicsView(QGraphicsView):
    def __init__(self, scene, parent=None):
        super(GridGraphicsView, self).__init__(scene, parent)
        self.gridSize = 20  # grid spacing in pixels
        self.gridPen = QPen(QColor(80, 80, 80), 1)  # grid line appearance

    def drawBackground(self, painter: QPainter, rect: QRectF):
        super(GridGraphicsView, self).drawBackground(painter, rect)
        painter.setPen(self.gridPen)
        intLeft = int(rect.left()) - (int(rect.left()) % self.gridSize)
        intTop = int(rect.top()) - (int(rect.top()) % self.gridSize)
        x = intLeft
        while x < rect.right():
            painter.drawLine(QLineF(x, rect.top(), x, rect.bottom()))
            x += self.gridSize
        y = intTop
        while y < rect.bottom():
            painter.drawLine(QLineF(rect.left(), y, rect.right(), y))
            y += self.gridSize

from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsObject
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import QRectF, Qt, QObject

from PyQt5.QtWidgets import QGraphicsObject
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QBrush, QPen

class InteractiveBar(QGraphicsObject):
    # Signals for selection and dragging.
    barSelected = pyqtSignal(int)
    barDragged = pyqtSignal(int, float)
    
    def __init__(self, x, y, width, height, index, active=True, parent=None):
        super(InteractiveBar, self).__init__(parent)
        self._rect = QRectF(x, y, width, height)
        self.index = index
        self.setFlags(QGraphicsObject.ItemIsSelectable)
        self.setAcceptHoverEvents(True)
        self.dragging = False
        self.active=active
        self.drag_offset = 0
        # Default brush; can be changed later.
        self._brush = QBrush(Qt.blue)
        self._pen = QPen(Qt.NoPen)
    
    def boundingRect(self):
        return self._rect
    
    def paint(self, painter, option, widget):
        painter.setBrush(self._brush)
        painter.setPen(self._pen)
        painter.drawRect(self._rect)
    
    def setRect(self, rect):
        self.prepareGeometryChange()
        self._rect = rect
        self.update()
    
    def rect(self):
        return self._rect
    
    def setBrush(self, brush):
        self._brush = brush
        self.update()
    
    def hoverMoveEvent(self, event):
        top_edge_y = self._rect.height()
        if abs(event.pos().y() - top_edge_y) < 5:
            self.setCursor(Qt.SizeVerCursor)
        else:
            self.setCursor(Qt.ArrowCursor)
        super(InteractiveBar, self).hoverMoveEvent(event)
    
    def mousePressEvent(self, event):
        top_edge_y = self._rect.height()
        if event.button() == Qt.LeftButton:
            if abs(event.pos().y() - top_edge_y) < 5:
                self.dragging = True
                self.drag_offset = event.pos().y() - top_edge_y
            else:
                self.barSelected.emit(self.index)
        super(InteractiveBar, self).mousePressEvent(event)
    
    def mouseMoveEvent(self, event):
        if self.dragging:
            new_top = event.pos().y() - self.drag_offset
            new_height = max(new_top, 0)
            new_rect = QRectF(self._rect.x(), self._rect.y(), self._rect.width(), new_height)
            self.setRect(new_rect)
            self.barDragged.emit(self.index, new_height)
        else:
            super(InteractiveBar, self).mouseMoveEvent(event)
    
    def mouseReleaseEvent(self, event):
        self.dragging = False
        super(InteractiveBar, self).mouseReleaseEvent(event)
    
    def setHeight(self, height):
        """
        Set the height of the bar.
        
        Parameters:
            height (float): The new height value for the bar. Negative values will be clamped to 0.
        """
        new_height = max(height, 0)
        self.prepareGeometryChange()
        # Update the rectangle's height while keeping the x, y, and width unchanged.
        self._rect.setHeight(new_height)
        self.update()
    
    def getXValue(self):
        """
        Retrieve the x value corresponding to this bar.
        """
        return self._rect.x()
    
    def getHeightValue(self):
        """
        Retrieve the current height of the bar.
        """
        return self._rect.height()


class InteractiveHorizontalLine(QGraphicsObject):
    """
    A draggable horizontal line with custom cursor behavior.
    Supports both vertical shifting and rotation around a center point.
    Emits signals when its vertical position changes, rotation changes, or menu actions are triggered.
    """
    lineDragged = pyqtSignal(float)       # emits the new y position
    lineRotated = pyqtSignal(float)       # emits the new angle in radians
    fitToRequested = pyqtSignal()         # emits when 'Fit To' is selected
    resetRequested = pyqtSignal()         # emits when 'Reset' is selected

    def __init__(self, y, x_range, parent=None):
        super(InteractiveHorizontalLine, self).__init__(parent)
        self._y = y
        self.x_range = x_range  # Tuple (xmin, xmax)
        self.setAcceptHoverEvents(True)
        self.dragging = False
        self.drag_offset = 0
        self.pen = QPen(QColor(255, 40, 40), 0.1)
        self.margin = 5  # Sensitivity margin for cursor changes
        self.setFlags(QGraphicsObject.ItemIsMovable | QGraphicsObject.ItemIsSelectable)
        
        # New attributes for rotation
        self.center_x = (x_range[0] + x_range[1]) / 2  # center point x-coordinate
        self.angle = 0  # angle in radians
        self.mode = "shift"  # Default mode: "shift" or "rotate"
        
        # Center point marker
        self.center_marker_size = 3  # size of the center point marker

    def boundingRect(self):
        xmin, xmax = self.x_range
        # Make bounding rect larger to accommodate rotation
        y_range = abs(self.slope() * (xmax - xmin) / 2) + self.margin
        return QRectF(xmin, self._y - y_range, 
                     xmax - xmin, 2 * y_range)

    def paint(self, painter, option, widget):
        xmin, xmax = self.x_range

        if self.mode == "rotate":
            # Change pen color for rotation mode
            self.pen.setColor(QColor(10, 180, 10))
        else:
            # Reset to default color for shift mode
            self.pen.setColor(QColor(180, 8, 8))
        painter.setPen(self.pen)
        
        # Calculate y values at endpoints based on slope
        slope = self.slope()
        y_left = self._y - slope * (self.center_x - xmin)
        y_right = self._y + slope * (xmax - self.center_x)
        
        # Draw the line
        painter.drawLine(QPointF(float(xmin), float(y_left)), QPointF(float(xmax), float(y_right)))
        
        # Draw the center point marker
        #if self.mode == "rotate":
        #    center_marker_pen = QPen(QColor(255, 40, 40), 0.1)
        #    center_marker_brush = QBrush(QColor(255, 40, 40))
        #    painter.setPen(center_marker_pen)
        #    painter.setBrush(center_marker_brush)
        #    painter.drawEllipse(QPointF(self.center_x, self._y), self.center_marker_size/2, self.center_marker_size/2)

    def slope(self):
        """Calculate and return the current slope based on angle."""
        return np.tan(self.angle)
        
    def hoverMoveEvent(self, event):
        pos = event.pos()
        
        # Check if near center point (for potential rotation mode)
        near_center = (abs(pos.x() - self.center_x) < self.center_marker_size and 
                       abs(pos.y() - self._y) < self.center_marker_size)
        
        # Check if near line
        xmin, xmax = self.x_range
        slope = self.slope()
        y_at_x = self._y + slope * (pos.x() - self.center_x)
        near_line = abs(pos.y() - y_at_x) < self.margin
        
        if near_center:
            self.setCursor(Qt.SizeAllCursor)
        elif near_line:
            if self.mode == "shift":
                self.setCursor(Qt.SizeVerCursor)
            else:
                self.setCursor(Qt.PointingHandCursor)
        else:
            self.setCursor(Qt.ArrowCursor)
            
        super(InteractiveHorizontalLine, self).hoverMoveEvent(event)

    def mousePressEvent(self, event):
        pos = event.pos()
        if event.button() == Qt.LeftButton:
            # Check if clicking near center point
            near_center = (abs(pos.x() - self.center_x) < self.center_marker_size and 
                          abs(pos.y() - self._y) < self.center_marker_size)
                          
            # Check if near line
            xmin, xmax = self.x_range
            slope = self.slope()
            y_at_x = self._y + slope * (pos.x() - self.center_x)
            near_line = abs(pos.y() - y_at_x) < self.margin
            
            if near_line or near_center:
                self.dragging = True
                if self.mode == "shift" or near_center:
                    self.drag_offset = pos.y() - self._y
                elif self.mode == "rotate":
                    # Store reference angle for rotation calculations
                    self.ref_x = pos.x()
                    self.ref_y = pos.y()
                    
        super(InteractiveHorizontalLine, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.dragging:
            pos = event.pos()
            if self.mode == "shift":
                # Shift mode: move line up or down
                new_y = pos.y() - self.drag_offset
                self._y = new_y
                self.prepareGeometryChange()
                self.update()
                self.lineDragged.emit(self._y)
            elif self.mode == "rotate":
                downscale = 100  # Adjust this value to control rotation sensitivity
                # Rotate mode: rotate around center point
                #dx = (pos.x() - self.center_x)/downscale
                dy = min((pos.y() - self._y)/downscale, 1.0)
                angle = dy * np.pi
                angle_interval = np.deg2rad(5)  # 5 degrees in radians
                #if abs(angle) < angle_interval:
                #    angle = 0
                #angle_clipped = np.round(angle / angle_interval) * angle_interval

                # Update angle
                self.angle = angle
                self.prepareGeometryChange()
                self.update()
                self.lineRotated.emit(self.angle)
        else:
            super(InteractiveHorizontalLine, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
        super(InteractiveHorizontalLine, self).mouseReleaseEvent(event)

    def contextMenuEvent(self, event):
        """
        Show a context menu on right-click with mode selection and other options.
        """
        menu = QMenu()
        
        # Mode submenu
        mode_menu = menu.addMenu("Mode")
        shift_action = mode_menu.addAction("Shift")
        rotate_action = mode_menu.addAction("Rotate")
        
        # Mark current mode as checked
        shift_action.setCheckable(True)
        rotate_action.setCheckable(True)
        shift_action.setChecked(self.mode == "shift")
        rotate_action.setChecked(self.mode == "rotate")
        
        # Other actions
        fit_action = menu.addAction("Fit To")
        reset_action = menu.addAction("Reset")
        
        # Connect signals
        shift_action.triggered.connect(lambda: self.setMode("shift"))
        rotate_action.triggered.connect(lambda: self.setMode("rotate"))
        fit_action.triggered.connect(self.fitToRequested.emit)
        reset_action.triggered.connect(self.resetRequested.emit)

        # Execute the menu at the global screen position
        menu.exec_(event.screenPos())
        event.accept()

    def setMode(self, mode):
        """Set the interaction mode to 'shift' or 'rotate'."""
        self.mode = mode
        self.update()

    def setY(self, y):
        """Programmatically set the line's y-position."""
        self._y = y
        self.prepareGeometryChange()
        self.update()
        
    def setAngle(self, angle):
        """Programmatically set the line's angle in radians."""
        self.angle = angle
        self.prepareGeometryChange()
        self.update()

    def getY(self):
        return self._y
    
    def getY_atPoint(self, x):
        """Get the Y value at a specific X position."""
        slope = self.slope()
        return self._y + slope * (x - self.center_x)
        
    def getAngle(self):
        return self.angle
    
    def getValueAtX(self, x):
        """Get the Y value at a specific X position based on current angle."""
        slope = self.slope()
        return self._y + slope * (x - self.center_x)