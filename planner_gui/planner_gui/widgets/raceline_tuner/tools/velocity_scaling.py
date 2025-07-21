from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QSlider,
    QGraphicsRectItem, QComboBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPen, QBrush, QColor
import pyqtgraph as pg
import numpy as np
from scipy.interpolate import make_interp_spline  # For cubic B-Spline interpolation

from styles import *
from widgets.raceline_tuner.custom import InteractiveBar, InteractiveHorizontalLine
from widgets.raceline_tuner.structs import RacelineChange

class VelocityScalingWidget(QWidget):
    globalScaleChanged = pyqtSignal()
    
    def __init__(self, parent_tuner):
        super(VelocityScalingWidget, self).__init__()
        self.parent_tuner = parent_tuner
        # NEW: Define number of extra waypoints to display on each side of the sector.
        self.n_waypoints = 10

        # Containers for interactive bars and vx/steering masking items.
        self.interactive_bars = []
        self.vx_mask_items = []
        self.sa_mask_items = []  # For steering angle plot mask items
        self.raceline_changes = []

        self.mode = None
        self.current_speed_scale = 1.0

        # Main vertical layout for the widget.
        main_layout = QVBoxLayout(self)
        
        # === Header ===


        header_widget = QWidget()
        header_layout = QHBoxLayout(header_widget)
        header_label = QLabel("Velocity Scaling")
        header_label.setFixedHeight(20)
        header_layout.addWidget(header_label)

        self.insertion_mode_dropdown = QComboBox()
        self.insertion_mode_dropdown.addItem("None")
        self.insertion_mode_dropdown.addItem("Linear Ramp")
        self.insertion_mode_dropdown.setFixedHeight(20)
        header_layout.addWidget(self.insertion_mode_dropdown)

        # NEW: Insert button added before the compute button.
        insert_button = QPushButton("Insert")
        insert_button.setStyleSheet(button_thin_stylesheet)
        insert_button.setFixedHeight(20)
        insert_button.clicked.connect(self.insert)
        header_layout.addWidget(insert_button)
        # Compute button
        compute_button = QPushButton("Compute")
        compute_button.setStyleSheet(button_thin_stylesheet)
        compute_button.setFixedHeight(20)
        compute_button.clicked.connect(self.compute)
        header_layout.addWidget(compute_button)
        # Accept button
        accept_button = QPushButton("Accept")
        accept_button.setStyleSheet(button_thin_stylesheet)
        accept_button.setFixedHeight(20)
        accept_button.clicked.connect(self.accept)
        header_layout.addWidget(accept_button)
        
        
        # --- Group top widgets into one container and make it fixed-height ---
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.addWidget(header_widget)
        top_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(top_widget, 0)
        
        # === Central Container with Two Columns ===
        scaling_container = QWidget()
        scaling_layout = QVBoxLayout(scaling_container)
        scaling_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Toolbar with buttons: Reset, Linear, B-Spline, smooth.
        self.toolbar_widget = QWidget()
        toolbar_layout = QHBoxLayout(self.toolbar_widget)
        reset_button = QPushButton("Reset")
        reset_button.setStyleSheet(button_thin_stylesheet)
        reset_button.setFixedHeight(20)
        reset_button.clicked.connect(self.reset_velocities)
        toolbar_layout.addWidget(reset_button)
        self.linear_button = QPushButton("Linear")
        self.linear_button.setStyleSheet(button_thin_stylesheet)
        self.linear_button.setFixedHeight(20)
        self.linear_button.clicked.connect(self.setLinear)
        toolbar_layout.addWidget(self.linear_button)
        self.bspline_button = QPushButton("B-Spline")
        self.bspline_button.setStyleSheet(button_thin_stylesheet)
        self.bspline_button.setFixedHeight(20)
        self.bspline_button.clicked.connect(self.setBSpline)
        toolbar_layout.addWidget(self.bspline_button)
        self.smooth_button = QPushButton("smooth")
        self.smooth_button.setStyleSheet(button_thin_stylesheet)
        self.smooth_button.setFixedHeight(20)
        self.smooth_button.clicked.connect(self.setSmooth)
        toolbar_layout.addWidget(self.smooth_button)
        # NEW: Make the toolbar a thin header.
        self.toolbar_widget.setFixedHeight(30)
        scaling_layout.addWidget(self.toolbar_widget, 0)
        
        # Bar plot widget (below the toolbar)
        self.bar_plot_widget = pg.GraphicsLayoutWidget()
        self.bar_plot_widget.setStyleSheet(bar_plot_stylesheet)
        self.bar_plot = self.bar_plot_widget.addPlot()
        self.bar_plot_widget.setBackground(container_bg_color)
        self.bar_plot.getViewBox().setLimits(yMin=0)
        self.bar_plot.disableAutoRange(axis=pg.ViewBox.YAxis)
        self.bar_plot.setMouseEnabled(x=False, y=False)
        self.bar_plot.setLabel('bottom', 'Waypoint Index')
        self.bar_plot.setLabel('left', 'Speed [m/s]')
        # Expand this plot to fill remaining space
        scaling_layout.addWidget(self.bar_plot_widget, 1)
        scaling_layout.setStretch(0, 0)
        scaling_layout.setStretch(1, 1)
        
        # Add the central container, letting it grab all leftover vertical space
        main_layout.addWidget(scaling_container, 1)
        
        self.setLayout(main_layout)
        
        # === Connections ===
        self.parent_tuner.sectorChanged.connect(self.onSectorChange)
        #self.gui_timer = QTimer()
        #self.gui_timer.timeout.connect(self.update_data)
        #self.gui_timer.start(int(1000 / 10))

    # ----- Sector Change Slot: Update x-range and y-range for the speed plot -----
    @pyqtSlot()
    def onSectorChange(self):

        self.update_bar_plot()
    
    # ----- Toolbar Button Callbacks -----
    def setLinear(self):
        print("Linear mode selected")
        self.mode = 'linear'
        # Remove any existing interactive line.
        if hasattr(self, 'linear_line'):
            self.bar_plot.removeItem(self.linear_line)
            del self.linear_line
            return
        
        raceline = self.parent_tuner.raceline()
        if raceline.size > 0:
            current_profile = raceline[:, self.parent_tuner.raceline.VX_INDEX]
            baseline = float(np.median(current_profile))
        else:
            baseline = 0
            current_profile = np.array([])
        self.linear_baseline = baseline
        self.original_profile = np.copy(current_profile)
        # Use the current x-range from the bar plot as the horizontal extent.
        x_range = self.bar_plot.viewRange()[0]
        self.linear_line = InteractiveHorizontalLine(baseline, x_range)
        self.linear_line.lineDragged.connect(self.linear_line_moved)
        self.linear_line.lineRotated.connect(self.linear_line_rotated)  # Add connection to rotation signal
        self.linear_line.resetRequested.connect(self.reset_velocities)
        self.linear_line.fitToRequested.connect(self.fit_to_line)
        self.bar_plot.addItem(self.linear_line)
        
    def setBSpline(self):
        print("B-Spline mode selected")
        self.mode = 'bspline'
        if hasattr(self, 'linear_line'):
            self.bar_plot.removeItem(self.linear_line)
            del self.linear_line
        raceline = self.parent_tuner.raceline()
        arc_lengths = raceline[:, self.parent_tuner.raceline.S_INDEX]
        velocities = raceline[:, self.parent_tuner.raceline.VX_INDEX]
        #self.update_bar_plot()
    
    def setSmooth(self):
        print("Smooth mode selected")
        self.mode = 'smooth'
        self.update_bar_plot()
    
    # ----- Reset Button Callback -----
    def reset_velocities(self):
        """Reset the bar plot velocities to the original raceline values."""
        raceline = self.parent_tuner.raceline()
        current_profile = raceline[:, self.parent_tuner.raceline.VX_INDEX]
        for bar in self.interactive_bars:
            bar.setHeight(current_profile[bar.index])
        
        self.update_parent_vx_plot()

    
    # ----- Slider Update -----
    #def update_slider_label(self, value):
    #    float_value = value / 1000.0
    #    self.speed_scale = float_value
    #    self.speed_scale_label.setText(f"{float_value:.3f}")
    def fit_to_line(self):
        """Set all interactive bar heights to the current line Y-value."""
        #new_y = self.linear_line.getY()
                # Determine active bars (within sector)

        # Update the UI bars
        for bar in self.interactive_bars:
            if bar.active:
                x = bar.getXValue()
                new_y = self.linear_line.getY_atPoint(x)
                bar.setHeight(new_y)

        self.update_parent_vx_plot()
        

    # ----- Bar Plot Update -----
    def clear_plot(self):
        self.bar_plot.clear()
        
    def update_bar_plot(self, raceline=None):
        self.bar_plot.clear()
        self.interactive_bars = []
        self.bar_plot_widget.setBackground(container_bg_color)
        sector = self.parent_tuner.current_sector  # (start_idx, end_idx)

        raceline = self.parent_tuner.raceline() if raceline is None else raceline
        total_points = raceline.shape[0]
        if sector is None:
            start_idx, end_idx = 0, total_points - 1
        else:
            start_idx, end_idx = sector
 
        n = self.n_waypoints
        disp_start_idx = max(0, start_idx - n)
        disp_end_idx = min(total_points - 1, end_idx + n)
        
        # Create display indices with special handling for wrap-around case
        is_wrapped = start_idx > end_idx
        
        if is_wrapped:
            disp_indices_first = np.arange(disp_start_idx, total_points, dtype=int)
            disp_indices_second = np.arange(0, disp_end_idx + 1, dtype=int)
            disp_indices = np.concatenate([disp_indices_first, disp_indices_second])
            # Create a continuous index space for visualization
            plot_indices_first = disp_indices_first
            plot_indices_second = disp_indices_second + total_points
            plot_indices = np.concatenate([plot_indices_first, plot_indices_second])
        else:
            disp_indices = np.arange(disp_start_idx, disp_end_idx + 1)
            plot_indices = disp_indices
        
        num_samples = 1000
        if len(disp_indices) > num_samples:
            if is_wrapped:
                # Maintain relative spacing in plot indices for both segments
                ratio = len(disp_indices_first) / len(disp_indices)
                subsample_count_first = max(1, int(ratio * num_samples))
                subsample_count_second = max(1, num_samples - subsample_count_first)
                
                indices_first = np.linspace(0, len(disp_indices_first)-1, subsample_count_first, dtype=int)
                indices_second = np.linspace(0, len(disp_indices_second)-1, subsample_count_second, dtype=int)
                
                subsample_indices = np.concatenate([
                    disp_indices_first[indices_first],
                    disp_indices_second[indices_second]
                ])
                plot_subsample_indices = np.concatenate([
                    plot_indices_first[indices_first],
                    plot_indices_second[indices_second]
                ])
            else:
                sample_indices = np.linspace(0, len(disp_indices)-1, num_samples, dtype=int)
                subsample_indices = disp_indices[sample_indices]
                plot_subsample_indices = plot_indices[sample_indices]
        else:
            subsample_indices = disp_indices
            plot_subsample_indices = plot_indices
        
        current_profile = raceline[:, self.parent_tuner.raceline.VX_INDEX]
        active_bar_color = QColor(0, 180, 255)
        bar_width = 0.5
        
        # Determine active bars (within sector)
        def is_active(idx):
            if start_idx <= end_idx:
                return start_idx <= idx <= end_idx
            else:
                return idx >= start_idx or idx <= end_idx
                
        for i, (idx, plot_idx) in enumerate(zip(subsample_indices, plot_subsample_indices)):
            active = is_active(idx)
            bar_color = active_bar_color if active else QColor(120, 120, 120)
            # Use plot_idx for visualization position, but idx for data reference
            bar = InteractiveBar(plot_idx, 0, bar_width, current_profile[idx], index=idx, active=active)
            bar.setBrush(QBrush(bar_color))
            bar.setZValue(10)
            if self.mode == 'bspline':
                bar.setFlag(QGraphicsRectItem.ItemIsMovable, True)
                bar.barDragged.connect(lambda idx, new_height, b=bar: self.bspline_bar_dragged(b, new_height))
            else:
                bar.setFlag(QGraphicsRectItem.ItemIsMovable, False)
                bar.barSelected.connect(lambda idx: self.parent_tuner.mark_point(idx))
                bar.barDragged.connect(lambda idx, new_height: self.parent_tuner.mark_point(idx))
            self.bar_plot.addItem(bar)
            self.interactive_bars.append(bar)
        
        # Set x-range to show continuous visualization even when wrapped
        if is_wrapped:
            self.bar_plot.setXRange(np.min(plot_subsample_indices), np.max(plot_subsample_indices), padding=0)
        else:
            self.bar_plot.setXRange(np.min(subsample_indices), np.max(subsample_indices), padding=0)
            
        self.bar_plot.showGrid(x=False, y=True)
        data_values = current_profile[subsample_indices]
        y_min = 0 
        y_max = np.max(data_values) + 2
        self.bar_plot.setYRange(y_min, y_max, padding=0)

        self.update_parent_vx_plot()

    # ----- Linear Mode: Horizontal Line Drag Callback -----
    def linear_line_moved(self, new_y):
        delta = new_y - self.linear_baseline
        sector = self.parent_tuner.current_sector
        if sector is not None:
            start_idx, end_idx = sector
            def is_active(idx):
                if start_idx <= end_idx:
                    return start_idx <= idx <= end_idx
                else:
                    return idx >= start_idx or idx <= end_idx
        else:
            is_active = lambda idx: True

        for bar in self.interactive_bars:
            if is_active(bar.index):
                new_height = self.original_profile[bar.index] + delta
            else:
                new_height = self.original_profile[bar.index]
            bar.setHeight(new_height)

        self.update_parent_vx_plot()

    def linear_line_rotated(self, angle):
        """Handle rotation of the linear line by applying proportional changes to bar heights."""
        sector = self.parent_tuner.current_sector
        if sector is not None:
            start_idx, end_idx = sector
            def is_active(idx):
                if start_idx <= end_idx:
                    return start_idx <= idx <= end_idx
                else:
                    return idx >= start_idx or idx <= end_idx
        else:
            is_active = lambda idx: True
        
        # Get the center X position
        center_x = self.linear_line.center_x
        
        # Get the slope of the line
        slope = np.tan(angle)
        
        for bar in self.interactive_bars:
            if is_active(bar.index):
                # Calculate height adjustment based on line slope and x-offset from center
                x_offset = bar.getXValue() - center_x
                delta_y = slope * x_offset
                
                # Apply the adjustment to original profile
                new_height = self.original_profile[bar.index] + delta_y
                # Ensure height doesn't go below zero
                new_height = max(0, new_height)
                
                bar.setHeight(new_height)
        
        self.update_parent_vx_plot()

    def update_parent_vx_plot(self):

        raceline = self.parent_tuner.raceline()
        new_velocity = raceline[:, self.parent_tuner.raceline.VX_INDEX]

        bars_sorted = sorted(self.interactive_bars, key=lambda b: b.index)
        for bar in bars_sorted:
            x_val = int(bar.index)
            new_velocity[x_val] = bar.getHeightValue()

        arc_lengths = raceline[:, self.parent_tuner.raceline.S_INDEX]
        self.parent_tuner.set_changed_vx_plot(arc_lengths, new_velocity)

    
    # ----- BSpline Mode: Bar Drag Callback -----
    def bspline_bar_dragged(self, bar, new_height):
        sector = self.parent_tuner.current_sector
        if sector is not None:
            start_idx, end_idx = sector
            def is_active(idx):
                if start_idx <= end_idx:
                    return start_idx <= idx <= end_idx
                else:
                    return idx >= start_idx or idx <= end_idx
        else:
            is_active = lambda idx: True

        if not is_active(bar.index):
            return  # Do not modify buffer points
        self.bspline_control_points[bar.index] = new_height
        x = np.arange(len(self.bspline_control_points))
        y = self.bspline_control_points
        try:
            spline = make_interp_spline(x, y, k=3)
            y_spline = spline(x)
        except Exception as e:
            print("Error computing B-Spline:", e)
            return
        for b in self.interactive_bars:
            if is_active(b.index):
                new_val = y_spline[b.index]
            else:
                new_val = self.bspline_control_points[b.index]
            b.setHeight(new_val)
    
    # ----- Insert and Compute Callbacks -----
    def insert(self):
        """Insert velocities based on the selected insertion mode.
           This function takes over the functionalities of the compute button and applies
           different insertion logic based on the dropdown selection.
        """
        mode = self.insertion_mode_dropdown.currentText()
        if mode.lower() == "none":
            self.insert_raw()
        elif mode.lower() == "linear ramp":
            self.insert_linear_ramp()
        else:
            # Fallback to compute if other modes are added later
            self.compute()
        # --- Modification #1: Update the bar plot with the new values computed ---
        #import ipdb; ipdb.set_trace()
        change = self.raceline_changes[-1]
        raceline = self.parent_tuner.raceline()
        raceline[:,self.parent_tuner.raceline.VX_INDEX] = change()
        self.update_bar_plot(raceline)

    def insert_raw(self):
        
        sector = self.parent_tuner.current_sector
        if sector is None:
            start_idx, end_idx = 0, len(self.parent_tuner.raceline()) - 1
        else:
            start_idx, end_idx = sector

        raceline = self.parent_tuner.raceline()
        new_velocity = raceline[:, self.parent_tuner.raceline.VX_INDEX]
        # TODO: Fix this for wrapping
        bars_sorted = sorted(self.interactive_bars, key=lambda b: b.index)
        for bar in bars_sorted:
            x_val = int(bar.index)
            new_velocity[x_val] = bar.getHeightValue()

        self.raceline_changes.append(RacelineChange(0, -1, new_velocity, 7))

    
    def insert_linear_ramp(self):
        """Insertion mode: linear ramp.
           For the waypoints before and after the current sector (displayed as sector ± n waypoints),
           perform a linear interpolation (ramp) between a reference point and the sector boundary.
           For the start side, take the first waypoint going backward with a velocity difference under a threshold;
           if none is found, take the one with the smallest difference. Do the same for the end side.
        """
        sector = self.parent_tuner.current_sector
        raceline = self.parent_tuner.raceline()
        total_points = raceline.shape[0]
        if sector is None:
            start_idx, end_idx = 0, total_points - 1
        else:
            start_idx, end_idx = sector
        start_idx, end_idx = sector
  
        n = self.n_waypoints
        x_positions = raceline[:, self.parent_tuner.raceline.S_INDEX]
        
        if not self.interactive_bars:
            return
        def is_active(idx):
            if start_idx <= end_idx:
                return start_idx <= idx <= end_idx
            else:
                return idx >= start_idx or idx <= end_idx
        
        bars_sorted = sorted(self.interactive_bars, key=lambda b: b.index)
        x_vals, x_vals_before, x_vals_after = [], [], []
        ind_vals, ind_vals_before, ind_vals_after = [], [], []
        y_vals, y_vals_before, y_vals_after = [], [], []
        #import ipdb; ipdb.set_trace()
        new_velocity = np.copy(raceline[:, self.parent_tuner.raceline.VX_INDEX])
        for bar in bars_sorted:
            x_val = x_positions[bar.index]
            if is_active(bar.index):
                y_val = bar.getHeightValue()
                x_vals.append(x_val)
                y_vals.append(y_val)
                ind_vals.append(bar.index)
            elif bar.index < start_idx:
                y_val = raceline[int(bar.index), self.parent_tuner.raceline.VX_INDEX]
                x_vals_before.append(x_val)
                y_vals_before.append(y_val)
                ind_vals_before.append(bar.index)
            else:
                y_val = raceline[int(bar.index), self.parent_tuner.raceline.VX_INDEX]
                x_vals_after.append(x_val)
                y_vals_after.append(y_val)
                ind_vals_after.append(bar.index)

        x_vals, x_vals_before, x_vals_after = np.array(x_vals), np.array(x_vals_before), np.array(x_vals_after)
        y_vals,y_vals_before, y_vals_after = np.array(y_vals), np.array(y_vals_before), np.array(y_vals_after)
        
        for x, y in zip(ind_vals, y_vals):
            new_velocity[x] = y


        start_val, end_val = y_vals[0], y_vals[-1]

        min_ind_before = np.argmin(np.abs(y_vals_before-start_val))
        min_ind_after = np.argmin(np.abs(y_vals_after-end_val))
        if min_ind_before <= 2 and start_val - y_vals_before[-1] > 0.3:
            min_ind_before = 5
        if len(y_vals_before)-min_ind_before >= 1:
            m = (start_val - y_vals_before[min_ind_before]) / (x_vals[0] - x_vals_before[min_ind_before])
            b = y_vals_before[min_ind_before] - m * x_vals_before[min_ind_before]
            indices_between = np.arange(ind_vals_before[min_ind_before], ind_vals_before[-1]+1)
            indices_abs = np.arange(min_ind_before, len(x_vals_before)) # TODO: this hsouldnt be plus one but has to
            for ind_abs, ind in zip(indices_abs, indices_between):
                new_velocity[ind] = m * x_vals_before[ind_abs] + b
        
        #import ipdb; ipdb.set_trace()
        if min_ind_after <= 2 and end_val - y_vals_after[0] > 0.3:
            min_ind_after = 5
        if min_ind_after >= 1:
            m = (end_val - y_vals_after[min_ind_after]) / (x_vals[-1] - x_vals_after[min_ind_after])
            b = y_vals_after[min_ind_after] - m * x_vals_after[min_ind_after]
            indices_between = np.arange(ind_vals_after[0], ind_vals_after[min_ind_after]+1)
            indices_abs = np.arange(0, min_ind_after)
            for ind_abs, ind in zip(indices_abs, indices_between):
                new_velocity[ind] = m * x_vals_after[ind_abs] + b


        #indices = ind_vals_before + ind_vals + ind_vals_after
        self.raceline_changes.append(RacelineChange(0, -1, new_velocity, 7))
        
    
    # ----- Compute Callback (Existing Functionality) -----
    def compute(self):
        self.insert()
    
    # ----- Accept Callback -----
    def accept(self):
        # --- Modification #2: Forward the new velocities to the raceline and update the main view ---
        if len(self.raceline_changes) == 0:
            self.insert_raw()
        for change in self.raceline_changes:
            self.parent_tuner.raceline.set_velocity(change(), change.start_index, change.end_index)

        #self.parent_tuner.raceline.compute_raceline(compute_acceleration=True)
        self.raceline_changes = []
        self.parent_tuner.save()
        self.parent_tuner.update_scene()
