import sys
import json
import os
import numpy as np
from typing import Tuple
from pathlib import Path
from datetime import datetime
import traceback
import cv2 as cv

from PyQt5.QtWidgets import (
    QWidget, QGraphicsScene, QGraphicsView, QApplication, QGridLayout,
    QGraphicsEllipseItem, QGraphicsItem, QTableWidget, QHeaderView, QTableWidgetItem,
    QHBoxLayout, QPushButton, QVBoxLayout, QTabWidget, QLabel, QTextEdit, QComboBox, QCheckBox,
    QFileDialog
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

class ControlPoint(QGraphicsEllipseItem):
    def __init__(self, x, y, id: int, radius: int = 8, color: Tuple[int, int, int] = (255, 0, 0),
                 bezier_widget=None, is_movable: bool = True):
        super(ControlPoint, self).__init__(-radius, -radius, 2 * radius, 2 * radius)
        self.setPos(x, y)
        self.setBrush(QBrush(QColor(*color)))
        self.id = id
        self.bezier_widget = bezier_widget
        if is_movable:
            self.setFlags(QGraphicsItem.ItemIsMovable | QGraphicsItem.ItemIsSelectable)
        else:
            self.setFlags(QGraphicsItem.ItemIsSelectable)

    def mouseReleaseEvent(self, event):
        super(ControlPoint, self).mouseReleaseEvent(event)
        if self.bezier_widget:
            self.bezier_widget.update_curve()


class RacelinePoint(QGraphicsEllipseItem):
    def __init__(self, x, y, id: int, radius: int = 4,
                 color: Tuple[int, int, int] = (255, 255, 255),
                 is_movable: bool = False, tuner = None):
        super(RacelinePoint, self).__init__(-radius, -radius, 2 * radius, 2 * radius)
        self.setPos(x, y)
        self.setBrush(QBrush(QColor(*color)))
        self.id = id
        self.tuner = tuner
        self.is_movable = is_movable
        self.setMovable(is_movable)

    def setMovable(self, is_movable: bool):
        if is_movable:
            self.setFlags(QGraphicsItem.ItemIsMovable | QGraphicsItem.ItemIsSelectable)
        else:
            self.setFlags(QGraphicsItem.ItemIsSelectable)

    def mousePressEvent(self, event):
        super(RacelinePoint, self).mousePressEvent(event)
        if self.tuner and self.tuner.selecting_sector:
            if self.id not in self.tuner.selected_sector_points:
                self.tuner.selected_sector_points.append(self.id)
                print(f"Point {self.id} selected for sector.")
            if len(self.tuner.selected_sector_points) == 2:
                start_idx, end_idx = self.tuner.selected_sector_points
                self.tuner.add_sector(start_idx, end_idx)
                self.tuner.selecting_sector = False
                self.tuner.selected_sector_points = []
                self.tuner.view.setCursor(Qt.ArrowCursor)

    def mouseReleaseEvent(self, event):
        super(RacelinePoint, self).mouseReleaseEvent(event)
        if self.tuner and False:
            start_idx, end_idx = self.tuner.current_sector if self.tuner.current_sector else (None, None)
            if start_idx is not None and self.tuner.point_in_sector(self.id, start_idx, end_idx):
                self.tuner.update_sector_curve()
                self.tuner.dragging_mode = False
                self.tuner.set_sector_points_movable(False)


class SectorContainer:

    def __init__(self, sectors: dict):

        self.sectors = sectors





class RacelineContainer:
    S = 0
    D = 1
    

    def __init__(self,
                 raceline: np.ndarray,
                 inner_track_bound: np.ndarray,
                 outer_track_bound: np.ndarray,
                 k_reg: float = 3,
                 s_reg: float = 1,
                 stepsize_prep: float = 0.05,
                 stepsize_reg: float = 0.2,
                 stepsize_interp_after_opt: float = 0.1,
                 safety_width: float = 0.8,
                 init_pose_ind: int = 0,
                 ggv_file = str(Path(os.environ['LARACE_ROOT'])/"src"/"core"/"config"/"global_planner"/"veh_dyn_info"/"ggv.csv"),
                 ax_max_machines_file = str(Path(os.environ['LARACE_ROOT'])/"src"/"core"/"config"/"global_planner"/"veh_dyn_info"/"ax_max_machines.csv"),
                 vel_profile_conv_filt_window: int = -1 ,
                 dyn_model_exp: float = 1.0,
                 dragcoeff: float = 0.0136,
                 mass: float = 3.518,
                 v_max: float = 15.0,
                 raw_data=None,
                 ):
        
        self.raceline = raceline
        self.inner_track_bound = inner_track_bound
        self.outer_track_bound = outer_track_bound
        self.raw_data = raw_data

        self.interp_params = {
            "k_reg": k_reg,
            "s_reg": s_reg,
            "stepsize_prep": stepsize_prep,
            "stepsize_reg": stepsize_reg,
            "stepsize_interp_after_opt": stepsize_interp_after_opt,
            "safety_width": safety_width,
            "init_pose_ind": init_pose_ind,
            "ggv_file": ggv_file,
            "ax_max_machines_file": ax_max_machines_file,
            "vel_profile_conv_filt_window": vel_profile_conv_filt_window,
            "dyn_model_exp": dyn_model_exp,
            "dragcoeff": dragcoeff,
            "mass": mass,
            "v_max": v_max,
        }

        self.speed_scale = 1.0
        self.map_path = None
        self.map_info = None
        self.map = None
        self._requires_recompute = False
        self.ggv = None
        self.ax_max_machines = None

        self.load_veh_info()

    def __call__(self):
        return np.copy(self.raceline)
    
    def __getitem__(self, index):
        # This method is called when you use obj[index]
        # It supports both integer indexes and slices.
        if isinstance(index, slice):
            # Return a new CustomList for slices
            start_idx = index.start
            end_idx = index.stop
            if start_idx > end_idx:
                return np.concatenate([self.raceline[start_idx:], self.raceline[:end_idx]])
            else:
                return self.raceline[index]
        elif isinstance(index, int):
            return self.raceline[index]
        else:
            raise TypeError("Invalid argument type.")

    def __setitem__(self, index, value):
        # This method is called when you assign via obj[index] = value
        if isinstance(index, slice):
            # Return a new CustomList for slices
            start_idx = index.start
            end_idx = index.stop
            if start_idx > end_idx:
                self.raceline[start_idx:] = value[:(self.raceline.shape[0]-start_idx)]
                self.raceline[:end_idx] = value[(self.raceline.shape[0]-start_idx):]
            else:
                self.raceline[index] = value
        elif isinstance(index, int):
            self.raceline[index] = value
        else:
            raise TypeError("Invalid argument type.")

    def __delitem__(self, index):
        # This method is called when you delete an item with del obj[index]
        del self.raceline[index]

    def __len__(self):
        # Allow usage of len(obj)
        return len(self.raceline)

    def __iter__(self):
        # Allow iteration over the object
        return iter(self.raceline)
    
    @property
    def inner_trackbounds(self):
        return np.copy(self.inner_track_bound)
    @property
    def outer_trackbounds(self):
        return np.copy(self.outer_track_bound)
    


    def insert_points(self, 
                        points: np.ndarray,
                        start_index: int = None, end_index: int= None,
                        recompute: bool = True,
                        ):
        start_index = start_index or 0
        end_index = end_index or self().shape[0]
        if points.shape[0] != end_index - start_index:
            print(f"Please provide the same number of points ({points.shape[0]}) as defined by interval [{start_index}, {end_index}]")
            return
        
        self.raceline[start_index:end_index,2:4] = points
        if recompute:
            self.compute_raceline()
        else:
            self._requires_recompute = True

    def insert_velocity(self,
                        velocities: np.ndarray,
                        start_index: int = None, end_index: int= None,
                        compute_ax_profile: bool = False,
                        ):
        start_index = start_index or 0
        end_index = end_index or self().shape[0]
        end_index = self().shape[0]+end_index+1 if end_index < 0 else end_index
        if velocities.shape[0] != end_index - start_index:
            print(f"Please provide the same number of velocities ({velocities.shape[0]}) as defined by interval [{start_index}, {end_index}]")
            return
        self.raceline[start_index:end_index,4] = velocities

    def load_veh_info(self):
        try:
            self.ggv, self.ax_max_machines = import_veh_dyn_info(
                ggv_import_path=self.interp_params["ggv_file"],
                ax_max_machines_import_path=self.interp_params["ax_max_machines_file"])
        except Exception as e:
            print(f"Error loading vehicle info: {str(e)}")
            traceback.print_exc()
    
    def load_map(self, path: str):
        self.map_path = Path(path)
        info_file_path = self.map_path / (self.map_path.stem + ".yaml")
        try:
            with open(info_file_path, 'r') as file:
                self.map_info = yaml.safe_load(file)
        except Exception as e:
            print(f"Error loading map info: {str(e)}")
            traceback.print_exc()
            return False
        map_file_path = self.map_path / self.map_info["planner_image"]
        try:
            self.map = cv.flip(cv.imread(str(map_file_path), 0), 0)
        except Exception as e:
            print(f"Error loading map: {str(e)}")
            traceback.print_exc()
            return False
        return True

    @classmethod
    def from_raw_data(cls, raw_data):
        raceline, inner_track_bound, outer_track_bound = cls._parse_raw_data(raw_data)
        return cls(raceline, inner_track_bound, outer_track_bound, raw_data=raw_data)
    
    @classmethod
    def _parse_raw_data(cls, raw_data):
        inner_track_bound, outer_track_bound = [], []
        for marker in raw_data["trackbounds_markers"]["markers"]:
            color = np.array((marker['color']['r'], marker['color']['g'], marker['color']['b']))
            if np.all(color == np.array((0.5, 1.0, 0.))):
                inner_track_bound.append([marker["pose"]["position"]["x"],
                                          marker["pose"]["position"]["y"],
                                          marker["pose"]["position"]["z"]])
            else:
                outer_track_bound.append([marker["pose"]["position"]["x"],
                                          marker["pose"]["position"]["y"],
                                          marker["pose"]["position"]["z"]])
        raceline = np.array([
            (wpnt['s_m'], wpnt['d_m'], wpnt['x_m'], wpnt['y_m'], wpnt['vx_mps'], wpnt['kappa_radpm'],
             wpnt['d_right'], wpnt['d_left'])
            for wpnt in raw_data["global_traj_wpnts"]["wpnts"]
        ])
        return raceline, inner_track_bound, outer_track_bound

    @classmethod
    def from_file(cls, file: str):
        try:
            with open(file, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            print(f"Error loading file {file}: {str(e)}")
            traceback.print_exc()
            return None
        return cls.from_raw_data(data)
    
    def load_raw_data(self, raw_data):
        raceline, inner_track_bound, outer_track_bound = self._parse_raw_data(raw_data)
        self.raceline = raceline
        self.inner_track_bound = inner_track_bound
        self.outer_track_bound = outer_track_bound
        self.raw_data = raw_data

    
    def compute_raceline(self, 
                         interpolate_raceline: bool = False,
                         compute_velocity: bool = False,
                         compute_acceleration: bool = False,

                         
                         ):
        
        if self.map is None or self.map_info is None:
            print("Error: To recompute the raceline please provide the RacelineContainer with additional map information.")
            return
        if self.ggv is None or self.ax_max_machines is None:
            print("Error: To recompute the raceline please provide the vehicle model apriori")
            return
        
        points = self.raceline[:,2:4]
        init_pose = np.zeros(3)
        init_pose[:2] = points[self.interp_params["init_pose_ind"]]
        points_px = (points - np.array((self.map_info["origin"][0], self.map_info["origin"][1]))) / self.map_info["resolution"]
        track_bounds_right, track_bounds_left = extract_track_bounds(points_px,
                                                            self.map,
                                                            map_editor_mode=False,
                                                            map_resolution=self.map_info["resolution"],
                                                            map_origin=(self.map_info["origin"][0], self.map_info["origin"][1]),
                                                            initial_position=init_pose,
                                                            show_plots=False)
        points_wd = add_dist_to_cent(centerline_smooth=points_px,
                                     centerline_meter=points,
                                     map_resolution=self.map_info["resolution"],
                                     safety_width=self.interp_params["safety_width"],
                                     show_plots=False,
                                     dist_transform=None,
                                     bound_r=track_bounds_right,
                                     bound_l=track_bounds_left,
                                     reverse=False)
        reftrack_imp = points_wd
        reg_smooth_opts = {
            "k_reg": self.interp_params["k_reg"],
            "s_reg": self.interp_params["s_reg"]
        }
        stepsize_opts = {
            "stepsize_prep": self.interp_params["stepsize_prep"],
            "stepsize_reg": self.interp_params["stepsize_reg"],
            "stepsize_interp_after_opt": self.interp_params["stepsize_interp_after_opt"]
        }
        #import ipdb; ipdb.set_trace()

        #import ipdb; ipdb.set_trace()
        if interpolate_raceline:
            reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = prep_track(
                reftrack_imp=reftrack_imp,
                reg_smooth_opts=reg_smooth_opts,
                stepsize_opts=stepsize_opts,
                debug=False,
                min_width=None)
            alpha_opt = np.zeros(reftrack_interp.shape[0])
            raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, t_vals_opt_interp, s_points_opt_interp, \
            spline_lengths_opt, el_lengths_opt_interp = create_raceline(
                refline=reftrack_interp[:, :2],
                normvectors=normvec_normalized_interp,
                alpha=alpha_opt,
                stepsize_interp=stepsize_opts["stepsize_interp_after_opt"])
        else:
            reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = prep_track_no_interp(
                reftrack_imp=reftrack_imp,
                reg_smooth_opts=reg_smooth_opts,
                stepsize_opts=stepsize_opts,
                debug=False,
                min_width=None)
            alpha_opt = np.zeros(reftrack_interp.shape[0])
            raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, t_vals_opt_interp, s_points_opt_interp, \
            spline_lengths_opt, el_lengths_opt_interp = create_raceline_wout_interp(refline=reftrack_interp[:, :2])
        
        psi_vel_opt, kappa_opt = calc_head_curv_an(
            coeffs_x=coeffs_x_interp,
            coeffs_y=coeffs_y_interp,
            ind_spls=spline_inds_opt_interp,
            t_spls=t_vals_opt_interp)
        
        if compute_velocity or interpolate_raceline:

            vx_profile_opt = calc_vel_profile(
                ggv=self.ggv,
                ax_max_machines=self.ax_max_machines,
                v_max=self.interp_params["v_max"],
                kappa=kappa_opt,
                el_lengths=el_lengths_opt_interp,
                closed=True,
                filt_window=self.interp_params["vel_profile_conv_filt_window"] if self.interp_params["vel_profile_conv_filt_window"]>0 else None,
                dyn_model_exp=self.interp_params["dyn_model_exp"],
                drag_coeff=self.interp_params["dragcoeff"],
                m_veh=self.interp_params["mass"])
        else:
            vx_profile_opt = self.raceline[:,4]
        
        vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
        if compute_acceleration or interpolate_raceline:
            ax_profile_opt = calc_ax_profile(
                vx_profile=vx_profile_opt_cl,
                el_lengths=el_lengths_opt_interp,
                eq_length_output=False)
        else:
            ax_profile_opt = self.raceline[:,5]

        estimated_lap_time = calc_t_profile(
            vx_profile=vx_profile_opt,
            ax_profile=ax_profile_opt,
            el_lengths=el_lengths_opt_interp)
        estimated_lap_time = estimated_lap_time[-1]
        trajectory_opt = np.column_stack((s_points_opt_interp,
                                           raceline_interp,
                                           psi_vel_opt,
                                           kappa_opt,
                                           vx_profile_opt,
                                           ax_profile_opt))
        
        spline_data_opt = np.column_stack((spline_lengths_opt, coeffs_x_opt, coeffs_y_opt))
        #import ipdb; ipdb.set_trace()
        closing_dist = np.linalg.norm(np.sum(spline_data_opt[:, 0]) - trajectory_opt[-1, 0])
        if closing_dist < 5e-2:
            self.global_trajectory = np.vstack((trajectory_opt[:-1], trajectory_opt[0, :]))
        else:
            self.global_trajectory = np.vstack((trajectory_opt, trajectory_opt[0, :]))
        
        self.global_trajectory[-1, 0] = np.sum(spline_data_opt[:, 0])

        traj_d_right, traj_d_left = dist_to_bounds(
            trajectory=self.global_trajectory,
            bound_r=track_bounds_right,
            bound_l=track_bounds_left,
            centerline=points,
            safety_width=self.interp_params['safety_width'],
            show_plots=False,
            reverse=False)
        
        map_info_str = ''
        map_info_str += f'IQP estimated lap time: {np.round(estimated_lap_time, 4)}s; '
        map_info_str += f'IQP maximum speed: {np.max(self.global_trajectory[:, 5]):.4f}m/s; '

        self.raw_data = write_waypoints(
            map_info = map_info_str,
            est_lap_time = estimated_lap_time,
            centerline = points_wd,
            global_trajectory = self.global_trajectory,
            d_right = traj_d_right,
            d_left = traj_d_left,
            bounds_left = track_bounds_left,
            bounds_right = track_bounds_right,
            return_save_dict = True
        )
        self.load_raw_data(self.raw_data)
        self._requires_recompute = False

    def save(self, path: str):
        #import ipdb; ipdb.set_trace()
        try:
            with open(path, 'w') as f:
                json.dump(self.raw_data, f)
        except Exception as e:
            print(f"Error saving file {path}: {str(e)}")

    


class RacelineChange:

    def __init__(self,
                  start_index: int,
                  end_index: int,
                  change: np.ndarray,
                  dim: int = None):
        self.start_index = start_index
        self.end_index = end_index
        self.change = change
        self.dim = dim

    def __call__(self):
        return self.change
    