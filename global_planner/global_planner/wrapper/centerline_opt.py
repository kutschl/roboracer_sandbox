import os, sys, shutil
from pathlib import Path
import cv2 as cv
import yaml
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import importlib
import traceback
from matplotlib.patches import FancyArrowPatch
import io
from typing import Literal
import yaml

import numpy as np
from PIL import Image
from skimage.morphology import skeletonize

from global_planner.trajectory_planning_helpers.import_veh_dyn_info import import_veh_dyn_info
from global_planner.utils import extract_centerline, smooth_centerline, add_dist_to_cent, extract_track_bounds, dist_to_bounds, write_centerline, write_waypoints, write_global_trajectory_to_csv
from global_planner.gco.helper_funcs_glob.interp_track import interp_track
from global_planner.gco.trajectory_optimizer import trajectory_optimizer
from global_planner.trajectory_planning_helpers.calc_head_curv_num import calc_head_curv_num
from global_planner.trajectory_planning_helpers.create_raceline import create_raceline
from global_planner.trajectory_planning_helpers.calc_ax_profile import calc_ax_profile
from global_planner.trajectory_planning_helpers.calc_t_profile import calc_t_profile
from global_planner.trajectory_planning_helpers.calc_vel_profile import calc_vel_profile
from global_planner.gco.helper_funcs_glob.prep_track import prep_track
from global_planner.trajectory_planning_helpers.calc_head_curv_an import calc_head_curv_an
from global_planner.gco.helper_funcs_glob.check_traj import check_traj
from global_planner.custom.racetrack_spline import racetrack_spliner


class CenterlineTrajectoryOptimizer:

    def __init__(self,
                 map_path: str,
                 traj_name: Literal["min_curve","min_time"] = "centerline",
                 visualize_results: bool = False
                 ):
        
        self.map_path = map_path
        self.traj_name = traj_name
        self.visualize_results = visualize_results
        self.output_path = Path(self.map_path) / "plans" 
        self.centerline_path = str(self.output_path / 'iqp_centerline')
        
        self.map = None
        self.map_info = None
        self.centerline_path = None
        self.ggv = None
        self.ax_max_machines = None

        self.parameters = {
            "centerline_length": 0.0,
            "safety_width": 0.8,
            "init_pose_ind": 0,
            "smoothing_iters": 4,
            "filter_size_ratio": 0.09,
            "filter_size_iter_scale": 1.0,
            "stepsize_prep": 0.05,
            "stepsize_reg": 0.2,
            "stepsize_interp_after_opt": 0.1,
            "k_reg": 3,
            "s_reg": 1,
            "v_max": 15.0,
            "dragcoeff": 0.0136,
            "mass": 3.518,
            "curvlim": 1.0,
            "length": 0.52,
            "width": 0.3,
            "dyn_model_exp": 1.0,
            "vel_profile_conv_filt_window": -1,
            "map_editor_mode": False,
            "reverse_mapping": False,
            "ggv_file": str(Path(os.environ['LARACE_ROOT'])/"src"/"core"/"config"/"global_planner"/"veh_dyn_info"/"ggv.csv"),
            "ax_max_machines_file": str(Path(os.environ['LARACE_ROOT'])/"src"/"core"/"config"/"global_planner"/"veh_dyn_info"/"ax_max_machines.csv")
        }

        self.load_map(self.map_path)
        self.load_veh_info()


    def load_map(self, map_path: str):
        self.map_path = Path(map_path)
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
            self.map = cv.flip(cv.imread(map_file_path, 0), 0)
            #self.map = cv.imread(map_file_path, 0)
        except Exception as e:
            print(f"Error loading map: {str(e)}")
            traceback.print_exc()
            return False
        print("Map loaded successfully.")
        return True
    
    def load_veh_info(self):
        try:
            self.ggv, self.ax_max_machines = import_veh_dyn_info(ggv_import_path=self.parameters["ggv_file"],
                                ax_max_machines_import_path=self.parameters["ax_max_machines_file"])
        except Exception as e:
            print(f"Error loading vehicle info: {str(e)}")
            traceback.print_exc()

    def skeletonize(self):
        try:
            skeleton_raw = skeletonize((self.map>0).astype(np.uint8), method='lee')
            skeleton_raw = skeleton_raw > 0
            self.skeleton = skeleton_raw.astype(np.uint8)*255
            skeleton_vis = cv.cvtColor(self.map, cv.COLOR_GRAY2RGB)
            skeleton_vis[skeleton_raw] = (225, 55, 55)
        except Exception as e:
            print(f"Error skeletonizing map: {str(e)}")
            traceback.print_exc()
            return False
        
        try:
            self.skeleton_vis = skeleton_vis
            pil_image = Image.fromarray(self.skeleton_vis)
            pil_image.save(self.output_path / 'skeleton.png')
        except Exception as e:
            print(f"Error saving skeleton image: {str(e)}")
            traceback.print_exc()
            return False
        
        return True
    
    def extract_track_bounds(self):
        init_pose = np.zeros(3)
        init_pose[:2] = self.centerline_smooth[self.parameters["init_pose_ind"]]

        # convert centerline from cells to meters
        self.centerline_meter = np.zeros(np.shape(self.centerline_smooth))
        self.centerline_meter[:, 0] = self.centerline_smooth[:, 0] * self.map_info["resolution"] + self.map_info["origin"][0]
        self.centerline_meter[:, 1] = self.centerline_smooth[:, 1] * self.map_info["resolution"] + self.map_info["origin"][1]

        # interpolate centerline to 0.1m stepsize: less computation needed later for distance to track bounds
        self.centerline_meter = np.column_stack((self.centerline_meter, np.zeros((self.centerline_meter.shape[0], 2))))

        self.centerline_meter_int = interp_track(reftrack=self.centerline_meter, stepsize_approx=0.1)[:, :2]

        # get distance to initial position for every point on centerline
        cent_distance = np.sqrt(np.power(self.centerline_meter_int[:, 0] - init_pose[0], 2)
                                + np.power(self.centerline_meter_int[:, 1] - init_pose[1], 2))

        min_dist_ind = np.argmin(cent_distance)

        cent_direction = np.angle([complex(self.centerline_meter_int[min_dist_ind, 0] -
                                            self.centerline_meter_int[min_dist_ind - 1, 0],
                                            self.centerline_meter_int[min_dist_ind, 1] -
                                            self.centerline_meter_int[min_dist_ind - 1, 1])])



        self.track_bounds_right, self.track_bounds_left = extract_track_bounds(self.centerline_smooth,
                                                            self.map,
                                                            map_editor_mode=False,
                                                            map_resolution=self.map_info["resolution"],
                                                            map_origin=(self.map_info["origin"][0], self.map_info["origin"][1]),
                                                            initial_position=init_pose,
                                                            show_plots=False)
        

        self.centerline_with_dist = add_dist_to_cent(centerline_smooth=self.centerline_smooth,
                                            centerline_meter=self.centerline_meter_int,
                                            map_resolution=self.map_info["resolution"],
                                            safety_width=self.parameters["safety_width"],
                                            show_plots=False,
                                            dist_transform=None,
                                            bound_r=self.track_bounds_right,
                                            bound_l=self.track_bounds_left,
                                            reverse=False)

        self.centerline_path = str(self.output_path / 'iqp_centerline')
        write_centerline(self.centerline_with_dist, self.centerline_path)
        
        try:
            fig = plt.figure(figsize=(15, 10))
            ax = fig.add_subplot(1, 1, 1)
            ax.plot(self.centerline_meter[:, 0], self.centerline_meter[:, 1], color='black', label='Centerline')
            ax.plot(self.track_bounds_right[:, 0], self.track_bounds_right[:, 1], color='blue', label='Right Bound')
            ax.plot(self.track_bounds_left[:, 0], self.track_bounds_left[:, 1], color='green', label='Left Bound')
            ax.legend()
            if self.visualize_results:
                plt.savefig(self.output_path / 'track_bounds.png', bbox_inches='tight', pad_inches=0.1)

            # print pyplot figure to image
            buf = io.BytesIO()
            plt.savefig(buf, bbox_inches='tight', pad_inches=0.1)
            buf.seek(0)
            pil_image = Image.open(buf)
            pil_image = pil_image.convert('RGB')
            self.track_bounds_vis = np.array(pil_image)
            buf.close()
        except Exception as e:
            print(f"Error saving track bounds image: {str(e)}")
            traceback.print_exc()
            return False
        
        return True
    
    def interpolate_raceline(self):
        output_path = self.output_path / 'interpolated'
        os.makedirs(output_path, exist_ok=True)
        racetrack_spliner(
            self.centerline_meter[:,:2],
            self.track_bounds_left,
            self.track_bounds_right,
            output_path=output_path,
            initial_downsample=64,
        )

    
    def compute_centerline(self):
        try:
            self.centerline = extract_centerline(
                    skeleton=self.skeleton,
                    cent_length=self.parameters["centerline_length"],
                    map_resolution=self.map_info["resolution"],
                    map_editor_mode=self.parameters["map_editor_mode"])
            self.centerline = self.centerline[::-1] if self.parameters["reverse_mapping"] else self.centerline
            self.centerline_smooth = np.copy(self.centerline)
            print(f"# Center line points: {self.centerline.shape[0]}")

            filter_size = min(max(20, int(self.centerline.shape[0] * self.parameters["filter_size_ratio"])), 60)
            for i in range(self.parameters["smoothing_iters"]):
                print(f"Smoothing iteration {i+1}/{self.parameters['smoothing_iters']}, filter size: {filter_size}")
                self.centerline_smooth = smooth_centerline(self.centerline_smooth, force_filter_length=filter_size)
                filter_size = int(filter_size*self.parameters["filter_size_iter_scale"])
                

            self.centerline_vis = np.copy(self.map)
            self.centerline_vis = self.centerline_vis[...,np.newaxis].repeat(3, axis=-1)
            for p in self.centerline:
                self.centerline_vis[int(p[1]), int(p[0])] = (0, 0, 255)
            for p in self.centerline_smooth:
                self.centerline_vis[int(p[1]), int(p[0])] = (255, 0, 0)

            if self.visualize_results:
                Image.fromarray(self.centerline_vis).save(self.output_path / 'centerline.png')
        except Exception as e:
            print(f"Failed to generate centerline: {e}")
            traceback.print_exc()
            return False

        init_pose = np.zeros(3)
        init_pose[:2] = self.centerline_smooth[self.parameters["init_pose_ind"]]
        self.track_bounds_right, self.track_bounds_left = extract_track_bounds(self.centerline_smooth,
                                                    self.map,
                                                    map_editor_mode=False,
                                                    map_resolution=self.map_info["resolution"],
                                                    map_origin=(self.map_info["origin"][0], self.map_info["origin"][1]),
                                                    initial_position=init_pose,
                                                    show_plots=False)
        
        # convert centerline from cells to meters
        self.centerline_meter = np.zeros(np.shape(self.centerline_smooth))
        self.centerline_meter[:, 0] = self.centerline_smooth[:, 0] * self.map_info["resolution"] + self.map_info["origin"][0]
        self.centerline_meter[:, 1] = self.centerline_smooth[:, 1] * self.map_info["resolution"] + self.map_info["origin"][1]

        # interpolate centerline to 0.1m stepsize: less computation needed later for distance to track bounds
        self.centerline_meter = np.column_stack((self.centerline_meter, np.zeros((self.centerline_meter.shape[0], 2))))

        self.centerline_meter_int = interp_track(reftrack=self.centerline_meter, stepsize_approx=0.1)[:, :2]

        return True
    
    def compute_raceline(self):
        imp_opts = {"flip_imp_track": False,                # flip imported track to reverse direction
                "set_new_start": False,                 # set new starting point (changes order, not coordinates)
                "new_start": np.array([-2.4, 0.0]),    # [x_m, y_m]
                "min_track_width": None,                # [m] minimum enforced track width (set None to deactivate)
                "num_laps": 1} 
        reftrack_imp = self.import_track(imp_opts=imp_opts,
                                file_path=self.centerline_path+".csv",
                                width_veh=self.parameters["width"])

        reg_smooth_opts = {
            "k_reg": self.parameters["k_reg"],
            "s_reg": self.parameters["s_reg"]
        }
        stepsize_opts = {
            "stepsize_prep": self.parameters["stepsize_prep"],
            "stepsize_reg": self.parameters["stepsize_reg"],
            "stepsize_interp_after_opt": self.parameters["stepsize_interp_after_opt"]
        }
        reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = prep_track(reftrack_imp=reftrack_imp,
                                                                                                        reg_smooth_opts=reg_smooth_opts,
                                                                                                        stepsize_opts=stepsize_opts,
                                                                                                        debug=False,
                                                                                                        min_width=None)
        alpha_opt = np.zeros(reftrack_interp.shape[0])
        raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, t_vals_opt_interp, s_points_opt_interp,\
        spline_lengths_opt, el_lengths_opt_interp = create_raceline(refline=reftrack_interp[:, :2],
                                                                    normvectors=normvec_normalized_interp,
                                                                    alpha=alpha_opt,
                                                                    stepsize_interp=stepsize_opts["stepsize_interp_after_opt"])
        
        psi_vel_opt, kappa_opt = calc_head_curv_an(coeffs_x=coeffs_x_opt,
                                            coeffs_y=coeffs_y_opt,
                                            ind_spls=spline_inds_opt_interp,
                                            t_spls=t_vals_opt_interp)

        vx_profile_opt = calc_vel_profile(ggv=self.ggv,
                             ax_max_machines=self.ax_max_machines,
                             v_max=self.parameters["v_max"],
                             kappa=kappa_opt,
                             el_lengths=el_lengths_opt_interp,
                             closed=True,
                             filt_window=self.parameters["vel_profile_conv_filt_window"] if self.parameters["vel_profile_conv_filt_window"]>0 else None,
                             dyn_model_exp=self.parameters["dyn_model_exp"],
                             drag_coeff=self.parameters["dragcoeff"],
                             m_veh=self.parameters["mass"])
        
        # calculate longitudinal acceleration profile
        vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
        ax_profile_opt = calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                            el_lengths=el_lengths_opt_interp,
                                            eq_length_output=False)

        # calculate laptime
        self.estimated_lap_time = calc_t_profile(vx_profile=vx_profile_opt,
                                            ax_profile=ax_profile_opt,
                                            el_lengths=el_lengths_opt_interp)
        self.estimated_lap_time = self.estimated_lap_time[-1]
        # Post-processing
        trajectory_opt = np.column_stack((s_points_opt_interp,
                                    raceline_interp,
                                    psi_vel_opt,
                                    kappa_opt,
                                    vx_profile_opt,
                                    ax_profile_opt))
        spline_data_opt = np.column_stack((spline_lengths_opt, coeffs_x_opt, coeffs_y_opt))

        # create a closed race trajectory array
        self.global_trajectory = np.vstack((trajectory_opt, trajectory_opt[0, :]))
        self.global_trajectory[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

        #bound1, bound2 = check_traj(reftrack=reftrack_interp,
        #        reftrack_normvec_normalized=normvec_normalized_interp,
        #        length_veh=self.parameters["length"],
        #        width_veh=self.parameters["width"],
        #        debug=False,
        #        trajectory=trajectory_opt,
        #        ggv=self.ggv,
        #        ax_max_machines=self.ax_max_machines,
        #        v_max=self.parameters["v_max"],
        #        curvlim=self.parameters["curvlim"],
        #        mass_veh=self.parameters["mass"],
        #        dragcoeff=self.parameters["dragcoeff"])
        
        # [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2]

        self.traj_d_right, self.traj_d_left = dist_to_bounds(trajectory=self.global_trajectory,
                                                    bound_r=self.track_bounds_right,
                                                    bound_l=self.track_bounds_left,
                                                    centerline=self.centerline_meter_int,
                                                    safety_width=self.parameters['safety_width'],
                                                    use_normal_dist=True,
                                                    show_plots=False,
                                                    reverse=False)
        
        self.map_info_str = ''
        self.map_info_str += f'IQP estimated lap time: {np.round(self.estimated_lap_time, 4)}s; '
        self.map_info_str += f'IQP maximum speed: {np.max(self.global_trajectory[:, 5]):.4f}m/s; '
        print(self.map_info_str)
        self.global_trajectory[:,3] += np.pi
        file_out_path = self.output_path / f'{self.traj_name}.csv'
        write_global_trajectory_to_csv(self.global_trajectory, self.traj_d_left, self.traj_d_right, file_out_path)

        self.global_trajectory_pixel = self.global_trajectory[:, 1:3]
        self.global_trajectory_pixel = np.round((self.global_trajectory_pixel - np.array([[self.map_info["origin"][0], self.map_info["origin"][1]]])) / self.map_info["resolution"]).astype(np.int32)

        try:
            # Visualizations
            fig = plt.figure(figsize=(25, 20))
            ax = fig.add_subplot(2, 2, 1)
            ax.plot(self.centerline_meter[:, 0], self.centerline_meter[:, 1], color='red', label='Centerline')
            ax.plot(self.track_bounds_right[:, 0], self.track_bounds_right[:, 1], color='black', label='Track Boundaries')
            ax.plot(self.track_bounds_left[:, 0], self.track_bounds_left[:, 1], color='black')
            ax.plot(self.global_trajectory[:, 1], self.global_trajectory[:, 2], color='green', label='Raceline')
            pt1 = self.global_trajectory[0]
            pt2 = self.global_trajectory[6]
            arrow = FancyArrowPatch((pt1[1], pt1[2]), (pt2[1], pt2[2]), mutation_scale=15, color='black')
            ax.add_patch(arrow)
            ax.set_title("Raceline")

            ax = fig.add_subplot(2, 2, 2)
            ax.plot(self.global_trajectory[:, 0], self.global_trajectory[:, 3], color='blue')
            ax.axhline(0, color='red', linestyle='-', linewidth=1, label='y=0')
            ax.grid(True)
            ax.set_xlabel("s (m)")
            ax.set_ylabel("psi (rad)")
            ax.set_title("Steering Angle")

            ax = fig.add_subplot(2, 2, 3)
            ax.plot(self.global_trajectory[:, 0], self.global_trajectory[:, 4], color='blue')
            ax.axhline(0, color='red', linestyle='-', linewidth=1, label='y=0')
            ax.grid(True)
            ax.set_xlabel("s (m)")
            ax.set_ylabel("vx (m/s)")
            ax.set_title("Velocity")

            ax = fig.add_subplot(2, 2, 4)
            ax.plot(self.global_trajectory[:, 0], self.global_trajectory[:, 5], color='blue')
            ax.axhline(0, color='red', linestyle='-', linewidth=1, label='y=0')
            ax.grid(True)
            ax.set_xlabel("s (m)")
            ax.set_ylabel("ax (m/s^2)")
            ax.set_title("Velocity")

            if self.visualize_results:
                plt.savefig(self.output_path / 'global_trajectory.png', bbox_inches='tight', pad_inches=0.1)

            # print pyplot figure to image
            buf = io.BytesIO()
            plt.savefig(buf, bbox_inches='tight', pad_inches=0.1)
            buf.seek(0)
            pil_image = Image.open(buf)
            pil_image = pil_image.convert('RGB')
            self.glb_traj_vis = np.array(pil_image)
            buf.close()
        except Exception as e:
            print(f"Error visualizing the results: {e}")
            traceback.print_exc()

        return True

    
    def save_ros2_msgs(self):
        if not (importlib.util.find_spec("rclpy") is not None):
            print("rclpy module not found. ROS2 messages cannot be saved.")
            return False
        if self.map_info_str is None \
            or self.estimated_lap_time is None \
            or self.global_trajectory is None \
            or self.traj_d_left is None \
            or self.traj_d_right is None \
            or self.track_bounds_left is None \
            or self.track_bounds_right is None:
            print("Please run trajectory optimization completely before saving ROS2 messages.")
            return False
        
        mincurve_output = self.output_path / 'global_waypoint_msgs.json'
        try:
            write_waypoints(
                map_info=self.map_info_str,
                est_lap_time=self.estimated_lap_time,
                centerline=self.centerline_with_dist,
                global_trajectory=self.global_trajectory,
                d_right=self.traj_d_right,
                d_left=self.traj_d_left,
                bounds_left=self.track_bounds_left,
                bounds_right=self.track_bounds_right,
                output_file=mincurve_output
            )
            print(f"File saved to {mincurve_output}")
        except Exception as e:
            print(f"Error saving ROS2 messages: {str(e)}")
            traceback.print_exc()
            return False
        return True
    
    def import_track(self,
                 file_path: str,
                 imp_opts: dict,
                 width_veh: float) -> np.ndarray:
        """
        Created by:
        Alexander Heilmeier
        Modified by:
        Thomas Herrmann

        Documentation:
        This function includes the algorithm part connected to the import of the track.

        Inputs:
        file_path:      file path of track.csv containing [x_m,y_m,w_tr_right_m,w_tr_left_m]
        imp_opts:       import options showing if a new starting point should be set or if the direction should be reversed
        width_veh:      vehicle width required to check against track width

        Outputs:
        reftrack_imp:   imported track [x_m, y_m, w_tr_right_m, w_tr_left_m]
        """

        # load data from csv file
        csv_data_temp = np.loadtxt(file_path, comments='#', delimiter=',')

        # get coords and track widths out of array
        if np.shape(csv_data_temp)[1] == 3:
            refline_ = csv_data_temp[:, 0:2]
            w_tr_r = csv_data_temp[:, 2] / 2
            w_tr_l = w_tr_r

        elif np.shape(csv_data_temp)[1] == 4:
            refline_ = csv_data_temp[:, 0:2]
            w_tr_r = csv_data_temp[:, 2]
            w_tr_l = csv_data_temp[:, 3]

        elif np.shape(csv_data_temp)[1] == 5:  # omit z coordinate in this case
            refline_ = csv_data_temp[:, 0:2]
            w_tr_r = csv_data_temp[:, 3]
            w_tr_l = csv_data_temp[:, 4]

        else:
            raise IOError("Track file cannot be read!")

        refline_ = np.tile(refline_, (imp_opts["num_laps"], 1))
        w_tr_r = np.tile(w_tr_r, imp_opts["num_laps"])
        w_tr_l = np.tile(w_tr_l, imp_opts["num_laps"])

        # assemble to a single array
        reftrack_imp = np.column_stack((refline_, w_tr_r, w_tr_l))

        # check if imported centerline should be flipped, i.e. reverse direction
        if imp_opts["flip_imp_track"]:
            reftrack_imp = np.flipud(reftrack_imp)

        # check if imported centerline should be reordered for a new starting point
        if imp_opts["set_new_start"]:
            ind_start = np.argmin(np.power(reftrack_imp[:, 0] - imp_opts["new_start"][0], 2)
                                + np.power(reftrack_imp[:, 1] - imp_opts["new_start"][1], 2))
            reftrack_imp = np.roll(reftrack_imp, reftrack_imp.shape[0] - ind_start, axis=0)

        # check minimum track width for vehicle width plus a small safety margin
        w_tr_min = np.amin(reftrack_imp[:, 2] + reftrack_imp[:, 3])

        if w_tr_min < width_veh + 0.5:
            print("WARNING: Minimum track width %.2fm is close to or smaller than vehicle width!" % np.amin(w_tr_min))

        return reftrack_imp


    
