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

import numpy as np
from PIL import Image
from skimage.morphology import skeletonize

from global_planner.utils import ( 
    extract_centerline,
    smooth_centerline, 
    add_dist_to_cent,
    extract_track_bounds,
    dist_to_bounds,
    write_centerline,
    write_waypoints,
    write_raw_data,
    write_global_trajectory_to_csv
)
from global_planner.gco.helper_funcs_glob.interp_track import interp_track
from global_planner.gco.trajectory_optimizer import trajectory_optimizer
from global_planner.trajectory_planning_helpers.calc_head_curv_num import calc_head_curv_num
from global_planner.custom.racetrack_spline import racetrack_spliner



class GlobalTrajectoryOptimizer:

    def __init__(self,
                 map_path: str,
                 traj_name: Literal["min_curve","min_time"] = "min_curve",
                 visualize_results: bool = False,
                 ):
        
        self.map_path = map_path
        self.traj_name = traj_name
        self.visualize_results = visualize_results
        self.output_path = Path(self.map_path) / "plans"
        self.centerline_path = str(self.output_path / 'iqp_centerline')

        self.map = None
        self.map_info = None
        self.centerline_path = None

        self.parameters = {
            "centerline_length": 0.0,
            "safety_width": 0.6,
            "occ_grid_threshold": 10,
            "max_iters": 100,
            "curvature_limit": 10,
            "map_editor_mode": False,
            "init_pose_ind": 0,
            "reverse_mapping": False,
            "show_plots": False,
            "recompute_mintime": False,
            "recompute_velocity": False,
            "border_normal_dist": True,
            "config_path": str(Path(os.environ['LARACE_ROOT'])/"src"/"core"/"config"/"global_planner")
        }

        ##-- Intermediate Results --##
        self.skeleton = None

        self.centerline = None
        self.centerline_smooth = None
        self.centerline_with_dist = None
        self.centerline_meter = None
        self.centerline_meter_int = None

        self.track_bounds_left = None
        self.track_bounds_right = None

        self.map_info = None
        self.glbal_trajectory = None

        self.traj_d_right = None
        self.traj_d_left = None

        ##-- Visualizations --##
        self.skeleton_vis = None
        self.centerline_vis = None
        self.track_bounds_vis = None
        self.glb_traj_vis = None

        self.load_map(self.map_path)

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
            if self.visualize_results:
                pil_image = Image.fromarray(self.skeleton_vis)
                pil_image.save(self.output_path / 'skeleton.png')
        except Exception as e:
            print(f"Error saving skeleton image: {str(e)}")
            traceback.print_exc()
            return False
        
        return True
    
    def define_centerline(self):
        try:
            self.centerline = extract_centerline(
                    skeleton=self.skeleton,
                    cent_length=self.parameters["centerline_length"],
                    map_resolution=self.map_info["resolution"],
                    map_editor_mode=self.parameters["map_editor_mode"])
            self.centerline = self.centerline[::-1] if self.parameters["reverse_mapping"] else self.centerline
            print(f"# Center line points: {self.centerline.shape[0]}")
            force_filter_length = int(self.centerline.shape[0] // 10)
            force_filter_length = min(max(10, force_filter_length), 60)
            #force_filter_length = self.parameters["filter_size_"]
            self.centerline_smooth = np.copy(self.centerline)
            self.centerline_smooth = smooth_centerline(self.centerline_smooth, force_filter_length=force_filter_length)

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
        return True

    def extract_track_bounds(self):
        #import ipdb; ipdb.set_trace()
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



        trackbounds = extract_track_bounds(self.centerline_smooth,
                                                            self.map,
                                                            map_editor_mode=False,
                                                            map_resolution=self.map_info["resolution"],
                                                            map_origin=(self.map_info["origin"][0], self.map_info["origin"][1]),
                                                            initial_position=init_pose,
                                                            show_plots=self.parameters['show_plots'])
        
        if self.parameters["reverse_mapping"]:
            self.track_bounds_left, self.track_bounds_right = trackbounds
        else:
            self.track_bounds_right, self.track_bounds_left = trackbounds
        
        

        self.centerline_with_dist = add_dist_to_cent(centerline_smooth=self.centerline_smooth,
                                            centerline_meter=self.centerline_meter_int,
                                            map_resolution=self.map_info["resolution"],
                                            safety_width=self.parameters["safety_width"],
                                            show_plots=self.parameters['show_plots'],
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
        #import ipdb; ipdb.set_trace()
        output_path = self.output_path / 'interpolated'
        os.makedirs(output_path, exist_ok=True)
        raceline = self.global_trajectory[:,1:3]
        raceline_meter = np.round((raceline - np.array([[self.map_info["origin"][0], self.map_info["origin"][1]]])) / self.map_info["resolution"]).astype(np.int32)
        init_pose = np.zeros(3)
        init_pose[:2] = raceline_meter[self.parameters["init_pose_ind"]]

        _, _,track_bounds_left, track_bounds_right  = dist_to_bounds(trajectory=self.global_trajectory,
                                                    bound_r=self.track_bounds_right,
                                                    bound_l=self.track_bounds_left,
                                                    centerline=self.centerline_meter_int,
                                                    safety_width=self.parameters['safety_width'],
                                                    show_plots=self.parameters['show_plots'],
                                                    reverse=False,
                                                    stepsize_approx=0.05,
                                                    return_border_points=True
                                                    )
        racetrack_spliner(
            self.global_trajectory[:,:2],
            track_bounds_left,
            track_bounds_right,
            output_path=output_path,
            center_scaling = 2,
            right_scaling = 100,
            left_scaling = 100
        )

            
    def trajectory_optimization(self):
        #print(f"Reverse mapping: {self.parameters['reverse_mapping']}")
        centerline_coords = np.array([
            [coord[0], coord[1]] for coord in self.centerline_with_dist
        ])

        psi_centerline, kappa_centerline = calc_head_curv_num(
                path=centerline_coords,
                el_lengths=0.1 * np.ones(len(centerline_coords) - 1),
                is_closed=False
            )

        if self.traj_name == "min_curve":
            curv_opt_type = 'mincurv_iqp'
        elif self.traj_name == "min_time":
            curv_opt_type ='mintime'
        else:
            curv_opt_type = "mincurv_iqp"

        print('Start Global Trajectory optimization with iterative minimum curvature...')
        try:
            self.global_trajectory, bound_r_iqp, bound_l_iqp, self.estimated_lap_time = trajectory_optimizer(
                input_path=self.parameters["config_path"],
                track_name=self.centerline_path,
                curv_opt_type=curv_opt_type,
                safety_width=self.parameters["safety_width"],
                plot=self.parameters["show_plots"],
                recompute_velocity_profile=self.parameters["recompute_velocity"],
                reopt_mintime=self.parameters["recompute_mintime"],
                iter_limit = self.parameters["max_iters"],
                curvature_limit= self.parameters["curvature_limit"] 
                )
        except RuntimeError as e:
            print(f"Error during iterative optimization, error: {e}")
            traceback.print_exc()
            return False
        
        self.bound_right_raceline = bound_r_iqp
        self.bound_left_raceline = bound_l_iqp

        # [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2]

        self.traj_d_right, self.traj_d_left = dist_to_bounds(trajectory=self.global_trajectory,
                                                    bound_r=self.track_bounds_right,
                                                    bound_l=self.track_bounds_left,
                                                    centerline=self.centerline_meter_int,
                                                    safety_width=self.parameters['safety_width'],
                                                    show_plots=self.parameters['show_plots'],
                                                    use_normal_dist=self.parameters['border_normal_dist'],
                                                    reverse=False)
        self.map_info_str = ''
        self.map_info_str += f'IQP estimated lap time: {round(self.estimated_lap_time, 4)}s; '
        self.map_info_str += f'IQP maximum speed: {round(np.amax(self.global_trajectory[:, 5]), 4)}m/s; '
        print(self.map_info_str)
        file_out_path = self.output_path / f'{self.traj_name}.csv'
        self.global_trajectory[:,3] += np.pi
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