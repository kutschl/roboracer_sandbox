import os

import numpy as np
from pathlib import Path
from tqdm import tqdm

from scipy.interpolate import splprep, splev,interp1d

def interpolate_path(pts,smooth_value,scale=2,derivative_order=0):
    u_test = np.arange(len(pts))
    tck, u = splprep(pts.T, u=u_test, s=smooth_value, per=1)
    u_new = np.linspace(u.min(), u.max(), len(pts)*scale)
    x_new, y_new = splev(u_new, tck, der=derivative_order)
    interp_points = np.concatenate((x_new.reshape((-1, 1)), y_new.reshape((-1, 1))), axis=1)
    return interp_points


def calculate_delta_angle(a1,a2):
    """ Calculate"""
    a1= np.where(a1<0,a1+2*np.pi,a1)
    if a2<0:
        a2 = a2 + 2*np.pi
    delta = abs(a1 - a2)
    alternate_delta = 2*np.pi - delta
    min_delta = np.minimum(delta,alternate_delta)
    return min_delta


def calculate_arc_lengths(waypoints):
    d = np.diff(waypoints,axis=0)
    consecutive_diff= np.sqrt(np.sum(np.power(d, 2), axis=1))
    dists_cum = np.cumsum(consecutive_diff)
    dists_cum = np.insert(dists_cum, 0, 0.0)
    return dists_cum


def get_wall_point( 
                   wall_points, 
                   center_point, 
                   center_points_angles, 
                   idx,
                   sort_dist_window_percentage: float = 3.0,
                   ):
    SORT_DISTANCES_WINDOW = int(sort_dist_window_percentage*len(wall_points)/100.0)
    SORT_ANGLES_WINDOW =    int(sort_dist_window_percentage*len(wall_points)/100.0)
    distances_array = np.linalg.norm(wall_points - center_point, axis=1)
    #sort points based on distance to the center line point and pick points equal to SORT_DISTANCES_WINDOW
    sorted_distances_idx = np.argsort(distances_array)[0:SORT_DISTANCES_WINDOW]
    point_delta = wall_points[sorted_distances_idx] - center_point
    point_angle = np.arctan2(point_delta[:, 1], point_delta[:, 0])
    #finding difference in angle between the tangent at centerline and line joining wall point and centerline point
    angle_delta = calculate_delta_angle(point_angle, center_points_angles[idx])
    # sorting based on proximity of delta angle to 90 degrees which is perpendicular condition
    sorted_delta_angle_idx = np.argsort(np.abs(angle_delta - np.pi / 2))[0:SORT_ANGLES_WINDOW]
    angle_idx=0
    desired_idx = sorted_distances_idx[sorted_delta_angle_idx[angle_idx]]
    last_wall_index=desired_idx

    return wall_points[last_wall_index,:],point_angle[sorted_delta_angle_idx[angle_idx]],last_wall_index

def racetrack_spliner(center_line: np.ndarray, left_border: np.ndarray, right_border: np.ndarray,
                      center_smooth_value: float = 0.75,
                      right_smooth_value: float = 0.01,
                      left_smooth_value: float = 0.01,
                      center_scaling: float = 20,
                      left_scaling: float = 500,
                      right_scaling: float = 500,
                      reverse: bool = False,
                      max_speed: float = 5.0,
                      min_speed: float = 0.5, 
                      output_path: str = None,
                      initial_downsample: int = None,
                      loginfo = print,
                      ):
    loginfo("Computing racetrack spline interpolation!")
    #Read waypoints from respective files
    if reverse:
        center_line = center_line[::-1]
    
    if initial_downsample is not None:
        center_line = center_line[::initial_downsample]
        left_border = left_border[::initial_downsample]
        right_border = right_border[::initial_downsample]

    loginfo("Interpolating center line and borders...")
    # Interpolate center line upto desired resolution
    center_interp_path = interpolate_path(center_line,smooth_value=center_smooth_value, scale=center_scaling, derivative_order=0)
    center_derivative =  interpolate_path(center_line,smooth_value=center_smooth_value, scale=center_scaling, derivative_order=1)
    second_derivative =  interpolate_path(center_line,smooth_value=center_smooth_value, scale=center_scaling, derivative_order=2)
    curvature = abs(center_derivative[:,0]* second_derivative[:,1] - center_derivative[:,1]* second_derivative[:,0]) / np.power(center_derivative[:,0]** 2 + center_derivative[:,1]** 2, 1.5)
    
    center_points_angles= np.arctan2(center_derivative[:,1], center_derivative[:,0])

    loginfo("Reinterpolate borders and courser scale...")
    #Interpolate outer and inner wall line upto 2X scale compared to center line
    right_interp_path = interpolate_path(right_border,smooth_value=right_smooth_value,scale=right_scaling,derivative_order=0)
    left_interp_path =  interpolate_path(left_border, smooth_value=left_smooth_value, scale=left_scaling, derivative_order=0)

    final_right_path = np.zeros_like(center_interp_path)
    final_left_path = np.zeros_like(center_interp_path)
    final_right_path_width = np.zeros_like(center_points_angles)
    final_left_path_width = np.zeros_like(center_points_angles)

    #Perpendicular angle satisfying criteria for wall points
    final_right_point_angle=np.zeros_like(center_points_angles)
    final_left_point_angle = np.zeros_like(center_points_angles)
    last_left_index = 0
    last_right_index=0

    loginfo("Compute final path information...")
    #import ipdb; ipdb.set_trace()
    for idx,center_point in tqdm(enumerate(center_interp_path), total=center_interp_path.shape[0]):
        final_right_path[idx,:],final_right_point_angle[idx], last_right_index = get_wall_point(right_interp_path,center_point, center_points_angles, idx)
        final_left_path[idx, :], final_left_point_angle[idx], last_left_index =  get_wall_point(left_interp_path, center_point, center_points_angles, idx)
        final_right_path_width[idx]=np.linalg.norm(final_right_path[idx,:]-center_interp_path[idx,:])
        final_left_path_width[idx] = np.linalg.norm(final_left_path[idx, :] - center_interp_path[idx, :])
    trackwidths = np.column_stack((final_right_path_width,final_left_path_width))

    idx = 0
    element_arc_lengths = calculate_arc_lengths(center_interp_path)
    #plt.plot(element_arc_lengths,curvature)

    vel_profile=np.interp(curvature, (curvature.min(), curvature.max()), (max_speed, min_speed))
    # vel_profile=interp1d(curvature,())
    #plt.plot(element_arc_lengths,vel_profile)
    #plt.show()
    vel_profile = np.column_stack((element_arc_lengths,vel_profile))
    

    #curvature_norm_obj = plt.Normalize(curvature.min(),curvature.max())
    #n_c = curvature_norm_obj(curvature)
    #plt.scatter(center_interp_path[:,0], center_interp_path[:,1], c=(1-n_c), cmap='RdYlGn',s = 3)
    #plt.show()
    # input('str')
    if output_path is not None:
        output_path = Path(output_path) 
        os.makedirs(output_path, exist_ok=True)
        loginfo(f"Saving results to {str(output_path)}")
        np.savetxt(str(output_path / 'centerline_waypoints.csv'), center_interp_path, delimiter=",")
        np.savetxt(str(output_path / 'right_waypoints.csv'), final_right_path, delimiter=",")
        np.savetxt(str(output_path / 'left_waypoints.csv'), final_left_path, delimiter=",")
        np.savetxt(str(output_path / 'track_widths.csv'), trackwidths, delimiter=",")
        np.savetxt(str(output_path / 'center_spline_derivatives.csv'), center_derivative, delimiter=",")
        np.savetxt(str(output_path / 'velocity_profile.csv'), vel_profile, delimiter=",")

    else:
        return center_interp_path, final_right_path, final_left_path, trackwidths, center_derivative, vel_profile
