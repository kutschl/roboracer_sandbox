import numpy as np
import cv2 as cv
import csv
import os
import matplotlib.pyplot as plt
from pathlib import Path

from trajectory_planning_helpers.calc_normal_vectors import calc_normal_vectors
from global_planner.gco.helper_funcs_glob.interp_track import interp_track
import json

from scipy.signal import savgol_filter
from skimage.segmentation import watershed

from f110_msgs.msg import WpntArray, Wpnt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point 

def extract_centerline(skeleton, cent_length: float, map_resolution: float, map_editor_mode: bool, min_centerline_pts: int= 100) -> np.ndarray:
    """
    Extract the centerline out of the skeletonized binary image.

    This function takes a skeletonized binary image as input and extracts the centerline from it. The centerline is
    determined by finding closed contours in the image and comparing their lengths to the expected approximate centerline
    length. The function returns the centerline in the form of a numpy array, where each element represents a point on
    the centerline in cells (not meters).

    Parameters
    ----------
    skeleton : np.ndarray
        The skeleton of the binarized and filtered map.
    cent_length : float
        The expected approximate centerline length.
    map_resolution : float
        The resolution of the map in meters per cell.
    map_editor_mode : bool
        A flag indicating whether the function is being called in map editor mode.

    Returns
    -------
    centerline : np.ndarray
        The centerline in the form [[x1, y1], [x2, y2], ...], where each element represents a point on the centerline
        in cells (not meters).

    Raises
    ------
    IOError
        If no closed contour is found in the skeletonized image.
    ValueError
        If all found contours do not have a similar line length as the centerline (can only happen if `map_editor_mode`
        is True).

    """
    # get contours from skeleton
    contours, hierarchy = cv.findContours(skeleton, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)

    # save all closed contours
    closed_contours = []
    for i, elem in enumerate(contours):
        opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
        if not opened:
            closed_contours.append(elem)
    
    valid_contours = []
    for cc in closed_contours:
        if cc.shape[0]>min_centerline_pts:
            valid_contours.append(cc)
    closed_contours = valid_contours
    

    # if we have no closed contour, we can't calculate a global trajectory
    if len(closed_contours) == 0:
        raise IOError("No closed contours")

    # calculate the line length of every contour to get the real centerline
    line_lengths = [np.inf] * len(closed_contours)
    for i, cont in enumerate(closed_contours):
        line_length = 0
        for k, pnt in enumerate(cont):
            line_length += np.sqrt((pnt[0][0] - cont[k - 1][0][0]) ** 2 +
                                   (pnt[0][1] - cont[k - 1][0][1]) ** 2)

        line_length *= map_resolution  # length in meters not cells
        # line length should be around the length of the centerline otherwise keep length infinity
        if np.abs(cent_length / line_length - 1.0) < 0.15:
            line_lengths[i] = line_length
        elif map_editor_mode or cent_length == 0.0:
            line_lengths[i] = line_length


    # take the shortest line
    min_line_length = min(line_lengths)

    if min_line_length == np.inf:
        raise ValueError("Only invalid closed contour line lengths")

    min_length_index = line_lengths.index(min_line_length)
    # print(line_lengths)
    smallest_contour = np.array(closed_contours[min_length_index]).flatten()

    # reshape smallest_contours from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
    # this will be the centerline
    len_reshape = int(len(smallest_contour) / 2)
    centerline = smallest_contour.reshape(len_reshape, 2)

    return centerline

def smooth_centerline(centerline: np.ndarray, force_filter_length: int = None) -> np.ndarray:
    """
    Smooth the centerline with a Savitzky-Golay filter.

    Notes
    -----
    The savgol filter doesn't ensure a smooth transition at the end and beginning of the centerline. That's why
    we apply a savgol filter to the centerline with start and end points on the other half of the track.
    Afterwards, we take the results of the second smoothed centerline for the beginning and end of the
    first centerline to get an overall smooth centerline

    Parameters
    ----------
    centerline : np.ndarray
        Unsmoothed centerline

    Returns
    -------
    centerline_smooth : np.ndarray
        Smooth centerline
    """
    # centerline_smooth = centerline
    # smooth centerline with a Savitzky Golay filter
    # filter_length = 9
    centerline_length = len(centerline)
    # print("Number of centerline points: ", centerline_length)

    # if centerline_length > 2000:
    #     filter_length = 21 # int(centerline_length / 200) * 10 + 1
    # elif centerline_length > 1000:
    #     filter_length = 21
    # elif centerline_length > 500:
    #     filter_length = 21
    # else:
    #     filter_length = 21
    if force_filter_length is not None:
        filter_length = force_filter_length
    centerline_smooth = savgol_filter(centerline, filter_length, 3, axis=0)

    # cen_len is half the length of the centerline
    cen_len = int(len(centerline) / 2)
    centerline2 = np.append(centerline[cen_len:], centerline[0:cen_len], axis=0)
    centerline_smooth2 = savgol_filter(centerline2, filter_length, 3, axis=0)

    # take points from second (smoothed) centerline for first centerline
    centerline_smooth[0:filter_length] = centerline_smooth2[cen_len:(cen_len + filter_length)]
    centerline_smooth[-filter_length:] = centerline_smooth2[(cen_len - filter_length):cen_len]

    return centerline_smooth

def add_dist_to_cent(centerline_smooth: np.ndarray,
                     centerline_meter: np.ndarray,
                     map_resolution: float,
                     safety_width: float,
                     show_plots: bool,
                     dist_transform=None,
                     bound_r: np.ndarray = None,
                     bound_l: np.ndarray = None,
                     reverse: bool = False) -> np.ndarray:
    """
    Add distance to track bounds to the centerline points.

    Parameters
    ----------
    centerline_smooth : np.ndarray
        Smooth centerline in cells (not meters)
    centerline_meter : np.ndarray
        Smooth centerline in meters (not cells)
    map_resolution : float
        Resolution of the map in meters per cell
    safety_width : float
        Safety width for the track bounds
    show_plots : bool
        Flag indicating whether to show plots or not
    dist_transform : Any, default=None
        Euclidean distance transform of the filtered black and white image
    bound_r : np.ndarray, default=None
        Points of the right track bound in meters
    bound_l : np.ndarray, default=None
        Points of the left track bound in meters
    reverse : bool, default=False
        Flag indicating whether the trajectory is being calculated in reverse direction

    Returns
    -------
    centerline_comp : np.ndarray
        Complete centerline with distance to right and left track bounds for every point
    """
    centerline_comp = np.zeros((len(centerline_meter), 4))

    if dist_transform is not None:
        width_track_right = dist_transform[centerline_smooth[:, 1].astype(int),
                                           centerline_smooth[:, 0].astype(int)] * map_resolution
        if len(width_track_right) != len(centerline_meter):
            width_track_right = np.interp(np.arange(0, len(centerline_meter)), np.arange(0, len(width_track_right)),
                                          width_track_right)
        width_track_left = width_track_right
    elif bound_r is not None and bound_l is not None:
        width_track_right, width_track_left = dist_to_bounds(centerline_meter, bound_r, bound_l,
                                                             centerline=centerline_meter,
                                                             safety_width=safety_width,
                                                             show_plots=show_plots,
                                                             reverse=reverse)
    else:
        raise IOError("No closed contours found...")

    centerline_comp[:, 0] = centerline_meter[:, 0]
    centerline_comp[:, 1] = centerline_meter[:, 1]
    centerline_comp[:, 2] = width_track_right
    centerline_comp[:, 3] = width_track_left
    return centerline_comp

def dist_to_bounds(
        trajectory: np.ndarray,
        bound_r,
        bound_l,
        centerline: np.ndarray,
        safety_width: float,
        show_plots: bool,
        reverse: bool = False,
        stepsize_approx: float = 0.1,
        use_normal_dist: bool = False,
        return_border_points: bool = False
        ) -> tuple[np.ndarray, np.ndarray]:
    """
    Calculate the distance to track bounds for every point on a trajectory.

    Parameters
    ----------
    trajectory : np.ndarray
        A trajectory in form [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2] or [x_m, y_m]
    bound_r
        Points in meters of boundary right
    bound_l
        Points in meters of boundary left
    centerline : np.ndarray
        Centerline only needed if global trajectory is given and plot of it is wanted
    safety_width : float
        Safety width in meters
    show_plots : bool
        Flag indicating whether to show plots or not
    reverse : bool, optional
        Flag indicating whether the trajectory is in reverse direction or not, by default False

    Returns
    -------
    dists_right, dists_left : tuple[np.ndarray, np.ndarray]
        Distances to the right and left track boundaries for every waypoint
    """
    # check format of trajectory
    if len(trajectory[0]) > 2:
        help_trajectory = trajectory[:, 1:3]
        psi = trajectory[:, 3] + np.pi
        trajectory_str = "Global Trajectory"
    else:
        help_trajectory = trajectory
        trajectory_str = "Centerline"

    # interpolate track bounds
    bound_r_tmp = np.column_stack((bound_r, np.zeros((bound_r.shape[0], 2))))
    bound_l_tmp = np.column_stack((bound_l, np.zeros((bound_l.shape[0], 2))))

    bound_r_int = interp_track(reftrack=bound_r_tmp, stepsize_approx=stepsize_approx)
    bound_l_int = interp_track(reftrack=bound_l_tmp, stepsize_approx=stepsize_approx)

    # find the closest points of the track bounds to global trajectory waypoints
    n_wpnt = len(help_trajectory)
    dists_right = np.zeros(n_wpnt)  # contains (min) distances between waypoints and right bound
    dists_left = np.zeros(n_wpnt)  # contains (min) distances between waypoints and left bound

    left_border_points = np.zeros((n_wpnt, 2))
    right_border_points = np.zeros((n_wpnt, 2))

    #import ipdb; ipdb.set_trace()

    for i, wpnt in enumerate(help_trajectory):

        if not use_normal_dist:
            dists_bound_right = np.sqrt(np.power(bound_r_int[:, 0] - wpnt[0], 2)+ np.power(bound_r_int[:, 1] - wpnt[1], 2))
            dists_bound_left = np.sqrt(np.power(bound_l_int[:, 0] - wpnt[0], 2) + np.power(bound_l_int[:, 1] - wpnt[1], 2))

            dists_right[i] = np.amin(dists_bound_right)

            min_index_right = np.argmin(dists_bound_right)
            right_border_points[i] = bound_r_int[min_index_right,:2]

            dists_left[i] = np.amin(dists_bound_left)

            min_index_left = np.argmin(dists_bound_left)
            left_border_points[i] = bound_l_int[min_index_left,:2]
        else:
            # 1) compute Euclidean distances to all boundary points
            euclid_r = np.linalg.norm(bound_r_int[:, :2] - wpnt, axis=1)
            euclid_l = np.linalg.norm(bound_l_int[:, :2] - wpnt, axis=1)

            # 2) select up to the 10 closest by Euclid
            k = 10
            inds_r = np.argsort(euclid_r)[:k]
            inds_l = np.argsort(euclid_l)[:k]

            # 3) get your normal directions
            right_normal = np.array([np.cos(psi[i]), np.sin(psi[i])])
            left_normal  = -right_normal

            # --- RIGHT BOUNDARY: restrict to k nn ---
            cand_r      = bound_r_int[inds_r, :2]
            diff_r      = cand_r - wpnt
            t_r         = diff_r.dot(right_normal)
            Q_r         = wpnt + np.outer(t_r, right_normal)
            d_perp_r    = np.linalg.norm(cand_r - Q_r, axis=1)
            local_r     = np.argmin(d_perp_r)      # best among the k by perpendicular distance
            best_r      = inds_r[local_r]          # map back to full array index

            # store Euclidean distance for that best point
            dists_right[i]         = euclid_r[best_r]
            right_border_points[i] = bound_r_int[best_r, :2]

            # --- LEFT BOUNDARY: same procedure ---
            cand_l      = bound_l_int[inds_l, :2]
            diff_l      = cand_l - wpnt
            t_l         = diff_l.dot(left_normal)
            Q_l         = wpnt + np.outer(t_l, left_normal)
            d_perp_l    = np.linalg.norm(cand_l - Q_l, axis=1)
            local_l     = np.argmin(d_perp_l)
            best_l      = inds_l[local_l]

            # store Euclidean distance for that best point
            dists_left[i]          = euclid_l[best_l]
            left_border_points[i]  = bound_l_int[best_l, :2]
    

    if show_plots and trajectory_str == "Global Trajectory":
        width_veh_real = 0.3
        normvec_normalized_opt = calc_normal_vectors(trajectory[:, 3])

        veh_bound1_virt = trajectory[:, 1:3] + normvec_normalized_opt * safety_width / 2
        veh_bound2_virt = trajectory[:, 1:3] - normvec_normalized_opt * safety_width / 2

        veh_bound1_real = trajectory[:, 1:3] + normvec_normalized_opt * width_veh_real / 2
        veh_bound2_real = trajectory[:, 1:3] - normvec_normalized_opt * width_veh_real / 2

        # plot track including optimized path
        fig, ax = plt.subplots()
        fig.suptitle("Track with optimized path")

        ax.plot(bound_r[:, 0], bound_r[:, 1], "k-", linewidth=1.0, label="Track bounds")
        ax.plot(bound_l[:, 0], bound_l[:, 1], "k-", linewidth=1.0)
        ax.plot(centerline[:, 0], centerline[:, 1], "k--", linewidth=1.0, label='Centerline')

        ax.plot(veh_bound1_virt[:, 0], veh_bound1_virt[:, 1], "b", linewidth=0.5, label="Vehicle width with safety")
        ax.plot(veh_bound2_virt[:, 0], veh_bound2_virt[:, 1], "b", linewidth=0.5)
        ax.plot(veh_bound1_real[:, 0], veh_bound1_real[:, 1], "c", linewidth=0.5, label="Real vehicle width")
        ax.plot(veh_bound2_real[:, 0], veh_bound2_real[:, 1], "c", linewidth=0.5)

        ax.plot(trajectory[:, 1], trajectory[:, 2], 'tab:orange', linewidth=2.0, label="Global trajectory")

        plt.grid()
        ax1 = plt.gca()

        point1_arrow = np.array([trajectory[0, 1], trajectory[0, 2]])
        point2_arrow = np.array([trajectory[5, 1], trajectory[5, 2]])
        vec_arrow = (point2_arrow - point1_arrow)
        ax1.arrow(point1_arrow[0], point1_arrow[1], vec_arrow[0], vec_arrow[1], width=0.05,
                  head_width=0.3, head_length=0.3, fc='g', ec='g')

        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from origin [m]")
        plt.ylabel("y-distance from origin [m]")
        plt.title(f"Global trajectory with vehicle width")
        plt.legend()

        plt.show()
    # Return flipped distances if map_editor reversing
    if reverse:
        if return_border_points:
            return dists_left, dists_right, left_border_points, right_border_points
        return dists_left, dists_right
    else:
        if return_border_points:
            return dists_right, dists_left, right_border_points, left_border_points
        return dists_right, dists_left


def extract_track_bounds(
        centerline: np.ndarray,
        filtered_bw: np.ndarray,
        map_editor_mode: bool,
        map_resolution: float,
        map_origin: tuple,
        initial_position: np.ndarray,
        show_plots: bool) -> tuple[np.ndarray, np.ndarray]:
    """
    Extract the boundaries of the track.

    Use the watershed algorithm with the centerline as marker to extract the boundaries of the filtered black
    and white image of the map.

    Parameters
    ----------
    centerline : np.ndarray
        The centerline of the track (in cells not meters)
    filtered_bw : np.ndarray
        Filtered black and white image of the track
    map_editor_mode : bool
        Flag indicating whether the map editor mode is enabled
    map_resolution : float
        Resolution of the map (in meters per cell)
    map_origin : Point
        Origin point of the map
    initial_position : np.ndarray
        Initial position of the vehicle
    show_plots : bool
        Flag indicating whether to show plots

    Returns
    -------
    bound_right, bound_left : tuple[np.ndarray, np.ndarray]
        Points of the track bounds right and left in meters

    Raises
    ------
    IOError
        If there were more (or less) than two track bounds found
    """
    # create a black and white image of the centerline
    cent_img = np.zeros((filtered_bw.shape[0], filtered_bw.shape[1]), dtype=np.uint8)
    cv.drawContours(cent_img, [centerline.astype(int)], 0, 255, 2, cv.LINE_8)

    # create markers for watershed algorithm
    _, cent_markers = cv.connectedComponents(cent_img)

    # apply watershed algorithm to get only the track (without any lidar beams outside the track)
    dist_transform = cv.distanceTransform(filtered_bw, cv.DIST_L2, 5)
    labels = watershed(-dist_transform, cent_markers, mask=filtered_bw)

    closed_contours = []

    for label in np.unique(labels):
        if label == 0:
            continue

        # Create a mask, the mask should be the track
        mask = np.zeros(filtered_bw.shape, dtype="uint8")
        mask[labels == label] = 255

        if show_plots and not map_editor_mode:
            plt.imshow(mask, cmap='gray')
            plt.show()

        # Find contours
        contours, hierarchy = cv.findContours(mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)

        # save all closed contours
        for i, cont in enumerate(contours):
            opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
            if not opened:
                closed_contours.append(cont)

        # there must not be more (or less) than two closed contour
        if len(closed_contours) != 2:
            raise IOError("More than two track bounds detected! Check input")
        # draw the boundary into the centerline image
        cv.drawContours(cent_img, closed_contours, 0, 255, 4)
        cv.drawContours(cent_img, closed_contours, 1, 255, 4)

    # the longest closed contour is the outer boundary
    bound_long = max(closed_contours, key=len)
    bound_long = np.array(bound_long).flatten()

    # reshape from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
    len_reshape = int(len(bound_long) / 2)
    bound_long = bound_long.reshape(len_reshape, 2)
    # convert to meter
    bound_long_meter = np.zeros(np.shape(bound_long))
    bound_long_meter[:, 0] = bound_long[:, 0] * map_resolution + map_origin[0]
    bound_long_meter[:, 1] = bound_long[:, 1] * map_resolution + map_origin[1]

    # inner boundary is the shorter one
    bound_short = min(closed_contours, key=len)
    bound_short = np.array(bound_short).flatten()

    # reshape from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
    len_reshape = int(len(bound_short) / 2)
    bound_short = bound_short.reshape(len_reshape, 2)
    # convert to meter
    bound_short_meter = np.zeros(np.shape(bound_short))
    bound_short_meter[:, 0] = bound_short[:, 0] * map_resolution + map_origin[0]
    bound_short_meter[:, 1] = bound_short[:, 1] * map_resolution + map_origin[1]

    # get distance to initial position for every point on the outer bound to figure out if it is the right
    # or left boundary
    bound_distance = np.sqrt(np.power(bound_long_meter[:, 0] - initial_position[0], 2)
                             + np.power(bound_long_meter[:, 1] - initial_position[1], 2))

    min_dist_ind = np.argmin(bound_distance)

    bound_direction = np.angle([complex(bound_long_meter[min_dist_ind, 0] - bound_long_meter[min_dist_ind - 1, 0],
                                        bound_long_meter[min_dist_ind, 1] - bound_long_meter[min_dist_ind - 1, 1])])

    norm_angle_right = initial_position[2] - np.pi
    if norm_angle_right < -np.pi:
        norm_angle_right = norm_angle_right + 2 * np.pi

    if compare_direction(norm_angle_right, bound_direction):
        bound_right = bound_long_meter
        bound_left = bound_short_meter
    else:
        bound_right = bound_short_meter
        bound_left = bound_long_meter

    if show_plots and not map_editor_mode:
        plt.imshow(cent_img, cmap='gray')
        fig1, ax1 = plt.subplots()
        ax1.plot(bound_right[:, 0], bound_right[:, 1], 'b', label='Right bound')
        ax1.plot(bound_left[:, 0], bound_left[:, 1], 'g', label='Left bound')
        ax1.plot(centerline[:, 0] * map_resolution + map_origin[0],
                 centerline[:, 1] * map_resolution + map_origin[1], 'r', label='Centerline')
        ax1.legend()
        plt.show()

    return bound_right, bound_left

def compare_direction(alpha: float, beta: float) -> bool:
    """
    Compare the direction of two points and check if they point in the same direction.

    Parameters
    ----------
    alpha : float
        direction angle in rad
    beta : float
        direction angle in rad

    Returns
    -------
    bool
        True if alpha and beta point in the same direction
    """
    delta_theta = np.abs(alpha - beta)

    if delta_theta > np.pi:
        delta_theta = 2 * np.pi - delta_theta

    return delta_theta < np.pi / 2


def write_centerline(centerline: np.ndarray, output_path: str) -> None:
    has_suffix = Path(output_path).suffix != ''
    if not has_suffix:
        output_path += '.csv'

    try:
        with open(output_path, 'w', newline='') as file:
            writer = csv.writer(file)
            for row in centerline:
                x_m = row[0]
                y_m = row[1]
                width_tr_right_m = row[2]
                width_tr_left_m = row[3]
                writer.writerow([x_m, y_m, width_tr_right_m, width_tr_left_m])
    except Exception as e:
        print(f"Error writing centerline to file '{output_path}': \n{e}")
        return False
    return True

def conv_psi(psi: float) -> float:
    """
    Convert psi angles where 0 is at the y-axis into psi angles where 0 is at the x-axis.

    Parameters
    ----------
    psi : float
        angle (in rad) to convert

    Returns
    -------
    new_psi : float
        converted angle
    """
    new_psi = psi + np.pi / 2

    if new_psi > np.pi:
        new_psi = new_psi - 2 * np.pi

    return new_psi

def write_raw_data(
        centerline: np.ndarray,
        global_trajectory: np.ndarray,
        d_right: np.ndarray,
        d_left: np.ndarray,
        output_path: str
):
    output_path = Path(output_path)
    np.savetxt(str(output_path / 'centerline_waypoints.csv'), centerline, delimiter=",")
    np.savetxt(str(output_path / 'global_trajectory.csv'), global_trajectory, delimiter=",")
    np.savetxt(str(output_path / 'right_waypoints.csv'), d_right, delimiter=",")
    np.savetxt(str(output_path / 'left_waypoints.csv'), d_left, delimiter=",")



def write_waypoints(
        map_info: str,
        est_lap_time: float,
        centerline: np.ndarray,
        global_trajectory: np.ndarray,
        d_right: np.ndarray,
        d_left: np.ndarray,
        bounds_right: np.ndarray,
        bounds_left: np.ndarray,
        output_file: str
):
    from geometry_msgs.msg import Point
    from f110_msgs.msg import Wpnt, WpntArray
    from visualization_msgs.msg import Marker, MarkerArray
    from rosidl_runtime_py.convert import message_to_ordereddict
    
    def get_centerline_markers(centerline: np.ndarray) -> MarkerArray:
        """
        Create a csv file with centerline maps.

        The centerline maps includes position and width to the track bounds in meter, so that it can be used in the
        global trajectory optimizer. The file has the following format: x_m, y_m, w_tr_right_m, w_tr_left_m .

        Parameters
        ----------
        centerline : np.ndarray
            The centerline in form [x_m, y_m, w_tr_right_m, w_tr_left_m]
        sp_bool : bool, default=False
            Used for shortest path optimization when another centerline csv should be created

        Returns
        -------
        centerline_markers : MarkerArray
            Centerline as a MarkerArray which can be published
        """
        centerline_markers = MarkerArray()
        centerline_wpnts = WpntArray()

        id_cnt = 0  # for marker id

        for row in centerline:
            x_m = row[0]
            y_m = row[1]
            width_tr_right_m = row[2]
            width_tr_left_m = row[3]

            cent_marker = Marker()
            cent_marker.header.frame_id = 'map'
            cent_marker.type = cent_marker.SPHERE
            cent_marker.scale.x = 0.05
            cent_marker.scale.y = 0.05
            cent_marker.scale.z = 0.05
            cent_marker.color.a = 1.0
            cent_marker.color.b = 1.0

            cent_marker.id = id_cnt
            cent_marker.pose.position.x = x_m
            cent_marker.pose.position.y = y_m
            cent_marker.pose.orientation.w = 1.0
            centerline_markers.markers.append(cent_marker)

            wpnt = Wpnt()
            wpnt.id = id_cnt
            wpnt.x_m = x_m
            wpnt.d_right = width_tr_right_m
            wpnt.d_left = width_tr_left_m
            wpnt.y_m = y_m
            centerline_wpnts.wpnts.append(wpnt)

            id_cnt += 1

        return centerline_wpnts, centerline_markers
    
    def create_wpnts_markers(trajectory: np.ndarray, d_right: np.ndarray, d_left: np.ndarray,
                         second_traj: bool = False) -> tuple[WpntArray, MarkerArray]:
        """
        Create and return a waypoint array and a marker array.

        Parameters
        ----------
        trajectory : np.ndarray
            A trajectory with waypoints in form [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2]
        d_right : np.ndarray
            Distances to the right track bounds for every waypoint in {trajectory}
        d_left : np.ndarray
            Distances to the left track bounds for every waypoint in {trajectory}
        second_traj : bool, default=False
            Display second trajectory with a different color than first, better for visualization

        Returns
        -------
        global_wpnts, global_markers : tuple[WpntArray, MarkerArray]
            A waypoint array and a marker array with all points of {trajectory}
        """
        max_vx_mps = max(trajectory[:, 5])

        global_wpnts = WpntArray()
        global_markers = MarkerArray()
        for i, pnt in enumerate(trajectory):
            global_wpnt = Wpnt()
            global_wpnt.id = i
            global_wpnt.s_m = pnt[0]
            global_wpnt.x_m = pnt[1]
            global_wpnt.d_right = d_right[i]
            global_wpnt.d_left = d_left[i]
            global_wpnt.y_m = pnt[2]
            global_wpnt.psi_rad = conv_psi(pnt[3])
            global_wpnt.kappa_radpm = pnt[4]
            global_wpnt.vx_mps = pnt[5]
            global_wpnt.ax_mps2 = pnt[6]
            if pnt.shape[0] > 7:
                global_wpnt.x_deriv = pnt[7]
                global_wpnt.y_deriv = pnt[8]
                global_wpnt.x_dderiv = pnt[9]
                global_wpnt.y_dderiv = pnt[10]
            else:
                global_wpnt.x_deriv =  0.0
                global_wpnt.y_deriv =  0.0
                global_wpnt.x_dderiv = 0.0
                global_wpnt.y_dderiv = 0.0

            global_wpnts.wpnts.append(global_wpnt)

            global_marker = Marker()
            global_marker.header.frame_id = 'map'
            global_marker.type = global_marker.CYLINDER
            global_marker.scale.x = 0.1
            global_marker.scale.y = 0.1
            global_marker.scale.z = global_wpnt.vx_mps / max_vx_mps
            global_marker.color.a = 1.0
            global_marker.color.r = 1.0
            global_marker.color.g = 1.0 if second_traj else 0.0

            global_marker.id = i
            global_marker.pose.position.x = pnt[1]
            global_marker.pose.position.y = pnt[2]
            global_marker.pose.position.z = global_wpnt.vx_mps / max_vx_mps / 2
            global_marker.pose.orientation.w = 1.0
            global_markers.markers.append(global_marker)

        return global_wpnts, global_markers
    
    def get_bound_markers(bound_r, bound_l, reverse: bool = False) -> MarkerArray:
        """
        Publish the track bounds as a MarkerArray.

        Parameters
        ----------
        bound_r : np.ndarray
            Points of the right track bound in meters
        bound_l : np.ndarray
            Points of the left track bound in meters
        reverse : bool, default=False
            Flag indicating whether the trajectory is being calculated in reverse direction

        Returns
        -------
        bounds_markers : MarkerArray
            Track bounds as a MarkerArray which can be published
        """
        bounds_markers = MarkerArray()
        id_cnt = 0
        if reverse:
            bound_l_real = bound_r.copy()
            bound_r_real = bound_l.copy()
        else:
            bound_l_real = bound_l
            bound_r_real = bound_r

        for i, pnt_r in enumerate(bound_r_real):
            bnd_r_mrk = Marker()
            bnd_r_mrk.header.frame_id = 'map'
            bnd_r_mrk.type = bnd_r_mrk.SPHERE
            bnd_r_mrk.scale.x = 0.05
            bnd_r_mrk.scale.y = 0.05
            bnd_r_mrk.scale.z = 0.05
            bnd_r_mrk.color.a = 1.0
            bnd_r_mrk.color.b = 0.5
            bnd_r_mrk.color.r = 0.5

            bnd_r_mrk.id = id_cnt
            id_cnt += 1
            bnd_r_mrk.pose.position.x = pnt_r[0]
            bnd_r_mrk.pose.position.y = pnt_r[1]
            bnd_r_mrk.pose.orientation.w = 1.0
            bounds_markers.markers.append(bnd_r_mrk)

        for i, pnt_l in enumerate(bound_l_real):
            bnd_l_mrk = Marker()
            bnd_l_mrk.header.frame_id = 'map'
            bnd_l_mrk.type = bnd_l_mrk.SPHERE
            bnd_l_mrk.scale.x = 0.05
            bnd_l_mrk.scale.y = 0.05
            bnd_l_mrk.scale.z = 0.05
            bnd_l_mrk.color.a = 1.0
            bnd_l_mrk.color.r = 0.5
            bnd_l_mrk.color.g = 1.0

            bnd_l_mrk.id = id_cnt
            id_cnt += 1
            bnd_l_mrk.pose.position.x = pnt_l[0]
            bnd_l_mrk.pose.position.y = pnt_l[1]
            bnd_l_mrk.pose.orientation.w = 1.0
            bounds_markers.markers.append(bnd_l_mrk)

        return bounds_markers
    
    # Generate centerline markers
    centerline_wpnts, centerline_markers = get_centerline_markers(centerline)
    # Generate global trajectory markers
    global_wpnts, global_markers = create_wpnts_markers(global_trajectory, d_right, d_left)
    # Generate the boundary markers
    bounds_markers = get_bound_markers(bounds_right, bounds_left)
    
    # Define the structure to save the waypoints
    save_dict = {
        'map_info_str': {'data': map_info},
        'est_lap_time': {'data': est_lap_time},
        'centerline_markers': message_to_ordereddict(centerline_markers),
        'centerline_waypoints':  message_to_ordereddict(centerline_wpnts),
        'global_traj_markers': message_to_ordereddict(global_markers),
        'global_traj_wpnts': message_to_ordereddict(global_wpnts),
        'trackbounds_markers': message_to_ordereddict(bounds_markers)
    }

    with open(output_file, 'w') as f:
        json.dump(save_dict, f, indent=4)

def write_global_trajectory_to_csv(global_trajectory: np.ndarray, left_trackbounds: np.ndarray, right_trackbounds: np.ndarray, output_path: str) -> None:
    """
        Write the global trajectory to a CSV file.

    """
    has_suffix = Path(output_path).suffix != ''
    if not has_suffix:
        output_path += '.csv'

    try:
        with open(output_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["s_m", "x_m", "y_m", "d_left", "d_right", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2", "dx", "dy", "ddx", "ddy"])
            for i, row in enumerate(global_trajectory):
                s_m = row[0]
                x_m = row[1]
                y_m = row[2]
                d_left = left_trackbounds[i]
                d_right = right_trackbounds[i]
                psi_rad = row[3]
                kappa_radpm = row[4]
                vx_mps = row[5]
                ax_mps2 = row[6]
                if row.shape[0] > 7:
                    x_deriv = row[7]
                    y_deriv = row[8]
                    x_dderiv = row[9]
                    y_dderiv = row[10]
                else:
                    x_deriv =  0.0
                    y_deriv =  0.0
                    x_dderiv = 0.0
                    y_dderiv = 0.0
                writer.writerow([s_m, x_m, y_m, d_left, d_right, psi_rad, kappa_radpm, vx_mps, ax_mps2, x_deriv, y_deriv, x_dderiv, y_dderiv])
    except Exception as e:
        print(f"Error writing centerline to file '{output_path}': \n{e}")
        return False
    return True

def load_waypoints(path: str, loginfo= print):

    try:
        with open(path, 'r') as f:
            waypoints = [tuple(line) for line in csv.reader(f)]
        waypoints = waypoints[1:]
        waypoints = np.array(waypoints, dtype=np.float32)
        #print(waypoints)
    except Exception as e:
        loginfo(f"Failed loading waypoints from: {path}\n{e}")
        return None
    return waypoints

def save_waypoints(path: str, data: np.ndarray, loginfo = print):
    """
        Write the global trajectory to a CSV file.

    """
    has_suffix = Path(path).suffix != ''
    if not has_suffix:
        path += '.csv'

    try:
        with open(path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["s_m", "x_m", "y_m", "d_left", "d_right", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2", "dx", "dy", "ddx", "ddy"])
            for row in data:
                s_m = row[0]
                x_m = row[1]
                y_m = row[2]
                d_left = row[3]
                d_right = row[4]
                psi_rad = row[5]
                kappa_radpm = row[6]
                vx_mps = row[7]
                ax_mps2 = row[8]
                if row.shape[0] > 7:
                    x_deriv = row[9]
                    y_deriv = row[10]
                    x_dderiv = row[11]
                    y_dderiv = row[12]
                else:
                    x_deriv =  0.0
                    y_deriv =  0.0
                    x_dderiv = 0.0
                    y_dderiv = 0.0
                writer.writerow([s_m, x_m, y_m, d_left, d_right, psi_rad, kappa_radpm, vx_mps, ax_mps2, x_deriv, y_deriv, x_dderiv, y_dderiv])
    except Exception as e:
        print(f"Error writing centerline to file '{path}': \n{e}")
        return False
    return True

def save_sectors(path: str, sectors: dict):
    data = {}
    if os.path.exists(path):
        with open(path, 'r') as f:
            data = json.load(f)
            
    data["sectors"] = sectors
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)
    

def array_to_wpnt_msg(wpnt_data: np.ndarray, inverse_bounds: bool = False):

    wpnt_array = WpntArray()
    wpnt_array.header.frame_id = "map"

    for i, data_point in enumerate(wpnt_data):
        wpnt = Wpnt()
        #print(data_point)
        wpnt.id = i
        wpnt.s_m = float(data_point[0])
        wpnt.x_m = float(data_point[1])
        wpnt.y_m = float(data_point[2])
        if inverse_bounds:
            wpnt.d_right = float(data_point[3])
            wpnt.d_left =  float(data_point[4])
        else:
            wpnt.d_left =  float(data_point[3])
            wpnt.d_right = float(data_point[4])

        wpnt.psi_rad =      float(data_point[5])
        wpnt.kappa_radpm =  float(data_point[6])
        wpnt.vx_mps =       float(data_point[7])
        wpnt.ax_mps2 =      float(data_point[8])
        if data_point.shape[0] > 9:
            wpnt.x_deriv =  float(data_point[9])
            wpnt.y_deriv =  float(data_point[10])
            wpnt.x_dderiv = float(data_point[11])
            wpnt.y_dderiv = float(data_point[12])
        wpnt_array.wpnts.append(wpnt)

    return wpnt_array

def array_to_trackbounds(wpnt_data: np.ndarray, inverse_bounds:bool=False):

    left_trackbounds, right_trackbounds = [], []

    for i, pnt in enumerate(wpnt_data):
        x = pnt[1]
        y = pnt[2]
        if inverse_bounds:
            d_left = pnt[4]
            d_right = pnt[3]
        else:
            d_left = pnt[3]
            d_right = pnt[4]
        psi = pnt[5]

        #left_normal = np.array([-np.sin(psi), np.cos(psi)])
        #right_normal = -left_normal 
        right_normal = np.array([-pnt[10], pnt[9]])
        left_normal = -right_normal 
        left_boundary = np.array([x, y]) + d_left * left_normal
        right_boundary = np.array([x, y]) + d_right * right_normal

        left_trackbounds.append(left_boundary)
        right_trackbounds.append(right_boundary)

    left_trackbounds = np.array(left_trackbounds)
    right_trackbounds = np.array(right_trackbounds)

    return left_trackbounds, right_trackbounds


def wpnt_msg_to_array(wpnt_array_msg: WpntArray, inverse_bounds: bool = False) -> np.ndarray:
    wpnts = wpnt_array_msg.wpnts
    N = len(wpnts)
    if N == 0:
        return np.zeros((0, 9))

    # Detect whether to include derivative columns
    has_deriv = any(
        (w.x_deriv != 0 or w.y_deriv != 0 or
         w.x_dderiv != 0 or w.y_dderiv != 0)
        for w in wpnts
    )
    cols = 13 if has_deriv else 9
    arr = np.zeros((N, cols), dtype=float)

    for i, w in enumerate(wpnts):
        arr[i, 0] = w.s_m
        arr[i, 1] = w.x_m
        arr[i, 2] = w.y_m

        if inverse_bounds:
            arr[i, 3] = w.d_right
            arr[i, 4] = w.d_left
        else:
            arr[i, 3] = w.d_left
            arr[i, 4] = w.d_right

        arr[i, 5] = w.psi_rad
        arr[i, 6] = w.kappa_radpm
        arr[i, 7] = w.vx_mps
        arr[i, 8] = w.ax_mps2

        if has_deriv:
            arr[i, 9]  = w.x_deriv
            arr[i, 10] = w.y_deriv
            arr[i, 11] = w.x_dderiv
            arr[i, 12] = w.y_dderiv

    return arr

def array_to_wpnt_marker_msg(wpnt_data: np.ndarray, color=(0.8, 0.2, 0.2), show_velocity:bool=True, show_heading: bool = True,
                             show_startline: bool = True) -> MarkerArray:
    max_vx_mps = max(wpnt_data[:, 5])

    flat_scale = 0.05
    arrow_width = 0.02
    arrow_length = 0.05
    start_arrow_width = 0.03
    start_arrow_shaft_width = 0.1
    start_arrow_length = 0.15
    start_arrow_head_length = 0.1
    alpha = 0.8
    marker_size = 0.07

    min_vx, max_vx = np.min(wpnt_data[:, 7]), np.max(wpnt_data[:, 7])


    global_markers = MarkerArray()
    for i, pnt in enumerate(wpnt_data):

        global_marker = Marker()
        global_marker.header.frame_id = 'map'
        global_marker.type = global_marker.CYLINDER
        global_marker.scale.x = marker_size
        global_marker.scale.y = marker_size
        global_marker.scale.z = float(pnt[7] / max_vx) if show_velocity else flat_scale
        global_marker.color.a = alpha
        global_marker.color.r = color[0]
        global_marker.color.g = color[1]
        global_marker.color.b = color[2]

        global_marker.id = i
        global_marker.pose.position.x = float(pnt[1])
        global_marker.pose.position.y = float(pnt[2])
        global_marker.pose.position.z = float(pnt[7]) / max_vx / 2 if show_velocity else flat_scale / 2
        global_marker.pose.orientation.w = 1.0
        global_markers.markers.append(global_marker)

        if show_heading:
            heading_marker = Marker()
            heading_marker.header.frame_id = "map"
            heading_marker.type = Marker.LINE_STRIP
            heading_marker.ns = "heading"
            heading_marker.id = i

            heading_marker.scale.x = arrow_width  # line width
            heading_marker.color.r = 0.1
            heading_marker.color.g = 0.1
            heading_marker.color.b = 0.1
            heading_marker.color.a = alpha

            pt1 = np.array([pnt[1],pnt[2]])
            pt1_point = Point()
            pt1_point.x = float(pt1[0])
            pt1_point.y = float(pt1[1])
            heading_vec = np.array([np.cos(pnt[5]+np.pi/2), np.sin(pnt[5]+np.pi/2)])
            pt2 = pt1 + heading_vec  * arrow_length
            pt2_point = Point()
            pt2_point.x = float(pt2[0])
            pt2_point.y = float(pt2[1])

            heading_marker.points.append(pt1_point)
            heading_marker.points.append(pt2_point)
            global_markers.markers.append(heading_marker)
        if show_startline and i==0:
            psi = pnt[5]
            right_normal = np.array([np.cos(psi), np.sin(psi)])
            left_normal = -right_normal
            d_right, d_left = pnt[3], pnt[4]

            left_boundary =  pnt[1:3] + d_left * left_normal
            right_boundary = pnt[1:3] + d_right * right_normal
            left_mid_boundary =  pnt[1:3] + d_left/2 * left_normal
            right_mid_boundary = pnt[1:3] + d_right/2 * right_normal

            left_point = Point()
            left_point.x = float(left_boundary[0])
            left_point.y = float(left_boundary[1])
            right_point = Point()
            right_point.x = float(right_boundary[0])
            right_point.y = float(right_boundary[1])

            start_line_marker = Marker()
            start_line_marker.header.frame_id = "map"
            start_line_marker.type = Marker.LINE_STRIP
            start_line_marker.ns = "start_line"
            start_line_marker.id = 0

            start_line_marker.scale.x = 0.03
            start_line_marker.color.r = 0.1
            start_line_marker.color.g = 0.1
            start_line_marker.color.b = 0.1
            start_line_marker.color.a = alpha

            start_line_marker.points.append(left_point)
            start_line_marker.points.append(right_point)

            global_markers.markers.append(start_line_marker)

           # Create left arrow marker, starting at left_mid_boundary and pointing along the heading
            left_mid_point = Point()
            left_mid_point.x = float(left_mid_boundary[0])
            left_mid_point.y = float(left_mid_boundary[1])
            left_arrow_tip = left_mid_boundary + np.array([np.cos(psi-np.pi/2), np.sin(psi-np.pi/2)]) * start_arrow_length
            left_arrow_tip_point = Point()
            left_arrow_tip_point.x = float(left_arrow_tip[0])
            left_arrow_tip_point.y = float(left_arrow_tip[1])

            start_line_arrow_marker1 = Marker()
            start_line_arrow_marker1.header.frame_id = "map"
            start_line_arrow_marker1.type = Marker.ARROW
            start_line_arrow_marker1.ns = "start_line"
            start_line_arrow_marker1.id = 1
            start_line_arrow_marker1.scale.x = start_arrow_width
            start_line_arrow_marker1.scale.y = start_arrow_shaft_width
            start_line_arrow_marker1.scale.z= start_arrow_head_length
            start_line_arrow_marker1.color.r = 0.1
            start_line_arrow_marker1.color.g = 0.1
            start_line_arrow_marker1.color.b = 0.1
            start_line_arrow_marker1.color.a = alpha
            start_line_arrow_marker1.points.append(left_mid_point)
            start_line_arrow_marker1.points.append(left_arrow_tip_point)

            global_markers.markers.append(start_line_arrow_marker1)

            # Create right arrow marker, starting at right_mid_boundary and pointing along the heading
            right_mid_point = Point()
            right_mid_point.x = float(right_mid_boundary[0])
            right_mid_point.y = float(right_mid_boundary[1])
            right_arrow_tip = right_mid_boundary + np.array([np.cos(psi-np.pi/2), np.sin(psi-np.pi/2)]) * start_arrow_length
            right_arrow_tip_point = Point()
            right_arrow_tip_point.x = float(right_arrow_tip[0])
            right_arrow_tip_point.y = float(right_arrow_tip[1])

            start_line_arrow_marker2 = Marker()
            start_line_arrow_marker2.header.frame_id = "map"
            start_line_arrow_marker2.type = Marker.ARROW
            start_line_arrow_marker2.ns = "start_line"
            start_line_arrow_marker2.id = 2
            start_line_arrow_marker2.scale.x = start_arrow_width
            start_line_arrow_marker2.scale.y = start_arrow_shaft_width
            start_line_arrow_marker2.scale.z= start_arrow_head_length
            start_line_arrow_marker2.color.r = 0.1
            start_line_arrow_marker2.color.g = 0.1
            start_line_arrow_marker2.color.b = 0.1
            start_line_arrow_marker2.color.a = alpha
            start_line_arrow_marker2.points.append(right_mid_point)
            start_line_arrow_marker2.points.append(right_arrow_tip_point)

            global_markers.markers.append(start_line_arrow_marker2)

    return global_markers

def load_sector_file(path: str):
    sectors = None
    try:
        with open(path, "r") as f:
            sector_data = json.load(f)
        sectors = sector_data["sectors"]
    except Exception as e:
        print(f"Failed to load sectors from '{path}': {e}")
    return sectors

def sector_dict_to_sector_msg(sectors):
    from f110_msgs.msg import Sector, SectorArray
    sector_array = SectorArray()
    sector_array.header.frame_id = "map"
    for i, sector in enumerate(sectors):
        sector_msg = Sector()
        sector_msg.start_s = float(sector["start_s"])
        sector_msg.end_s = float(sector["end_s"])
        sector_msg.start_ind = int(sector["start_ind"])
        sector_msg.end_ind = int(sector["end_ind"])
        sector_msg.v_max = float(sector["v_max"])
        sector_msg.l1_scale = float(sector["l1_scale"])
        sector_msg.l1_shift = float(sector["l1_shift"])
        sector_msg.l1_min = float(sector["l1_min"])
        sector_msg.l1_max = float(sector["l1_max"])
        sector_msg.fallback = sector["fallback"]
        sector_msg.overtaking = sector["overtaking"]
        sector_msg.force_fallback = sector.get("force_fallback", False)
        sector_msg.bang_bang = sector.get("bang_bang", False)
        sector_array.sectors.append(sector_msg)
    return sector_array

def sector_msg_to_dict(sector_array):
    sectors = []
    for sector in sector_array.sectors:
        sector_dict = {
            "start_s": sector.start_s,
            "end_s": sector.end_s,
            "start_ind": sector.start_ind,
            "end_ind": sector.end_ind,
            "v_max": sector.v_max,
            "l1_scale": sector.l1_scale,
            "l1_shift": sector.l1_shift,
            "l1_min": sector.l1_min,
            "l1_max": sector.l1_max,
            "fallback": sector.fallback,
            "overtaking": sector.overtaking,
            "force_fallback": sector.force_fallback,
            "bang_bang": sector.bang_bang
        }
        sectors.append(sector_dict)
    return sectors

def sector_dict_to_marker_msg(sectors, wpnt_data: np.ndarray):
    marker_array = MarkerArray()

    flat_scale = 0.05
    arrow_width = 0.02
    arrow_length = 0.05
    start_arrow_width = 0.03
    start_arrow_shaft_width = 0.1
    start_arrow_length = 0.15
    start_arrow_head_length = 0.1
    alpha = 0.8
    marker_size = 0.07
    color = (0.8, 0.2, 0.2, alpha)
    wpnt_id = 0

    for i, sector in enumerate(sectors):
        pnt = wpnt_data[int(sector["start_ind"])]
        psi = pnt[5]
        right_normal = np.array([np.cos(psi), np.sin(psi)])
        left_normal = -right_normal
        d_right, d_left = pnt[3], pnt[4]

        left_boundary =  pnt[1:3] + d_left * left_normal
        right_boundary = pnt[1:3] + d_right * right_normal
        left_mid_boundary =  pnt[1:3] + d_left/2 * left_normal
        right_mid_boundary = pnt[1:3] + d_right/2 * right_normal

        left_point = Point()
        left_point.x = float(left_boundary[0])
        left_point.y = float(left_boundary[1])
        right_point = Point()
        right_point.x = float(right_boundary[0])
        right_point.y = float(right_boundary[1])

        start_line_marker = Marker()
        start_line_marker.header.frame_id = "map"
        start_line_marker.type = Marker.LINE_STRIP
        start_line_marker.ns = "start_line"
        start_line_marker.id = wpnt_id

        wpnt_id += 1

        start_line_marker.scale.x = 0.03
        start_line_marker.color.r = 0.4
        start_line_marker.color.g = 0.4
        start_line_marker.color.b = 0.4
        start_line_marker.color.a = alpha

        start_line_marker.points.append(left_point)
        start_line_marker.points.append(right_point)

        marker_array.markers.append(start_line_marker)

        # Create left arrow marker, starting at left_mid_boundary and pointing along the heading
        left_mid_point = Point()
        left_mid_point.x = float(left_mid_boundary[0])
        left_mid_point.y = float(left_mid_boundary[1])
        left_arrow_tip = left_mid_boundary + np.array([np.cos(psi-np.pi/2), np.sin(psi-np.pi/2)]) * start_arrow_length
        left_arrow_tip_point = Point()
        left_arrow_tip_point.x = float(left_arrow_tip[0])
        left_arrow_tip_point.y = float(left_arrow_tip[1])

        start_line_arrow_marker1 = Marker()
        start_line_arrow_marker1.header.frame_id = "map"
        start_line_arrow_marker1.type = Marker.ARROW
        start_line_arrow_marker1.ns = "start_line"
        start_line_arrow_marker1.id = wpnt_id
        start_line_arrow_marker1.scale.x = start_arrow_width
        start_line_arrow_marker1.scale.y = start_arrow_shaft_width
        start_line_arrow_marker1.scale.z = start_arrow_head_length
        start_line_arrow_marker1.color.r = 0.4
        start_line_arrow_marker1.color.g = 0.4
        start_line_arrow_marker1.color.b = 0.4
        start_line_arrow_marker1.color.a = alpha
        start_line_arrow_marker1.points.append(left_mid_point)
        start_line_arrow_marker1.points.append(left_arrow_tip_point)

        wpnt_id += 1

        marker_array.markers.append(start_line_arrow_marker1)

        # Create right arrow marker, starting at right_mid_boundary and pointing along the heading
        right_mid_point = Point()
        right_mid_point.x = float(right_mid_boundary[0])
        right_mid_point.y = float(right_mid_boundary[1])
        right_arrow_tip = right_mid_boundary + np.array([np.cos(psi-np.pi/2), np.sin(psi-np.pi/2)]) * start_arrow_length
        right_arrow_tip_point = Point()
        right_arrow_tip_point.x = float(right_arrow_tip[0])
        right_arrow_tip_point.y = float(right_arrow_tip[1])

        start_line_arrow_marker2 = Marker()
        start_line_arrow_marker2.header.frame_id = "map"
        start_line_arrow_marker2.type = Marker.ARROW
        start_line_arrow_marker2.ns = "start_line"
        start_line_arrow_marker2.id = wpnt_id
        start_line_arrow_marker2.scale.x = start_arrow_width
        start_line_arrow_marker2.scale.y = start_arrow_shaft_width
        start_line_arrow_marker2.scale.z = start_arrow_head_length
        start_line_arrow_marker2.color.r = 0.4
        start_line_arrow_marker2.color.g = 0.4
        start_line_arrow_marker2.color.b = 0.4
        start_line_arrow_marker2.color.a = alpha
        start_line_arrow_marker2.points.append(right_mid_point)
        start_line_arrow_marker2.points.append(right_arrow_tip_point)

        marker_array.markers.append(start_line_arrow_marker2)

        wpnt_id += 1

        # Add a text marker to label the sector above the sector's midpoint.
        mid_pnt_index = int((sector["start_ind"]+sector["end_ind"])//2)
        pnt = wpnt_data[mid_pnt_index]

        middle_point = Point()
        middle_point.x = float(pnt[1])
        middle_point.y = float(pnt[2])
        # Set a z-offset so the text appears above
        middle_point.z = 1.0

        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "sector_text"
        text_marker.id = wpnt_id  # ensure unique id for text markers
        text_marker.scale.z = 0.3  # text height
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.pose.position = middle_point
        text_marker.text = f"Sector {i}"

        wpnt_id += 1

        marker_array.markers.append(text_marker)

    return marker_array

def array_to_bound_marker_msg(wpnt_data: np.ndarray, inverse_bounds:bool=False, alpha:float=1.0):
    wpnt_id = 1
    bounds_markers = MarkerArray()

    def _define_marker(pose, color=(0.,0.,0.,1.), z_offset:int=0.01, scale:float=0.05):
        nonlocal wpnt_id
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = 0.01
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = z_offset
        marker.pose.orientation.x = 0.
        marker.pose.orientation.y = 0.
        marker.pose.orientation.z = 0.
        marker.pose.orientation.w = 1.
        marker.id = wpnt_id
        wpnt_id += 1
        return marker
    
    for i, pnt in enumerate(wpnt_data):
        x = pnt[1]
        y = pnt[2]
        psi = pnt[5] #+ np.pi/2

        if inverse_bounds:
            d_left, d_right = pnt[4], pnt[3]
        else:
            d_right, d_left = pnt[3], pnt[4]


        right_normal = np.array([np.cos(psi), np.sin(psi)])
        left_normal = -right_normal

        left_boundary = np.array([x, y]) + d_left * left_normal
        right_boundary = np.array([x, y]) + d_right * right_normal

        left_marker = _define_marker(left_boundary, color=(0.2, 0.2, 1.0, alpha))
        right_marker = _define_marker(right_boundary, color=(0.8, 0.2, 0.2, alpha))

        bounds_markers.markers.append(left_marker)
        bounds_markers.markers.append(right_marker)

    return bounds_markers
    



    
    

