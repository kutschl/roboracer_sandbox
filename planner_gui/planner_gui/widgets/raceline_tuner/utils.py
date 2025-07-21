
import numpy as np
import trajectory_planning_helpers as tph
from trajectory_planning_helpers.spline_approximation import spline_approximation
from trajectory_planning_helpers.calc_splines import calc_splines
from trajectory_planning_helpers.check_normals_crossing import check_normals_crossing
import sys
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import math
from typing import Union
from skimage.segmentation import watershed
import cv2 as cv
import json



def prep_track_no_interp(reftrack_imp: np.ndarray,
                         reg_smooth_opts: dict,
                         stepsize_opts: dict,
                         debug: bool = True,
                         min_width: float = None) -> tuple:
    """
    Prepares the reference track for optimization without any interpolation.
    The output raceline corresponds directly to the input, except that a minimum
    track width is enforced if requested.

    Inputs:
      reftrack_imp:      Imported track [x_m, y_m, w_tr_right_m, w_tr_left_m]
      reg_smooth_opts:   Parameters for spline approximation (unused in this version)
      stepsize_opts:     Dictionary containing the stepsizes (unused in this version)
      debug:             Flag for printing debug messages.
      min_width:         [m] Minimum enforced track width (None to deactivate)

    Outputs:
      reftrack_interp:            Original raceline (possibly with width inflation)
      normvec_normalized_interp:  Normalized normal vectors on the reference line [x_m, y_m]
      a_interp:                   LES coefficients from the spline linear system
      coeffs_x_interp:            Spline coefficients for the x-component (per segment)
      coeffs_y_interp:            Spline coefficients for the y-component (per segment)
    """
    # Use the original raceline without any interpolation.
    reftrack_interp = np.copy(reftrack_imp)

    # Enforce minimum track width if specified.
    manipulated_track_width = False
    if min_width is not None:
        for i in range(reftrack_interp.shape[0]):
            cur_width = reftrack_interp[i, 2] + reftrack_interp[i, 3]
            if cur_width < min_width:
                manipulated_track_width = True
                # Inflate both sides equally.
                delta = (min_width - cur_width) / 2
                reftrack_interp[i, 2] += delta
                reftrack_interp[i, 3] += delta

    if manipulated_track_width:
        print("WARNING: Track region was smaller than the requested minimum track width -> Applied artificial inflation.",
              file=sys.stderr)

    # Create a closed version of the raceline by appending the first point at the end.
    #closing_length = np.linalg.norm(reftrack_interp[-1, :2]-reftrack_interp[0, :2])
    #if closing_length < 5e-2:
    #    refpath_interp_cl = np.vstack((reftrack_interp[:-1, :2], reftrack_interp[0, :2]))
    #else:
    refpath_interp_cl = np.vstack((reftrack_interp[:, :2], reftrack_interp[0, :2]))  # close the curve for spline computation
        
    

    # Compute the spline coefficients using the new function that does not apply any interpolation fixes.
    coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp = calc_splines_no_interp(refpath_interp_cl)

    return reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp


def calc_splines_no_interp(path: np.ndarray) -> tuple:
    """
    Computes curvature continuous cubic splines for a closed raceline
    without any interpolation fixes (i.e. using uniform parameterization).
    
    Assumes that the input path is closed (the first and last points are identical).
    
    For each segment i (with i = 0,..., n-1 where n = number of segments),
    the cubic polynomial is given by:
        P(t) = a3 * t^3 + a2 * t^2 + a1 * t + a0,  t in [0,1]
    
    The following conditions are enforced:
      - Position continuity:
          P_i(0) = point_i,
          P_i(1) = point_{i+1}
      - First derivative continuity:
          For segments i = 0,..., n-2:  P_i'(1) = P_{i+1}'(0)
          For the last segment:         P_{n-1}'(1) = P_0'(0)
      - Second derivative continuity:
          For segments i = 0,..., n-2:  P_i''(1) = P_{i+1}''(0)
          For the last segment:         P_{n-1}''(1) = P_0''(0)
    
    Outputs:
      coeffs_x: Spline coefficients for the x-component (shape: [n_segments, 4])
      coeffs_y: Spline coefficients for the y-component (shape: [n_segments, 4])
      M:        The linear system matrix used for the spline computation.
      normvec_normalized: Normalized normal vectors (based on the derivative coefficients),
                          shape: [n_segments, 2] where each row is [coeffs_y[i,1], -coeffs_x[i,1]] normalized.
    """
    # Verify that the path is closed.
    if not np.allclose(path[0], path[-1]):
        raise ValueError("Path must be closed: first and last points must be identical.")

    n_points = path.shape[0]
    n_segments = n_points - 1  # Last point is a duplicate of the first.
    N = 4 * n_segments      # Total number of unknowns (per coordinate).

    M = np.zeros((N, N))
    b_x = np.zeros((N, 1))
    b_y = np.zeros((N, 1))

    # Loop over each segment to fill the system.
    for i in range(n_segments):
        idx = 4 * i
        # Position condition at start of segment i: a0_i = point_i.
        M[idx, idx] = 1.0
        b_x[idx] = path[i, 0]
        b_y[idx] = path[i, 1]

        # Position condition at end of segment i: a0_i + a1_i + a2_i + a3_i = point_{i+1}.
        M[idx+1, idx:idx+4] = 1.0
        b_x[idx+1] = path[i+1, 0]
        b_y[idx+1] = path[i+1, 1]

        # First derivative continuity.
        # For segments 0 to n_segments-2:  a1_i + 2a2_i + 3a3_i - a1_{i+1} = 0.
        if i < n_segments - 1:
            M[idx+2, idx+1] = 1.0
            M[idx+2, idx+2] = 2.0
            M[idx+2, idx+3] = 3.0
            M[idx+2, idx+4 + 1] = -1.0  # a1 of the next segment.
        else:
            # For the last segment, enforce periodic derivative continuity:
            # a1_{n-1} + 2a2_{n-1} + 3a3_{n-1} - a1_0 = 0.
            M[idx+2, idx+1] = 1.0
            M[idx+2, idx+2] = 2.0
            M[idx+2, idx+3] = 3.0
            M[idx+2, 1] = -1.0

        # Second derivative continuity.
        # For segments 0 to n_segments-2:  2a2_i + 6a3_i - 2a2_{i+1} = 0.
        if i < n_segments - 1:
            M[idx+3, idx+2] = 2.0
            M[idx+3, idx+3] = 6.0
            M[idx+3, idx+4 + 2] = -2.0  # a2 of the next segment.
        else:
            # For the last segment, enforce periodic second derivative continuity:
            # 2a2_{n-1} + 6a3_{n-1} - 2a2_0 = 0.
            M[idx+3, idx+2] = 2.0
            M[idx+3, idx+3] = 6.0
            M[idx+3, 2] = -2.0

    # Solve the linear system for x and y coefficients.
    x_les = np.squeeze(np.linalg.solve(M, b_x))
    y_les = np.squeeze(np.linalg.solve(M, b_y))

    # Reshape solutions into arrays with one row per segment and four coefficients per row.
    coeffs_x = x_les.reshape((n_segments, 4))
    coeffs_y = y_les.reshape((n_segments, 4))

    # Compute normal vectors from the first derivative coefficients.
    # Here, the derivative at t=0 for each segment is given by the coefficient a1.
    normvec = np.stack((coeffs_y[:, 1], -coeffs_x[:, 1]), axis=1)
    norms = np.linalg.norm(normvec, axis=1, keepdims=True)
    normvec_normalized = normvec / norms

    return coeffs_x, coeffs_y, M, normvec_normalized


def prep_track(reftrack_imp: np.ndarray,
               reg_smooth_opts: dict,
               stepsize_opts: dict,
               debug: bool = True,
               min_width: float = None) -> tuple:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    This function prepares the inserted reference track for optimization.

    Inputs:
    reftrack_imp:               imported track [x_m, y_m, w_tr_right_m, w_tr_left_m]
    reg_smooth_opts:            parameters for the spline approximation
    stepsize_opts:              dict containing the stepsizes before spline approximation and after spline interpolation
    debug:                      boolean showing if debug messages should be printed
    min_width:                  [m] minimum enforced track width (None to deactivate)

    Outputs:
    reftrack_interp:            track after smoothing and interpolation [x_m, y_m, w_tr_right_m, w_tr_left_m]
    normvec_normalized_interp:  normalized normal vectors on the reference line [x_m, y_m]
    a_interp:                   LES coefficients when calculating the splines
    coeffs_x_interp:            spline coefficients of the x-component
    coeffs_y_interp:            spline coefficients of the y-component
    """

    # ------------------------------------------------------------------------------------------------------------------
    # INTERPOLATE REFTRACK AND CALCULATE INITIAL SPLINES ---------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # smoothing and interpolating reference track
    reftrack_interp = spline_approximation(track=reftrack_imp,
                             k_reg=reg_smooth_opts["k_reg"],
                             s_reg=reg_smooth_opts["s_reg"],
                             stepsize_prep=stepsize_opts["stepsize_prep"],
                             stepsize_reg=stepsize_opts["stepsize_reg"],
                             debug=debug)

    # calculate splines
    refpath_interp_cl = np.vstack((reftrack_interp[:, :2], reftrack_interp[0, :2]))

    coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp = calc_splines(path=refpath_interp_cl)

    # ------------------------------------------------------------------------------------------------------------------
    # ENFORCE MINIMUM TRACK WIDTH (INFLATE TIGHTER SECTIONS UNTIL REACHED) ---------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    manipulated_track_width = False

    if min_width is not None:
        for i in range(reftrack_interp.shape[0]):
            cur_width = reftrack_interp[i, 2] + reftrack_interp[i, 3]

            if cur_width < min_width:
                manipulated_track_width = True

                # inflate to both sides equally
                reftrack_interp[i, 2] += (min_width - cur_width) / 2
                reftrack_interp[i, 3] += (min_width - cur_width) / 2

    if manipulated_track_width:
        print("WARNING: Track region was smaller than requested minimum track width -> Applied artificial inflation in"
              " order to match the requirements!", file=sys.stderr)

    return reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp



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
    # filter_length = 20
    centerline_length = len(centerline)
    # print("Number of centerline points: ", centerline_length)

    if centerline_length > 2000:
        filter_length = int(centerline_length / 200) * 10 + 1
    elif centerline_length > 1000:
        filter_length = 81
    elif centerline_length > 500:
        filter_length = 41
    else:
        filter_length = 21
        #TODO filter size 
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

def normalize_psi(psi: Union[np.ndarray, float]) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Normalize heading psi such that [-pi,pi[ holds as interval boundaries.

    .. inputs::
    :param psi:         array containing headings psi to be normalized.
    :type psi:          Union[np.ndarray, float]

    .. outputs::
    :return psi_out:    array with normalized headings psi.
    :rtype psi_out:     np.ndarray

    .. notes::
    len(psi) = len(psi_out)
    """

    # use modulo operator to remove multiples of 2*pi
    psi_out = np.sign(psi) * np.mod(np.abs(psi), 2 * math.pi)

    # restrict psi to [-pi,pi[
    if type(psi_out) is np.ndarray:
        psi_out[psi_out >= math.pi] -= 2 * math.pi
        psi_out[psi_out < -math.pi] += 2 * math.pi

    else:
        if psi_out >= math.pi:
            psi_out -= 2 * math.pi
        elif psi_out < -math.pi:
            psi_out += 2 * math.pi

    return psi_out

def calc_head_curv_an(coeffs_x: np.ndarray,
                      coeffs_y: np.ndarray,
                      ind_spls: np.ndarray,
                      t_spls: np.ndarray,
                      calc_curv: bool = True,
                      calc_dcurv: bool = False) -> tuple:
    """
    author:
    Alexander Heilmeier
    Marvin Ochsenius (dcurv extension)

    .. description::
    Analytical calculation of heading psi, curvature kappa, and first derivative of the curvature dkappa
    on the basis of third order splines for x- and y-coordinate.

    .. inputs::
    :param coeffs_x:    coefficient matrix of the x splines with size (no_splines x 4).
    :type coeffs_x:     np.ndarray
    :param coeffs_y:    coefficient matrix of the y splines with size (no_splines x 4).
    :type coeffs_y:     np.ndarray
    :param ind_spls:    contains the indices of the splines that hold the points for which we want to calculate heading/curv.
    :type ind_spls:     np.ndarray
    :param t_spls:      containts the relative spline coordinate values (t) of every point on the splines.
    :type t_spls:       np.ndarray
    :param calc_curv:   bool flag to show if curvature should be calculated as well (kappa is set 0.0 otherwise).
    :type calc_curv:    bool
    :param calc_dcurv:  bool flag to show if first derivative of curvature should be calculated as well.
    :type calc_dcurv:   bool

    .. outputs::
    :return psi:        heading at every point.
    :rtype psi:         float
    :return kappa:      curvature at every point.
    :rtype kappa:       float
    :return dkappa:     first derivative of curvature at every point (if calc_dcurv bool flag is True).
    :rtype dkappa:      float

    .. notes::
    len(ind_spls) = len(t_spls) = len(psi) = len(kappa) = len(dkappa)
    """

    # check inputs
    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise ValueError("Coefficient matrices must have the same length!")

    if ind_spls.size != t_spls.size:
        raise ValueError("ind_spls and t_spls must have the same length!")

    if not calc_curv and calc_dcurv:
        raise ValueError("dkappa cannot be calculated without kappa!")

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE HEADING ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate required derivatives
    x_d = coeffs_x[ind_spls, 1] \
          + 2 * coeffs_x[ind_spls, 2] * t_spls \
          + 3 * coeffs_x[ind_spls, 3] * np.power(t_spls, 2)

    y_d = coeffs_y[ind_spls, 1] \
          + 2 * coeffs_y[ind_spls, 2] * t_spls \
          + 3 * coeffs_y[ind_spls, 3] * np.power(t_spls, 2)

    # calculate heading psi (pi/2 must be substracted due to our convention that psi = 0 is north)
    psi = np.arctan2(y_d, x_d) - math.pi / 2
    psi = normalize_psi(psi)

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE CURVATURE ----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if calc_curv:
        # calculate required derivatives
        x_dd = 2 * coeffs_x[ind_spls, 2] \
               + 6 * coeffs_x[ind_spls, 3] * t_spls

        y_dd = 2 * coeffs_y[ind_spls, 2] \
               + 6 * coeffs_y[ind_spls, 3] * t_spls

        # calculate curvature kappa
        kappa = (x_d * y_dd - y_d * x_dd) / np.power(np.power(x_d, 2) + np.power(y_d, 2), 1.5)

    else:
        kappa = 0.0

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE FIRST DERIVATIVE OF CURVATURE --------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if calc_dcurv:
        # calculate required derivatives
        x_ddd = 6 * coeffs_x[ind_spls, 3]

        y_ddd = 6 * coeffs_y[ind_spls, 3]

        # calculate first derivative of curvature dkappa
        dkappa = ((np.power(x_d, 2) + np.power(y_d, 2)) * (x_d * y_ddd - y_d * x_ddd) -
                  3 * (x_d * y_dd - y_d * x_dd) * (x_d * x_dd + y_d * y_dd)) / \
                 np.power(np.power(x_d, 2) + np.power(y_d, 2), 3)

        return psi, kappa, dkappa

    else:

        return psi, kappa

def conv_filt(signal: np.ndarray,
              filt_window: int,
              closed: bool) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    modified by:
    Tim Stahl

    .. description::
    Filter a given temporal signal using a convolution (moving average) filter.

    .. inputs::
    :param signal:          temporal signal that should be filtered (always unclosed).
    :type signal:           np.ndarray
    :param filt_window:     filter window size for moving average filter (must be odd).
    :type filt_window:      int
    :param closed:          flag showing if the signal can be considered as closable, e.g. for velocity profiles.
    :type closed:           bool

    .. outputs::
    :return signal_filt:    filtered input signal (always unclosed).
    :rtype signal_filt:     np.ndarray

    .. notes::
    signal input is always unclosed!

    len(signal) = len(signal_filt)
    """

    # check if window width is odd
    if not filt_window % 2 == 1:
        raise RuntimeError("Window width of moving average filter must be odd!")

    # calculate half window width - 1
    w_window_half = int((filt_window - 1) / 2)

    # apply filter
    if closed:
        # temporarily add points in front of and behind signal
        signal_tmp = np.concatenate((signal[-w_window_half:], signal, signal[:w_window_half]), axis=0)

        # apply convolution filter used as a moving average filter and remove temporary points
        signal_filt = np.convolve(signal_tmp,
                                  np.ones(filt_window) / float(filt_window),
                                  mode="same")[w_window_half:-w_window_half]

    else:
        # implementation 1: include boundaries during filtering
        # no_points = signal.size
        # signal_filt = np.zeros(no_points)
        #
        # for i in range(no_points):
        #     if i < w_window_half:
        #         signal_filt[i] = np.average(signal[:i + w_window_half + 1])
        #
        #     elif i < no_points - w_window_half:
        #         signal_filt[i] = np.average(signal[i - w_window_half:i + w_window_half + 1])
        #
        #     else:
        #         signal_filt[i] = np.average(signal[i - w_window_half:])

        # implementation 2: start filtering at w_window_half and stop at -w_window_half
        signal_filt = np.copy(signal)
        signal_filt[w_window_half:-w_window_half] = np.convolve(signal,
                                                                np.ones(filt_window) / float(filt_window),
                                                                mode="same")[w_window_half:-w_window_half]

    return signal_filt



def calc_vel_profile(ax_max_machines: np.ndarray,
                     kappa: np.ndarray,
                     el_lengths: np.ndarray,
                     closed: bool,
                     drag_coeff: float,
                     m_veh: float,
                     ggv: np.ndarray = None,
                     loc_gg: np.ndarray = None,
                     v_max: float = None,
                     dyn_model_exp: float = 1.0,
                     mu: np.ndarray = None,
                     v_start: float = None,
                     v_end: float = None,
                     filt_window: int = None) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    modified by:
    Tim Stahl

    .. description::
    Calculates a velocity profile using the tire and motor limits as good as possible.

    .. inputs::
    :param ax_max_machines: longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
                            in m/s, accelerations in m/s2. They should be handed in without considering drag resistance,
                            i.e. simply by calculating F_x_drivetrain / m_veh
    :type ax_max_machines:  np.ndarray
    :param kappa:           curvature profile of given trajectory in rad/m (always unclosed).
    :type kappa:            np.ndarray
    :param el_lengths:      element lengths (distances between coordinates) of given trajectory.
    :type el_lengths:       np.ndarray
    :param closed:          flag to set if the velocity profile must be calculated for a closed or unclosed trajectory.
    :type closed:           bool
    :param drag_coeff:      drag coefficient including all constants: drag_coeff = 0.5 * c_w * A_front * rho_air
    :type drag_coeff:       float
    :param m_veh:           vehicle mass in kg.
    :type m_veh:            float
    :param ggv:             ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
                            ATTENTION: Insert either ggv + mu (optional) or loc_gg!
    :type ggv:              np.ndarray
    :param loc_gg:          local gg diagrams along the path points: [[ax_max_0, ay_max_0], [ax_max_1, ay_max_1], ...],
                            accelerations in m/s2. ATTENTION: Insert either ggv + mu (optional) or loc_gg!
    :type loc_gg:           np.ndarray
    :param v_max:           Maximum longitudinal speed in m/s (optional if ggv is supplied, taking the minimum of the
                            fastest velocities covered by the ggv and ax_max_machines arrays then).
    :type v_max:            float
    :param dyn_model_exp:   exponent used in the vehicle dynamics model (usual range [1.0,2.0]).
    :type dyn_model_exp:    float
    :param mu:              friction coefficients (always unclosed).
    :type mu:               np.ndarray
    :param v_start:         start velocity in m/s (used in unclosed case only).
    :type v_start:          float
    :param v_end:           end velocity in m/s (used in unclosed case only).
    :type v_end:            float
    :param filt_window:     filter window size for moving average filter (must be odd).
    :type filt_window:      int

    .. outputs::
    :return vx_profile:     calculated velocity profile (always unclosed).
    :rtype vx_profile:      np.ndarray

    .. notes::
    All inputs must be inserted unclosed, i.e. kappa[-1] != kappa[0], even if closed is set True! (el_lengths is kind of
    closed if closed is True of course!)

    case closed is True:
    len(kappa) = len(el_lengths) = len(mu) = len(vx_profile)

    case closed is False:
    len(kappa) = len(el_lengths) + 1 = len(mu) = len(vx_profile)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # INPUT CHECKS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check if either ggv (and optionally mu) or loc_gg are handed in
    if (ggv is not None or mu is not None) and loc_gg is not None:
        raise RuntimeError("Either ggv and optionally mu OR loc_gg must be supplied, not both (or all) of them!")

    if ggv is None and loc_gg is None:
        raise RuntimeError("Either ggv or loc_gg must be supplied!")

    # check shape of loc_gg
    if loc_gg is not None:
        if loc_gg.ndim != 2:
            raise RuntimeError("loc_gg must have two dimensions!")

        if loc_gg.shape[0] != kappa.size:
            raise RuntimeError("Length of loc_gg and kappa must be equal!")

        if loc_gg.shape[1] != 2:
            raise RuntimeError("loc_gg must consist of two columns: [ax_max, ay_max]!")

    # check shape of ggv
    if ggv is not None and ggv.shape[1] != 3:
        raise RuntimeError("ggv diagram must consist of the three columns [vx, ax_max, ay_max]!")

    # check size of mu
    if mu is not None and kappa.size != mu.size:
        raise RuntimeError("kappa and mu must have the same length!")

    # check size of kappa and element lengths
    if closed and kappa.size != el_lengths.size:
        raise RuntimeError("kappa and el_lengths must have the same length if closed!")

    elif not closed and kappa.size != el_lengths.size + 1:
        raise RuntimeError("kappa must have the length of el_lengths + 1 if unclosed!")

    # check start and end velocities
    if not closed and v_start is None:
        raise RuntimeError("v_start must be provided for the unclosed case!")

    if v_start is not None and v_start < 0.0:
        v_start = 0.0
        print('WARNING: Input v_start was < 0.0. Using v_start = 0.0 instead!')

    if v_end is not None and v_end < 0.0:
        v_end = 0.0
        print('WARNING: Input v_end was < 0.0. Using v_end = 0.0 instead!')

    # check dyn_model_exp
    if not 1.0 <= dyn_model_exp <= 2.0:
        print('WARNING: Exponent for the vehicle dynamics model should be in the range [1.0, 2.0]!')

    # check shape of ax_max_machines
    if ax_max_machines.shape[1] != 2:
        raise RuntimeError("ax_max_machines must consist of the two columns [vx, ax_max_machines]!")

    # check v_max
    if v_max is None:
        if ggv is None:
            raise RuntimeError("v_max must be supplied if ggv is None!")
        else:
            v_max = min(ggv[-1, 0], ax_max_machines[-1, 0])

    else:
        # check if ggv covers velocity until v_max
        if ggv is not None and ggv[-1, 0] < v_max:
            raise RuntimeError("ggv has to cover the entire velocity range of the car (i.e. >= v_max)!")

        # check if ax_max_machines covers velocity until v_max
        if ax_max_machines[-1, 0] < v_max:
            raise RuntimeError("ax_max_machines has to cover the entire velocity range of the car (i.e. >= v_max)!")

    # ------------------------------------------------------------------------------------------------------------------
    # BRINGING GGV OR LOC_GG INTO SHAPE FOR EQUAL HANDLING AFTERWARDS --------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    """For an equal/easier handling of every case afterwards we bring all cases into a form where the local ggv is made
    available for every waypoint, i.e. [ggv_0, ggv_1, ggv_2, ...] -> we have a three dimensional array p_ggv (path_ggv)
    where the first dimension is the waypoint, the second is the velocity and the third is the two acceleration columns
    -> DIM = NO_WAYPOINTS_CLOSED x NO_VELOCITY ENTRIES x 3"""

    # CASE 1: ggv supplied -> copy it for every waypoint
    if ggv is not None:
        p_ggv = np.repeat(np.expand_dims(ggv, axis=0), kappa.size, axis=0)
        op_mode = 'ggv'

    # CASE 2: local gg diagram supplied -> add velocity dimension (artificial velocity of 10.0 m/s)
    else:
        p_ggv = np.expand_dims(np.column_stack((np.ones(loc_gg.shape[0]) * 10.0, loc_gg)), axis=1)
        op_mode = 'loc_gg'

    # ------------------------------------------------------------------------------------------------------------------
    # SPEED PROFILE CALCULATION (FB) -----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # transform curvature kappa into corresponding radii (abs because curvature has a sign in our convention)
    radii = np.abs(np.divide(1.0, kappa, out=np.full(kappa.size, np.inf), where=kappa != 0.0))

    # call solver
    if not closed:
        vx_profile = __solver_fb_unclosed(p_ggv=p_ggv,
                                          ax_max_machines=ax_max_machines,
                                          v_max=v_max,
                                          radii=radii,
                                          el_lengths=el_lengths,
                                          mu=mu,
                                          v_start=v_start,
                                          v_end=v_end,
                                          dyn_model_exp=dyn_model_exp,
                                          drag_coeff=drag_coeff,
                                          m_veh=m_veh,
                                          op_mode=op_mode)

    else:
        vx_profile = __solver_fb_closed(p_ggv=p_ggv,
                                        ax_max_machines=ax_max_machines,
                                        v_max=v_max,
                                        radii=radii,
                                        el_lengths=el_lengths,
                                        mu=mu,
                                        dyn_model_exp=dyn_model_exp,
                                        drag_coeff=drag_coeff,
                                        m_veh=m_veh,
                                        op_mode=op_mode)

    # ------------------------------------------------------------------------------------------------------------------
    # POSTPROCESSING ---------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if filt_window is not None:
        vx_profile = trajectory_planning_helpers.conv_filt.conv_filt(signal=vx_profile,
                                                                     filt_window=filt_window,
                                                                     closed=closed)

    return vx_profile


def __solver_fb_unclosed(p_ggv: np.ndarray,
                         ax_max_machines: np.ndarray,
                         v_max: float,
                         radii: np.ndarray,
                         el_lengths: np.ndarray,
                         v_start: float,
                         drag_coeff: float,
                         m_veh: float,
                         op_mode: str,
                         mu: np.ndarray = None,
                         v_end: float = None,
                         dyn_model_exp: float = 1.0) -> np.ndarray:

    # ------------------------------------------------------------------------------------------------------------------
    # FORWARD BACKWARD SOLVER ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # handle mu
    if mu is None:
        mu = np.ones(radii.size)
        mu_mean = 1.0
    else:
        mu_mean = np.mean(mu)

    # run through all the points and check for possible lateral acceleration
    if op_mode == 'ggv':
        # in ggv mode all ggvs are equal -> we can use the first one
        ay_max_global = mu_mean * np.amin(p_ggv[0, :, 2])   # get first lateral acceleration estimate
        vx_profile = np.sqrt(ay_max_global * radii)         # get first velocity profile estimate

        ay_max_curr = mu * np.interp(vx_profile, p_ggv[0, :, 0], p_ggv[0, :, 2])
        vx_profile = np.sqrt(np.multiply(ay_max_curr, radii))

    else:
        # in loc_gg mode all ggvs consist of a single line due to the missing velocity dependency, mu is None in this
        # case
        vx_profile = np.sqrt(p_ggv[:, 0, 2] * radii)        # get first velocity profile estimate

    # cut vx_profile to car's top speed
    vx_profile[vx_profile > v_max] = v_max

    # consider v_start
    if vx_profile[0] > v_start:
        vx_profile[0] = v_start

    # calculate acceleration profile
    vx_profile = __solver_fb_acc_profile(p_ggv=p_ggv,
                                         ax_max_machines=ax_max_machines,
                                         v_max=v_max,
                                         radii=radii,
                                         el_lengths=el_lengths,
                                         mu=mu,
                                         vx_profile=vx_profile,
                                         backwards=False,
                                         dyn_model_exp=dyn_model_exp,
                                         drag_coeff=drag_coeff,
                                         m_veh=m_veh)

    # consider v_end
    if v_end is not None and vx_profile[-1] > v_end:
        vx_profile[-1] = v_end

    # calculate deceleration profile
    vx_profile = __solver_fb_acc_profile(p_ggv=p_ggv,
                                         ax_max_machines=ax_max_machines,
                                         v_max=v_max,
                                         radii=radii,
                                         el_lengths=el_lengths,
                                         mu=mu,
                                         vx_profile=vx_profile,
                                         backwards=True,
                                         dyn_model_exp=dyn_model_exp,
                                         drag_coeff=drag_coeff,
                                         m_veh=m_veh)

    return vx_profile


def __solver_fb_closed(p_ggv: np.ndarray,
                       ax_max_machines: np.ndarray,
                       v_max: float,
                       radii: np.ndarray,
                       el_lengths: np.ndarray,
                       drag_coeff: float,
                       m_veh: float,
                       op_mode: str,
                       mu: np.ndarray = None,
                       dyn_model_exp: float = 1.0) -> np.ndarray:

    # ------------------------------------------------------------------------------------------------------------------
    # FORWARD BACKWARD SOLVER ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    no_points = radii.size

    # handle mu
    if mu is None:
        mu = np.ones(no_points)
        mu_mean = 1.0
    else:
        mu_mean = np.mean(mu)

    # run through all the points and check for possible lateral acceleration
    if op_mode == 'ggv':
        # in ggv mode all ggvs are equal -> we can use the first one
        ay_max_global = mu_mean * np.amin(p_ggv[0, :, 2])   # get first lateral acceleration estimate
        vx_profile = np.sqrt(ay_max_global * radii)         # get first velocity estimate (radii must be positive!)

        # iterate until the initial velocity profile converges (break after max. 100 iterations)
        converged = False

        for i in range(100):
            vx_profile_prev_iteration = vx_profile

            ay_max_curr = mu * np.interp(vx_profile, p_ggv[0, :, 0], p_ggv[0, :, 2])
            vx_profile = np.sqrt(np.multiply(ay_max_curr, radii))

            # break the loop if the maximum change of the velocity profile was below 0.5%
            if np.max(np.abs(vx_profile / vx_profile_prev_iteration - 1.0)) < 0.005:
                converged = True
                break

        if not converged:
            print("The initial vx profile did not converge after 100 iterations, please check radii and ggv!")

    else:
        # in loc_gg mode all ggvs consist of a single line due to the missing velocity dependency, mu is None in this
        # case
        vx_profile = np.sqrt(p_ggv[:, 0, 2] * radii)        # get first velocity estimate (radii must be positive!)

    # cut vx_profile to car's top speed
    vx_profile[vx_profile > v_max] = v_max

    """We need to calculate the speed profile for two laps to get the correct starting and ending velocity."""

    # double arrays
    vx_profile_double = np.concatenate((vx_profile, vx_profile), axis=0)
    radii_double = np.concatenate((radii, radii), axis=0)
    el_lengths_double = np.concatenate((el_lengths, el_lengths), axis=0)
    mu_double = np.concatenate((mu, mu), axis=0)
    p_ggv_double = np.concatenate((p_ggv, p_ggv), axis=0)

    # calculate acceleration profile
    vx_profile_double = __solver_fb_acc_profile(p_ggv=p_ggv_double,
                                                ax_max_machines=ax_max_machines,
                                                v_max=v_max,
                                                radii=radii_double,
                                                el_lengths=el_lengths_double,
                                                mu=mu_double,
                                                vx_profile=vx_profile_double,
                                                backwards=False,
                                                dyn_model_exp=dyn_model_exp,
                                                drag_coeff=drag_coeff,
                                                m_veh=m_veh)

    # use second lap of acceleration profile
    vx_profile_double = np.concatenate((vx_profile_double[no_points:], vx_profile_double[no_points:]), axis=0)

    # calculate deceleration profile
    vx_profile_double = __solver_fb_acc_profile(p_ggv=p_ggv_double,
                                                ax_max_machines=ax_max_machines,
                                                v_max=v_max,
                                                radii=radii_double,
                                                el_lengths=el_lengths_double,
                                                mu=mu_double,
                                                vx_profile=vx_profile_double,
                                                backwards=True,
                                                dyn_model_exp=dyn_model_exp,
                                                drag_coeff=drag_coeff,
                                                m_veh=m_veh)

    # use second lap of deceleration profile
    vx_profile = vx_profile_double[no_points:]

    return vx_profile


def __solver_fb_acc_profile(p_ggv: np.ndarray,
                            ax_max_machines: np.ndarray,
                            v_max: float,
                            radii: np.ndarray,
                            el_lengths: np.ndarray,
                            mu: np.ndarray,
                            vx_profile: np.ndarray,
                            drag_coeff: float,
                            m_veh: float,
                            dyn_model_exp: float = 1.0,
                            backwards: bool = False) -> np.ndarray:

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    no_points = vx_profile.size

    # check for reversed direction
    if backwards:
        radii_mod = np.flipud(radii)
        el_lengths_mod = np.flipud(el_lengths)
        mu_mod = np.flipud(mu)
        vx_profile = np.flipud(vx_profile)
        mode = 'decel_backw'
    else:
        radii_mod = radii
        el_lengths_mod = el_lengths
        mu_mod = mu
        mode = 'accel_forw'

    # ------------------------------------------------------------------------------------------------------------------
    # SEARCH START POINTS FOR ACCELERATION PHASES ----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    vx_diffs = np.diff(vx_profile)
    acc_inds = np.where(vx_diffs > 0.0)[0]                  # indices of points with positive acceleration
    if acc_inds.size != 0:
        # check index diffs -> we only need the first point of every acceleration phase
        acc_inds_diffs = np.diff(acc_inds)
        acc_inds_diffs = np.insert(acc_inds_diffs, 0, 2)    # first point is always a starting point
        acc_inds_rel = acc_inds[acc_inds_diffs > 1]         # starting point indices for acceleration phases
    else:
        acc_inds_rel = []                                   # if vmax is low and can be driven all the time

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE VELOCITY PROFILE ---------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # cast np.array as a list
    acc_inds_rel = list(acc_inds_rel)

    # while we have indices remaining in the list
    while acc_inds_rel:
        # set index to first list element
        i = acc_inds_rel.pop(0)

        # start from current index and run until either the end of the lap or a termination criterion are reached
        while i < no_points - 1:

            ax_possible_cur = calc_ax_poss(vx_start=vx_profile[i],
                                           radius=radii_mod[i],
                                           ggv=p_ggv[i],
                                           ax_max_machines=ax_max_machines,
                                           mu=mu_mod[i],
                                           mode=mode,
                                           dyn_model_exp=dyn_model_exp,
                                           drag_coeff=drag_coeff,
                                           m_veh=m_veh)

            vx_possible_next = math.sqrt(math.pow(vx_profile[i], 2) + 2 * ax_possible_cur * el_lengths_mod[i])

            if backwards:
                """
                We have to loop the calculation if we are in the backwards iteration (currently just once). This is 
                because we calculate the possible ax at a point i which does not necessarily fit for point i + 1 
                (which is i - 1 in the real direction). At point i + 1 (or i - 1 in real direction) we have a different 
                start velocity (vx_possible_next), radius and mu value while the absolute value of ax remains the same 
                in both directions.
                """

                # looping just once at the moment
                for j in range(1):
                    ax_possible_next = calc_ax_poss(vx_start=vx_possible_next,
                                                    radius=radii_mod[i + 1],
                                                    ggv=p_ggv[i + 1],
                                                    ax_max_machines=ax_max_machines,
                                                    mu=mu_mod[i + 1],
                                                    mode=mode,
                                                    dyn_model_exp=dyn_model_exp,
                                                    drag_coeff=drag_coeff,
                                                    m_veh=m_veh)

                    vx_tmp = math.sqrt(math.pow(vx_profile[i], 2) + 2 * ax_possible_next * el_lengths_mod[i])

                    if vx_tmp < vx_possible_next:
                        vx_possible_next = vx_tmp
                    else:
                        break

            # save possible next velocity if it is smaller than the current value
            if vx_possible_next < vx_profile[i + 1]:
                vx_profile[i + 1] = vx_possible_next

            i += 1

            # break current acceleration phase if next speed would be higher than the maximum vehicle velocity or if we
            # are at the next acceleration phase start index
            if vx_possible_next > v_max or (acc_inds_rel and i >= acc_inds_rel[0]):
                break

    # ------------------------------------------------------------------------------------------------------------------
    # POSTPROCESSING ---------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # flip output vel_profile if necessary
    if backwards:
        vx_profile = np.flipud(vx_profile)

    return vx_profile

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

def interp_track(reftrack: np.ndarray,
                 stepsize_approx: float = 1.0) -> np.ndarray:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    Use linear interpolation between track points to create new points with equal distances.

    Inputs:
    reftrack:           array containing the track information that shell be interpolated [x, y, w_tr_right, w_tr_left].
    stepsize_approx:    desired stepsize for the interpolation

    Outputs:
    reftrack_interp:    interpolated reference track (unclosed)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # FUNCTION BODY ----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    reftrack_cl = np.vstack((reftrack, reftrack[0]))

    # calculate element lengths (euclidian distance)
    el_lenghts = np.sqrt(np.sum(np.power(np.diff(reftrack_cl[:, :2], axis=0), 2), axis=1))

    # sum up total distance (from start) to every element
    dists_cum = np.cumsum(el_lenghts)
    dists_cum = np.insert(dists_cum, 0, 0.0)

    # calculate desired lenghts depending on specified stepsize (+1 because last element is included)
    no_points_interp = math.ceil(dists_cum[-1] / stepsize_approx) + 1
    dists_interp = np.linspace(0.0, dists_cum[-1], no_points_interp)

    # interpolate closed track points
    reftrack_interp_cl = np.zeros((no_points_interp, 4))
    reftrack_interp_cl[:, 0] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 0])
    reftrack_interp_cl[:, 1] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 1])
    reftrack_interp_cl[:, 2] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 2])
    reftrack_interp_cl[:, 3] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 3])

    # remove closed points
    reftrack_interp = reftrack_interp_cl[:-1]

    return reftrack_interp
def calc_tangent_vectors(psi: np.ndarray) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Use heading to calculate normalized (i.e. unit length) tangent vectors.

    .. inputs::
    :param psi:                     array containing the heading of every point (north up, range [-pi,pi[).
    :type psi:                      np.ndarray

    .. outputs::
    :return tangvec_normalized:     unit length tangent vectors for every point [x, y].
    :rtype tangvec_normalized:      np.ndarray

    .. notes::
    len(psi) = len(tangvec_normalized)
    """

    psi_ = np.copy(psi)

    # remap psi_vel to x-axis
    psi_ += math.pi / 2
    psi_ = normalize_psi(psi_)

    # get normalized tangent vectors
    tangvec_normalized = np.zeros((psi_.size, 2))
    tangvec_normalized[:, 0] = np.cos(psi_)
    tangvec_normalized[:, 1] = np.sin(psi_)

    return tangvec_normalized


def calc_normal_vectors_ahead(psi: np.ndarray) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Use heading to calculate normalized (i.e. unit length) normal vectors. Normal vectors point in direction psi + pi/2.

    .. inputs::
    :param psi:                     array containing the heading of every point (north up, range [-pi,pi[).
    :type psi:                      np.ndarray

    .. outputs::
    :return normvec_normalized:     unit length normal vectors for every point [x, y].
    :rtype normvec_normalized:      np.ndarray

    .. notes::
    len(psi) = len(normvec_normalized)
    """

    # calculate tangent vectors
    tangvec_normalized = calc_tangent_vectors(psi=psi)

    # find normal vectors
    normvec_normalized = np.stack((-tangvec_normalized[:, 1], tangvec_normalized[:, 0]), axis=1)

    return normvec_normalized
def calc_normal_vectors(psi: np.ndarray) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Use heading to calculate normalized (i.e. unit length) normal vectors. Normal vectors point in direction psi - pi/2.

    .. inputs::
    :param psi:                     array containing the heading of every point (north up, range [-pi,pi[).
    :type psi:                      np.ndarray

    .. outputs::
    :return normvec_normalized:     unit length normal vectors for every point [x, y].
    :rtype normvec_normalized:      np.ndarray

    .. notes::
    len(psi) = len(normvec_normalized)
    """

    # calculate normal vectors
    normvec_normalized = -calc_normal_vectors_ahead(psi=psi)

    return normvec_normalized

def dist_to_bounds(
        trajectory: np.ndarray,
        bound_r,
        bound_l,
        centerline: np.ndarray,
        safety_width: float,
        show_plots: bool,
        reverse: bool = False) -> tuple[np.ndarray, np.ndarray]:
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
        trajectory_str = "Global Trajectory"
    else:
        help_trajectory = trajectory
        trajectory_str = "Centerline"

    # interpolate track bounds
    bound_r_tmp = np.column_stack((bound_r, np.zeros((bound_r.shape[0], 2))))
    bound_l_tmp = np.column_stack((bound_l, np.zeros((bound_l.shape[0], 2))))

    bound_r_int = interp_track(reftrack=bound_r_tmp, stepsize_approx=0.1)
    bound_l_int = interp_track(reftrack=bound_l_tmp, stepsize_approx=0.1)

    # find the closest points of the track bounds to global trajectory waypoints
    n_wpnt = len(help_trajectory)
    dists_right = np.zeros(n_wpnt)  # contains (min) distances between waypoints and right bound
    dists_left = np.zeros(n_wpnt)  # contains (min) distances between waypoints and left bound

    for i, wpnt in enumerate(help_trajectory):
        dists_bound_right = np.sqrt(np.power(bound_r_int[:, 0] - wpnt[0], 2)
                                    + np.power(bound_r_int[:, 1] - wpnt[1], 2))
        dists_right[i] = np.amin(dists_bound_right)

        dists_bound_left = np.sqrt(np.power(bound_l_int[:, 0] - wpnt[0], 2)
                                   + np.power(bound_l_int[:, 1] - wpnt[1], 2))
        dists_left[i] = np.amin(dists_bound_left)

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
        return dists_left, dists_right
    else:
        return dists_right, dists_left

def calc_ax_poss(vx_start: float,
                 radius: float,
                 ggv: np.ndarray,
                 mu: float,
                 dyn_model_exp: float,
                 drag_coeff: float,
                 m_veh: float,
                 ax_max_machines: np.ndarray = None,
                 mode: str = 'accel_forw') -> float:
    """
    This function returns the possible longitudinal acceleration in the current step/point.

    .. inputs::
    :param vx_start:        [m/s] velocity at current point
    :type vx_start:         float
    :param radius:          [m] radius on which the car is currently driving
    :type radius:           float
    :param ggv:             ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
    :type ggv:              np.ndarray
    :param mu:              [-] current friction value
    :type mu:               float
    :param dyn_model_exp:   [-] exponent used in the vehicle dynamics model (usual range [1.0,2.0]).
    :type dyn_model_exp:    float
    :param drag_coeff:      [m2*kg/m3] drag coefficient incl. all constants: drag_coeff = 0.5 * c_w * A_front * rho_air
    :type drag_coeff:       float
    :param m_veh:           [kg] vehicle mass
    :type m_veh:            float
    :param ax_max_machines: longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
                            in m/s, accelerations in m/s2. They should be handed in without considering drag resistance.
                            Can be set None if using one of the decel modes.
    :type ax_max_machines:  np.ndarray
    :param mode:            [-] operation mode, can be 'accel_forw', 'decel_forw', 'decel_backw'
                            -> determines if machine limitations are considered and if ax should be considered negative
                            or positive during deceleration (for possible backwards iteration)
    :type mode:             str

    .. outputs::
    :return ax_final:       [m/s2] final acceleration from current point to next one
    :rtype ax_final:        float
    """

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check inputs
    if mode not in ['accel_forw', 'decel_forw', 'decel_backw']:
        raise RuntimeError("Unknown operation mode for calc_ax_poss!")

    if mode == 'accel_forw' and ax_max_machines is None:
        raise RuntimeError("ax_max_machines is required if operation mode is accel_forw!")

    if ggv.ndim != 2 or ggv.shape[1] != 3:
        raise RuntimeError("ggv must have two dimensions and three columns [vx, ax_max, ay_max]!")

    # ------------------------------------------------------------------------------------------------------------------
    # CONSIDER TIRE POTENTIAL ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate possible and used accelerations (considering tires)
    ax_max_tires = mu * np.interp(vx_start, ggv[:, 0], ggv[:, 1])
    ay_max_tires = mu * np.interp(vx_start, ggv[:, 0], ggv[:, 2])
    ay_used = math.pow(vx_start, 2) / radius

    # during forward acceleration and backward deceleration ax_max_tires must be considered positive, during forward
    # deceleration it must be considered negative
    if mode in ['accel_forw', 'decel_backw'] and ax_max_tires < 0.0:
        print("WARNING: Inverting sign of ax_max_tires because it should be positive but was negative!")
        ax_max_tires *= -1.0
    elif mode == 'decel_forw' and ax_max_tires > 0.0:
        print("WARNING: Inverting sign of ax_max_tires because it should be negative but was positve!")
        ax_max_tires *= -1.0

    radicand = 1.0 - math.pow(ay_used / ay_max_tires, dyn_model_exp)

    if radicand > 0.0:
        ax_avail_tires = ax_max_tires * math.pow(radicand, 1.0 / dyn_model_exp)
    else:
        ax_avail_tires = 0.0

    # ------------------------------------------------------------------------------------------------------------------
    # CONSIDER MACHINE LIMITATIONS -------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # consider limitations imposed by electrical machines during forward acceleration
    if mode == 'accel_forw':
        # interpolate machine acceleration to be able to consider varying gear ratios, efficiencies etc.
        ax_max_machines_tmp = np.interp(vx_start, ax_max_machines[:, 0], ax_max_machines[:, 1])
        ax_avail_vehicle = min(ax_avail_tires, ax_max_machines_tmp)
    else:
        ax_avail_vehicle = ax_avail_tires

    # ------------------------------------------------------------------------------------------------------------------
    # CONSIDER DRAG ----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate equivalent longitudinal acceleration of drag force at the current speed
    ax_drag = -math.pow(vx_start, 2) * drag_coeff / m_veh

    # drag reduces the possible acceleration in the forward case and increases it in the backward case
    if mode in ['accel_forw', 'decel_forw']:
        ax_final = ax_avail_vehicle + ax_drag
        # attention: this value will now be negative in forward direction if tire is entirely used for cornering
    else:
        ax_final = ax_avail_vehicle - ax_drag

    return ax_final



def calc_ax_profile(vx_profile: np.ndarray,
                    el_lengths: np.ndarray,
                    eq_length_output: bool = False) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    The function calculates the acceleration profile for a given velocity profile.

    .. inputs::
    :param vx_profile:          array containing the velocity profile used as a basis for the acceleration calculations.
    :type vx_profile:           np.ndarray
    :param el_lengths:          array containing the element lengths between every point of the velocity profile.
    :type el_lengths:           np.ndarray
    :param eq_length_output:    assumes zero acceleration for the last point of the acceleration profile and therefore
                                returns ax_profile with equal length to vx_profile.
    :type eq_length_output:     bool

    .. outputs::
    :return ax_profile:         acceleration profile calculated for the inserted vx_profile.
    :rtype ax_profile:          np.ndarray

    .. notes::
    case eq_length_output is True:
    len(vx_profile) = len(el_lengths) + 1 = len(ax_profile)

    case eq_length_output is False:
    len(vx_profile) = len(el_lengths) + 1 = len(ax_profile) + 1
    """

    # check inputs
    if vx_profile.size != el_lengths.size + 1:
        raise RuntimeError("Array size of vx_profile should be 1 element bigger than el_lengths!")

    # calculate longitudinal acceleration profile array numerically: (v_end^2 - v_beg^2) / 2*s
    if eq_length_output:
        ax_profile = np.zeros(vx_profile.size)
        ax_profile[:-1] = (np.power(vx_profile[1:], 2) - np.power(vx_profile[:-1], 2)) / (2 * el_lengths)
    else:
        ax_profile = (np.power(vx_profile[1:], 2) - np.power(vx_profile[:-1], 2)) / (2 * el_lengths)

    return ax_profile

def calc_t_profile(vx_profile: np.ndarray,
                   el_lengths: np.ndarray,
                   t_start: float = 0.0,
                   ax_profile: np.ndarray = None) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Calculate a temporal duration profile for a given trajectory.

    .. inputs::
    :param vx_profile:  array containing the velocity profile.
    :type vx_profile:   np.ndarray
    :param el_lengths:  array containing the element lengths between every point of the velocity profile.
    :type el_lengths:   np.ndarray
    :param t_start:     start time in seconds added to first array element.
    :type t_start:      float
    :param ax_profile:  acceleration profile fitting to the velocity profile.
    :type ax_profile:   np.ndarray

    .. outputs::
    :return t_profile:  time profile for the given velocity profile.
    :rtype t_profile:   np.ndarray

    .. notes::
    len(el_lengths) + 1 = len(t_profile)

    len(vx_profile) and len(ax_profile) must be >= len(el_lengths) as the temporal duration from one point to the next
    is only calculated based on the previous point.
    """

    # check inputs
    if vx_profile.size < el_lengths.size:
        raise RuntimeError("vx_profile and el_lenghts must have at least the same length!")

    if ax_profile is not None and ax_profile.size < el_lengths.size:
        raise RuntimeError("ax_profile and el_lenghts must have at least the same length!")

    # calculate acceleration profile if required
    if ax_profile is None:
        ax_profile = calc_ax_profile(vx_profile=vx_profile,
                                        el_lengths=el_lengths,
                                        eq_length_output=False)

    # calculate temporal duration of every step between two points
    no_points = el_lengths.size
    t_steps = np.zeros(no_points)

    for i in range(no_points):
        if not math.isclose(ax_profile[i], 0.0):
            t_steps[i] = (-vx_profile[i] + math.sqrt((math.pow(vx_profile[i], 2) + 2 * ax_profile[i] * el_lengths[i])))\
                         / ax_profile[i]

        else:  # ax == 0.0
            t_steps[i] = el_lengths[i] / vx_profile[i]

    # calculate temporal duration profile out of steps
    t_profile = np.insert(np.cumsum(t_steps), 0, 0.0) + t_start

    return t_profile

def calc_splines(path: np.ndarray,
                 el_lengths: np.ndarray = None,
                 psi_s: float = None,
                 psi_e: float = None,
                 use_dist_scaling: bool = True) -> tuple:
    """
    author:
    Tim Stahl & Alexander Heilmeier

    .. description::
    Solve for curvature continuous cubic splines (spline parameter t) between given points i (splines evaluated at
    t = 0 and t = 1). The splines must be set up separately for x- and y-coordinate.

    Spline equations:
    P_{x,y}(t)   =  a_3 * t³ +  a_2 * t² + a_1 * t + a_0
    P_{x,y}'(t)  = 3a_3 * t² + 2a_2 * t  + a_1
    P_{x,y}''(t) = 6a_3 * t  + 2a_2

    a * {x; y} = {b_x; b_y}

    .. inputs::
    :param path:                x and y coordinates as the basis for the spline construction (closed or unclosed). If
                                path is provided unclosed, headings psi_s and psi_e are required!
    :type path:                 np.ndarray
    :param el_lengths:          distances between path points (closed or unclosed). The input is optional. The distances
                                are required for the scaling of heading and curvature values. They are calculated using
                                euclidian distances if required but not supplied.
    :type el_lengths:           np.ndarray
    :param psi_s:               orientation of the {start, end} point.
    :type psi_s:                float
    :param psi_e:               orientation of the {start, end} point.
    :type psi_e:                float
    :param use_dist_scaling:    bool flag to indicate if heading and curvature scaling should be performed. This should
                                be done if the distances between the points in the path are not equal.
    :type use_dist_scaling:     bool

    .. outputs::
    :return x_coeff:            spline coefficients of the x-component.
    :rtype x_coeff:             np.ndarray
    :return y_coeff:            spline coefficients of the y-component.
    :rtype y_coeff:             np.ndarray
    :return M:                  LES coefficients.
    :rtype M:                   np.ndarray
    :return normvec_normalized: normalized normal vectors [x, y].
    :rtype normvec_normalized:  np.ndarray

    .. notes::
    Outputs are always unclosed!

    path and el_lengths inputs can either be closed or unclosed, but must be consistent! The function detects
    automatically if the path was inserted closed.

    Coefficient matrices have the form a_0i, a_1i * t, a_2i * t^2, a_3i * t^3.
    """

    # check if path is closed
    if np.all(np.isclose(path[0], path[-1])) and psi_s is None:
        closed = True
    else:
        closed = False

    # check inputs
    if not closed and (psi_s is None or psi_e is None):
        raise RuntimeError("Headings must be provided for unclosed spline calculation!")

    if el_lengths is not None and path.shape[0] != el_lengths.size + 1:
        raise RuntimeError("el_lengths input must be one element smaller than path input!")

    # if distances between path coordinates are not provided but required, calculate euclidean distances as el_lengths
    if use_dist_scaling and el_lengths is None:
        el_lengths = np.sqrt(np.sum(np.power(np.diff(path, axis=0), 2), axis=1))
    elif el_lengths is not None:
        el_lengths = np.copy(el_lengths)

    # if closed and use_dist_scaling active append element length in order to obtain overlapping elements for proper
    # scaling of the last element afterwards
    if use_dist_scaling and closed:
        el_lengths = np.append(el_lengths, el_lengths[0])

    # get number of splines
    no_splines = path.shape[0] - 1

    # calculate scaling factors between every pair of splines
    if use_dist_scaling:
        scaling = el_lengths[:-1] / el_lengths[1:]
    else:
        scaling = np.ones(no_splines - 1)

    # ------------------------------------------------------------------------------------------------------------------
    # DEFINE LINEAR EQUATION SYSTEM ------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # M_{x,y} * a_{x,y} = b_{x,y}) with a_{x,y} being the desired spline param
    # *4 because of 4 parameters in cubic spline
    M = np.zeros((no_splines * 4, no_splines * 4))
    b_x = np.zeros((no_splines * 4, 1))
    b_y = np.zeros((no_splines * 4, 1))

    # create template for M array entries
    # row 1: beginning of current spline should be placed on current point (t = 0)
    # row 2: end of current spline should be placed on next point (t = 1)
    # row 3: heading at end of current spline should be equal to heading at beginning of next spline (t = 1 and t = 0)
    # row 4: curvature at end of current spline should be equal to curvature at beginning of next spline (t = 1 and
    #        t = 0)
    template_M = np.array(                          # current point               | next point              | bounds
                [[1,  0,  0,  0,  0,  0,  0,  0],   # a_0i                                                  = {x,y}_i
                 [1,  1,  1,  1,  0,  0,  0,  0],   # a_0i + a_1i +  a_2i +  a_3i                           = {x,y}_i+1
                 [0,  1,  2,  3,  0, -1,  0,  0],   # _      a_1i + 2a_2i + 3a_3i      - a_1i+1             = 0
                 [0,  0,  2,  6,  0,  0, -2,  0]])  # _             2a_2i + 6a_3i               - 2a_2i+1   = 0

    for i in range(no_splines):
        j = i * 4

        if i < no_splines - 1:
            M[j: j + 4, j: j + 8] = template_M

            M[j + 2, j + 5] *= scaling[i]
            M[j + 3, j + 6] *= math.pow(scaling[i], 2)

        else:
            # no curvature and heading bounds on last element (handled afterwards)
            M[j: j + 2, j: j + 4] = [[1,  0,  0,  0],
                                     [1,  1,  1,  1]]

        b_x[j: j + 2] = [[path[i,     0]],
                         [path[i + 1, 0]]]
        b_y[j: j + 2] = [[path[i,     1]],
                         [path[i + 1, 1]]]

    # ------------------------------------------------------------------------------------------------------------------
    # SET BOUNDARY CONDITIONS FOR LAST AND FIRST POINT -----------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if not closed:
        # if the path is unclosed we want to fix heading at the start and end point of the path (curvature cannot be
        # determined in this case) -> set heading boundary conditions

        # heading start point
        M[-2, 1] = 1  # heading start point (evaluated at t = 0)

        if el_lengths is None:
            el_length_s = 1.0
        else:
            el_length_s = el_lengths[0]

        b_x[-2] = math.cos(psi_s + math.pi / 2) * el_length_s
        b_y[-2] = math.sin(psi_s + math.pi / 2) * el_length_s

        # heading end point
        M[-1, -4:] = [0, 1, 2, 3]  # heading end point (evaluated at t = 1)

        if el_lengths is None:
            el_length_e = 1.0
        else:
            el_length_e = el_lengths[-1]

        b_x[-1] = math.cos(psi_e + math.pi / 2) * el_length_e
        b_y[-1] = math.sin(psi_e + math.pi / 2) * el_length_e

    else:
        # heading boundary condition (for a closed spline)
        M[-2, 1] = scaling[-1]
        M[-2, -3:] = [-1, -2, -3]
        # b_x[-2] = 0
        # b_y[-2] = 0

        # curvature boundary condition (for a closed spline)
        M[-1, 2] = 2 * math.pow(scaling[-1], 2)
        M[-1, -2:] = [-2, -6]
        # b_x[-1] = 0
        # b_y[-1] = 0

    # ------------------------------------------------------------------------------------------------------------------
    # SOLVE ------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    x_les = np.squeeze(np.linalg.solve(M, b_x))  # squeeze removes single-dimensional entries
    y_les = np.squeeze(np.linalg.solve(M, b_y))

    # get coefficients of every piece into one row -> reshape
    coeffs_x = np.reshape(x_les, (no_splines, 4))
    coeffs_y = np.reshape(y_les, (no_splines, 4))

    # get normal vector (behind used here instead of ahead for consistency with other functions) (second coefficient of
    # cubic splines is relevant for the heading)
    normvec = np.stack((coeffs_y[:, 1], -coeffs_x[:, 1]), axis=1)

    # normalize normal vectors
    norm_factors = 1.0 / np.sqrt(np.sum(np.power(normvec, 2), axis=1))
    normvec_normalized = np.expand_dims(norm_factors, axis=1) * normvec

    return coeffs_x, coeffs_y, M, normvec_normalized

def calc_spline_lengths(coeffs_x: np.ndarray,
                        coeffs_y: np.ndarray,
                        quickndirty: bool = False,
                        no_interp_points: int = 15) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Calculate spline lengths for third order splines defining x- and y-coordinates by usage of intermediate steps.

    .. inputs::
    :param coeffs_x:            coefficient matrix of the x splines with size (no_splines x 4).
    :type coeffs_x:             np.ndarray
    :param coeffs_y:            coefficient matrix of the y splines with size (no_splines x 4).
    :type coeffs_y:             np.ndarray
    :param quickndirty:         True returns lengths based on distance between first and last spline point instead of
                                using interpolation.
    :type quickndirty:          bool
    :param no_interp_points:    length calculation is carried out with the given number of interpolation steps.
    :type no_interp_points:     int

    .. outputs::
    :return spline_lengths:     length of every spline segment.
    :rtype spline_lengths:      np.ndarray

    .. notes::
    len(coeffs_x) = len(coeffs_y) = len(spline_lengths)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check inputs
    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise RuntimeError("Coefficient matrices must have the same length!")

    # catch case with only one spline
    if coeffs_x.size == 4 and coeffs_x.shape[0] == 4:
        coeffs_x = np.expand_dims(coeffs_x, 0)
        coeffs_y = np.expand_dims(coeffs_y, 0)

    # get number of splines and create output array
    no_splines = coeffs_x.shape[0]
    spline_lengths = np.zeros(no_splines)

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE LENGHTS ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if quickndirty:
        for i in range(no_splines):
            spline_lengths[i] = math.sqrt(math.pow(np.sum(coeffs_x[i]) - coeffs_x[i, 0], 2)
                                          + math.pow(np.sum(coeffs_y[i]) - coeffs_y[i, 0], 2))

    else:
        # loop through all the splines and calculate intermediate coordinates
        t_steps = np.linspace(0.0, 1.0, no_interp_points)
        spl_coords = np.zeros((no_interp_points, 2))

        for i in range(no_splines):
            spl_coords[:, 0] = coeffs_x[i, 0] \
                               + coeffs_x[i, 1] * t_steps \
                               + coeffs_x[i, 2] * np.power(t_steps, 2) \
                               + coeffs_x[i, 3] * np.power(t_steps, 3)
            spl_coords[:, 1] = coeffs_y[i, 0] \
                               + coeffs_y[i, 1] * t_steps \
                               + coeffs_y[i, 2] * np.power(t_steps, 2) \
                               + coeffs_y[i, 3] * np.power(t_steps, 3)

            spline_lengths[i] = np.sum(np.sqrt(np.sum(np.power(np.diff(spl_coords, axis=0), 2), axis=1)))

    return spline_lengths

def interp_splines(coeffs_x: np.ndarray,
                   coeffs_y: np.ndarray,
                   spline_lengths: np.ndarray = None,
                   incl_last_point: bool = False,
                   stepsize_approx: float = None,
                   stepnum_fixed: list = None) -> tuple:
    """
    author:
    Alexander Heilmeier & Tim Stahl

    .. description::
    Interpolate points on one or more splines with third order. The last point (i.e. t = 1.0)
    can be included if option is set accordingly (should be prevented for a closed raceline in most cases). The
    algorithm keeps stepsize_approx as good as possible.

    .. inputs::
    :param coeffs_x:        coefficient matrix of the x splines with size (no_splines x 4).
    :type coeffs_x:         np.ndarray
    :param coeffs_y:        coefficient matrix of the y splines with size (no_splines x 4).
    :type coeffs_y:         np.ndarray
    :param spline_lengths:  array containing the lengths of the inserted splines with size (no_splines x 1).
    :type spline_lengths:   np.ndarray
    :param incl_last_point: flag to set if last point should be kept or removed before return.
    :type incl_last_point:  bool
    :param stepsize_approx: desired stepsize of the points after interpolation.                      \\ Provide only one
    :type stepsize_approx:  float
    :param stepnum_fixed:   return a fixed number of coordinates per spline, list of length no_splines. \\ of these two!
    :type stepnum_fixed:    list

    .. outputs::
    :return path_interp:    interpolated path points.
    :rtype path_interp:     np.ndarray
    :return spline_inds:    contains the indices of the splines that hold the interpolated points.
    :rtype spline_inds:     np.ndarray
    :return t_values:       containts the relative spline coordinate values (t) of every point on the splines.
    :rtype t_values:        np.ndarray
    :return dists_interp:   total distance up to every interpolation point.
    :rtype dists_interp:    np.ndarray

    .. notes::
    len(coeffs_x) = len(coeffs_y) = len(spline_lengths)

    len(path_interp = len(spline_inds) = len(t_values) = len(dists_interp)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # INPUT CHECKS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check sizes
    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise RuntimeError("Coefficient matrices must have the same length!")

    if spline_lengths is not None and coeffs_x.shape[0] != spline_lengths.size:
        raise RuntimeError("coeffs_x/y and spline_lengths must have the same length!")

    # check if coeffs_x and coeffs_y have exactly two dimensions and raise error otherwise
    if not (coeffs_x.ndim == 2 and coeffs_y.ndim == 2):
        raise RuntimeError("Coefficient matrices do not have two dimensions!")

    # check if step size specification is valid
    if (stepsize_approx is None and stepnum_fixed is None) \
            or (stepsize_approx is not None and stepnum_fixed is not None):
        raise RuntimeError("Provide one of 'stepsize_approx' and 'stepnum_fixed' and set the other to 'None'!")

    if stepnum_fixed is not None and len(stepnum_fixed) != coeffs_x.shape[0]:
        raise RuntimeError("The provided list 'stepnum_fixed' must hold an entry for every spline!")

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE NUMBER OF INTERPOLATION POINTS AND ACCORDING DISTANCES -------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if stepsize_approx is not None:
        # get the total distance up to the end of every spline (i.e. cumulated distances)
        if spline_lengths is None:
            spline_lengths = calc_spline_lengths(coeffs_x=coeffs_x,
                                                    coeffs_y=coeffs_y,
                                                    quickndirty=False)

        dists_cum = np.cumsum(spline_lengths)

        # calculate number of interpolation points and distances (+1 because last point is included at first)
        no_interp_points = math.ceil(dists_cum[-1] / stepsize_approx) + 1
        dists_interp = np.linspace(0.0, dists_cum[-1], no_interp_points)

    else:
        # get total number of points to be sampled (subtract overlapping points)
        no_interp_points = sum(stepnum_fixed) - (len(stepnum_fixed) - 1)
        dists_interp = None

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE INTERMEDIATE STEPS -------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # create arrays to save the values
    path_interp = np.zeros((no_interp_points, 2))           # raceline coords (x, y) array
    spline_inds = np.zeros(no_interp_points, dtype=int)  # save the spline index to which a point belongs
    t_values = np.zeros(no_interp_points)                   # save t values

    if stepsize_approx is not None:

        # --------------------------------------------------------------------------------------------------------------
        # APPROX. EQUAL STEP SIZE ALONG PATH OF ADJACENT SPLINES -------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # loop through all the elements and create steps with stepsize_approx
        for i in range(no_interp_points - 1):
            # find the spline that hosts the current interpolation point
            j = np.argmax(dists_interp[i] < dists_cum)
            spline_inds[i] = j

            # get spline t value depending on the progress within the current element
            if j > 0:
                t_values[i] = (dists_interp[i] - dists_cum[j - 1]) / spline_lengths[j]
            else:
                if spline_lengths.ndim == 0:
                    t_values[i] = dists_interp[i] / spline_lengths
                else:
                    t_values[i] = dists_interp[i] / spline_lengths[0]

            # calculate coords
            path_interp[i, 0] = coeffs_x[j, 0] \
                                + coeffs_x[j, 1] * t_values[i]\
                                + coeffs_x[j, 2] * math.pow(t_values[i], 2) \
                                + coeffs_x[j, 3] * math.pow(t_values[i], 3)

            path_interp[i, 1] = coeffs_y[j, 0]\
                                + coeffs_y[j, 1] * t_values[i]\
                                + coeffs_y[j, 2] * math.pow(t_values[i], 2) \
                                + coeffs_y[j, 3] * math.pow(t_values[i], 3)

    else:

        # --------------------------------------------------------------------------------------------------------------
        # FIXED STEP SIZE FOR EVERY SPLINE SEGMENT ---------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        j = 0

        for i in range(len(stepnum_fixed)):
            # skip last point except for last segment
            if i < len(stepnum_fixed) - 1:
                t_values[j:(j + stepnum_fixed[i] - 1)] = np.linspace(0, 1, stepnum_fixed[i])[:-1]
                spline_inds[j:(j + stepnum_fixed[i] - 1)] = i
                j += stepnum_fixed[i] - 1

            else:
                t_values[j:(j + stepnum_fixed[i])] = np.linspace(0, 1, stepnum_fixed[i])
                spline_inds[j:(j + stepnum_fixed[i])] = i
                j += stepnum_fixed[i]

        t_set = np.column_stack((np.ones(no_interp_points), t_values, np.power(t_values, 2), np.power(t_values, 3)))

        # remove overlapping samples
        n_samples = np.array(stepnum_fixed)
        n_samples[:-1] -= 1

        path_interp[:, 0] = np.sum(np.multiply(np.repeat(coeffs_x, n_samples, axis=0), t_set), axis=1)
        path_interp[:, 1] = np.sum(np.multiply(np.repeat(coeffs_y, n_samples, axis=0), t_set), axis=1)

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE LAST POINT IF REQUIRED (t = 1.0) -----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if incl_last_point:
        path_interp[-1, 0] = np.sum(coeffs_x[-1])
        path_interp[-1, 1] = np.sum(coeffs_y[-1])
        spline_inds[-1] = coeffs_x.shape[0] - 1
        t_values[-1] = 1.0

    else:
        path_interp = path_interp[:-1]
        spline_inds = spline_inds[:-1]
        t_values = t_values[:-1]

        if dists_interp is not None:
            dists_interp = dists_interp[:-1]

    # NOTE: dists_interp is None, when using a fixed step size
    return path_interp, spline_inds, t_values, dists_interp




def create_raceline(refline: np.ndarray,
                    normvectors: np.ndarray,
                    alpha: np.ndarray,
                    stepsize_interp: float) -> tuple:
    """
    author:
    Alexander Heilmeier

    .. description::
    This function includes the algorithm part connected to the interpolation of the raceline after the optimization.

    .. inputs::
    :param refline:         array containing the track reference line [x, y] (unit is meter, must be unclosed!)
    :type refline:          np.ndarray
    :param normvectors:     normalized normal vectors for every point of the reference line [x_component, y_component]
                            (unit is meter, must be unclosed!)
    :type normvectors:      np.ndarray
    :param alpha:           solution vector of the optimization problem containing the lateral shift in m for every point.
    :type alpha:            np.ndarray
    :param stepsize_interp: stepsize in meters which is used for the interpolation after the raceline creation.
    :type stepsize_interp:  float

    .. outputs::
    :return raceline_interp:                interpolated raceline [x, y] in m.
    :rtype raceline_interp:                 np.ndarray
    :return A_raceline:                     linear equation system matrix of the splines on the raceline.
    :rtype A_raceline:                      np.ndarray
    :return coeffs_x_raceline:              spline coefficients of the x-component.
    :rtype coeffs_x_raceline:               np.ndarray
    :return coeffs_y_raceline:              spline coefficients of the y-component.
    :rtype coeffs_y_raceline:               np.ndarray
    :return spline_inds_raceline_interp:    contains the indices of the splines that hold the interpolated points.
    :rtype spline_inds_raceline_interp:     np.ndarray
    :return t_values_raceline_interp:       containts the relative spline coordinate values (t) of every point on the
                                            splines.
    :rtype t_values_raceline_interp:        np.ndarray
    :return s_raceline_interp:              total distance in m (i.e. s coordinate) up to every interpolation point.
    :rtype s_raceline_interp:               np.ndarray
    :return spline_lengths_raceline:        lengths of the splines on the raceline in m.
    :rtype spline_lengths_raceline:         np.ndarray
    :return el_lengths_raceline_interp_cl:  distance between every two points on interpolated raceline in m (closed!).
    :rtype el_lengths_raceline_interp_cl:   np.ndarray
    """

    # calculate raceline on the basis of the optimized alpha values
    raceline = refline + np.expand_dims(alpha, 1) * normvectors

    # calculate new splines on the basis of the raceline
    raceline_cl = np.vstack((raceline, raceline[0]))

    coeffs_x_raceline, coeffs_y_raceline, A_raceline, normvectors_raceline = calc_splines(path=raceline_cl,
                                                                                            use_dist_scaling=False)

    # calculate new spline lengths
    spline_lengths_raceline = calc_spline_lengths(coeffs_x=coeffs_x_raceline,
                                                    coeffs_y=coeffs_y_raceline)

    # interpolate splines for evenly spaced raceline points
    raceline_interp, spline_inds_raceline_interp, t_values_raceline_interp, s_raceline_interp = interp_splines(spline_lengths=spline_lengths_raceline,
                                                                                                                coeffs_x=coeffs_x_raceline,
                                                                                                                coeffs_y=coeffs_y_raceline,
                                                                                                                incl_last_point=False,
                                                                                                                stepsize_approx=stepsize_interp)

    # calculate element lengths
    s_tot_raceline = float(np.sum(spline_lengths_raceline))
    el_lengths_raceline_interp = np.diff(s_raceline_interp)
    el_lengths_raceline_interp_cl = np.append(el_lengths_raceline_interp, s_tot_raceline - s_raceline_interp[-1])

    return raceline_interp, A_raceline, coeffs_x_raceline, coeffs_y_raceline, spline_inds_raceline_interp, \
           t_values_raceline_interp, s_raceline_interp, spline_lengths_raceline, el_lengths_raceline_interp_cl

def create_raceline_wout_interp(refline: np.ndarray) -> tuple:
    """
    author:
    Alexander Heilmeier

    .. description::
    Computes the raceline by applying the lateral shifts (alpha) to the reference line and
    returns the exact same raceline without any re-sampling or interpolation. Auxiliary outputs
    (such as spline coefficients, cumulative distances, etc.) are computed based on the original
    raceline points.

    .. inputs::
    :param refline:         array containing the track reference line [x, y] in meters (must be unclosed).
    :type refline:          np.ndarray
    :param normvectors:     normalized normal vectors for every point of the reference line [x_component, y_component]
                            (must be unclosed).
    :type normvectors:      np.ndarray
    :param alpha:           solution vector of the optimization problem containing the lateral shift in meters for every point.
    :type alpha:            np.ndarray
    :param stepsize_interp: desired interpolation stepsize in meters (ignored in this version).
    :type stepsize_interp:  float

    .. outputs::
    :return raceline_interp:                the raceline, i.e. the shifted refline [x, y] in meters (unchanged).
    :rtype raceline_interp:                 np.ndarray
    :return A_raceline:                     linear equation system matrix from the spline computation.
    :rtype A_raceline:                      np.ndarray
    :return coeffs_x_raceline:              spline coefficients for the x-component.
    :rtype coeffs_x_raceline:               np.ndarray
    :return coeffs_y_raceline:              spline coefficients for the y-component.
    :rtype coeffs_y_raceline:               np.ndarray
    :return spline_inds_raceline_interp:    indices corresponding to each original raceline point.
    :rtype spline_inds_raceline_interp:     np.ndarray
    :return t_values_raceline_interp:       parameter values (t) for each raceline point, linearly spaced.
    :rtype t_values_raceline_interp:        np.ndarray
    :return s_raceline_interp:              cumulative distances along the raceline.
    :rtype s_raceline_interp:               np.ndarray
    :return spline_lengths_raceline:        lengths of the computed splines on the raceline.
    :rtype spline_lengths_raceline:         np.ndarray
    :return el_lengths_raceline_interp_cl:  element lengths between consecutive raceline points with the closing segment appended.
    :rtype el_lengths_raceline_interp_cl:   np.ndarray

    .. notes::
    The provided refline is not re-interpolated. The shifted raceline is returned exactly as computed.
    """

    # Compute the raceline using the optimized lateral shifts
    raceline = refline 

    # Compute spline coefficients and lengths using a closed version of the raceline
    #closing_length = np.linalg.norm(raceline[-1]-raceline[0])
    #if closing_length < 5e-2:
    #    raceline_cl = np.vstack((raceline[:-1], raceline[0]))
    #else:
    raceline_cl = np.vstack((raceline, raceline[0]))  # close the curve for spline computation

    coeffs_x_raceline, coeffs_y_raceline, A_raceline, _ = calc_splines(
        path=raceline_cl,
        use_dist_scaling=False
    )
    spline_lengths_raceline = calc_spline_lengths(
        coeffs_x=coeffs_x_raceline,
        coeffs_y=coeffs_y_raceline
    )

    # Bypass interpolation: return the exact same raceline points
    raceline_interp = refline.copy()

    # Compute cumulative distances along the raceline
    deltas = np.diff(raceline, axis=0)
    seg_lengths = np.sqrt(np.sum(deltas**2, axis=1))
    s_raceline_interp = np.concatenate(([0.0], np.cumsum(seg_lengths)))

    # Assign original indices and a linear parameterization for t-values
    n_points = raceline.shape[0]
    spline_inds_raceline_interp = np.arange(n_points)
    t_values_raceline_interp = np.linspace(0.0, 1.0, n_points)

    # Compute element lengths between consecutive raceline points and append the closing segment length
    closing_length = np.linalg.norm(raceline[0] - raceline[-1])
    el_lengths_raceline_interp_cl = np.concatenate((seg_lengths, [closing_length]))

    return (raceline_interp, A_raceline, coeffs_x_raceline, coeffs_y_raceline,
            spline_inds_raceline_interp, t_values_raceline_interp, s_raceline_interp,
            spline_lengths_raceline, el_lengths_raceline_interp_cl)

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


def write_waypoints(
        map_info: str,
        est_lap_time: float,
        centerline: np.ndarray,
        global_trajectory: np.ndarray,
        d_right: np.ndarray,
        d_left: np.ndarray,
        bounds_right: np.ndarray,
        bounds_left: np.ndarray,
        output_file: str = None,
        return_msg: bool = False,
        return_save_dict: bool = False,
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
    if output_file is not None:
        with open(output_file, 'w') as f:
            json.dump(save_dict, f, indent=4)
    if return_save_dict:
        return save_dict
    if return_msg:
        return global_wpnts