'''
Shared functions to read and write map information (global waypoints)
'''
import os
import json
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from visualization_msgs.msg import MarkerArray
from f110_msgs.msg import WpntArray
from std_msgs.msg import String, Float32
from typing import Tuple, List, Dict


def write_global_waypoints(map_dir: str,
                           map_info_str: str,
                           est_lap_time: Float32,
                           centerline_markers: MarkerArray,
                           centerline_waypoints: WpntArray,
                           global_traj_markers_iqp: MarkerArray,
                           global_traj_wpnts_iqp: WpntArray,
                           global_traj_markers_sp: MarkerArray,
                           global_traj_wpnts_sp: WpntArray,
                           trackbounds_markers: MarkerArray
                           ) -> None:
    '''
    Writes map information to a JSON file in the specified `map_dir` directory.
    - map_info_str
        - from topic /map_infos: python str
    - est_lap_time
        - from topic /estimated_lap_time: Float32
    - centerline_markers
        - from topic /centerline_waypoints/markers: MarkerArray
    - centerline_waypoints
        - from topic /centerline_waypoints: WpntArray
    - global_traj_markers_iqp
        - from topic /global_waypoints: MarkerArray
    - global_traj_wpnts_iqp
        - from topic /global_waypoints/markers: WpntArray
    - global_traj_markers_sp
        - from topic /global_waypoints/shortest_path: MarkerArray
    - global_traj_wpnts_sp
        - from topic /global_waypoitns/shortest_path/markers: WpntArray
    - trackbounds_markers
        - from topic /trackbounds/markers: MarkerArray
    - write_path: Full path of JSON file to write
    '''

    path = os.path.join(map_dir, 'global_waypoints.json')
    print(f"[INFO] WRITE_GLOBAL_WAYPOINTS: Writing global waypoints to {path}")

    # Dictionary will be converted into a JSON for serialization
    d: Dict[str, Dict] = {}
    d['map_info_str'] = {'data': map_info_str}
    d['est_lap_time'] = {'data': est_lap_time.data}
    d['centerline_markers'] = message_to_ordereddict(centerline_markers)
    d['centerline_waypoints'] = message_to_ordereddict(centerline_waypoints)
    d['global_traj_markers_iqp'] = message_to_ordereddict(global_traj_markers_iqp)
    d['global_traj_wpnts_iqp'] = message_to_ordereddict(global_traj_wpnts_iqp)
    d['global_traj_markers_sp'] = message_to_ordereddict(global_traj_markers_sp)
    d['global_traj_wpnts_sp'] = message_to_ordereddict(global_traj_wpnts_sp)
    d['trackbounds_markers'] = message_to_ordereddict(trackbounds_markers)

    # serialize
    with open(path, 'w') as f:
        json.dump(d, f)

def read_global_waypoints(file: str) -> Tuple[String, Float32, MarkerArray, WpntArray, MarkerArray, WpntArray, MarkerArray]:
    print(f"[INFO] READ_GLOBAL_WAYPOINTS: Reading global waypoints from {file}")
    # Deserialize JSON and Reconstruct the maps elements
    with open(file, 'r') as f:
        wp_dict: Dict[str, List] = json.load(f)

    map_info_str = String()
    set_message_fields(map_info_str, wp_dict['map_info_str'])

    est_lap_time = Float32()
    set_message_fields(est_lap_time, wp_dict['est_lap_time'])

    centerline_markers = MarkerArray()
    set_message_fields(centerline_markers, wp_dict['centerline_markers'])

    centerline_waypoints = WpntArray()
    set_message_fields(centerline_waypoints, wp_dict['centerline_waypoints'])

    global_traj_markers = MarkerArray()
    set_message_fields(global_traj_markers, wp_dict['global_traj_markers'])

    global_traj_wpnts = WpntArray()
    set_message_fields(global_traj_wpnts, wp_dict['global_traj_wpnts'])

    trackbounds_markers = MarkerArray()
    set_message_fields(trackbounds_markers, wp_dict['trackbounds_markers'])

    return  map_info_str, est_lap_time, \
            centerline_markers, centerline_waypoints, \
            global_traj_markers, global_traj_wpnts, \
            trackbounds_markers