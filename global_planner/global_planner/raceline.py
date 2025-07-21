import numpy as np
import matplotlib.pyplot as plt

from global_planner.utils import *

from global_planner.trajectory_planning_helpers import (
    calc_ax_profile,
    calc_head_curv_num
)

class RacelineContainer:

    S_INDEX = 0
    X_INDEX = 1
    Y_INDEX = 2
    D_LEFT_INDEX = 3
    D_RIGHT_INDEX = 4
    PSI_INDEX = 5
    KAPPA_INDEX = 6
    VX_INDEX = 7
    AX_INDEX = 8
    DX_INDEX = 9
    DY_INDEX = 10
    DDX_INDEX = 11
    DDY_INDEX = 12

    def __init__(self, raceline: np.ndarray):
        self.raceline = raceline
        self.left_normals = None
        self.right_normals = None
        self.left_trackbound = None
        self.right_trackbound = None
        self.sectors = None
        
        self.compute_trackbounds()

    def __call__(self, inplace: bool = False):
        if inplace:
            return self.raceline
        return np.copy(self.raceline)
    
    def inverse_bounds(self):
        ltb = np.copy(self.raceline[:, self.D_LEFT_INDEX])
        rtb = np.copy(self.raceline[:, self.D_RIGHT_INDEX])
        self.raceline[:, self.D_LEFT_INDEX] = rtb
        self.raceline[:, self.D_RIGHT_INDEX] = ltb
    
    def compute_trackbounds(self): 

        self.left_trackbound = []
        self.right_trackbound = []
        self.compute_normals()

        for i, (pnt, left_normal, right_normal) in enumerate(zip(self.raceline, self.left_normals, self.right_normals)):
            # Get the current point
            x = pnt[self.X_INDEX]
            y = pnt[self.Y_INDEX]
            d_left = pnt[self.D_LEFT_INDEX]
            d_right = pnt[self.D_RIGHT_INDEX]

            left_boundary = np.array([x, y]) + d_left * left_normal
            right_boundary = np.array([x, y]) + d_right * right_normal

            # Append to the lists
            self.left_trackbound.append(left_boundary)
            self.right_trackbound.append(right_boundary)

        self.left_trackbound = np.stack(self.left_trackbound)
        self.right_trackbound = np.stack(self.right_trackbound) 

    def compute_inflated_trackbounds(self, inflation_factor: float = 1.0, inflation_add: float = 0.0, inverse_bounds: bool = False):
        left_trackbound = []
        right_trackbound = []
        self.compute_normals()

        for i, (pnt, left_normal, right_normal) in enumerate(zip(self.raceline, self.left_normals, self.right_normals)):
            # Get the current point
            x = pnt[self.X_INDEX]
            y = pnt[self.Y_INDEX]

            d_left =  (pnt[self.D_LEFT_INDEX] + inflation_add) * inflation_factor
            d_right = (pnt[self.D_RIGHT_INDEX] + inflation_add) * inflation_factor
            if inverse_bounds:
                d_left, d_right = d_right, d_left

            left_boundary = np.array([x, y]) + d_left * left_normal
            right_boundary = np.array([x, y]) + d_right * right_normal

            # Append to the lists
            left_trackbound.append(left_boundary)
            right_trackbound.append(right_boundary)

        left_trackbound = np.stack(left_trackbound)
        right_trackbound = np.stack(right_trackbound)

        return left_trackbound, right_trackbound   

    def compute_normals(self):
        self.left_normals = []
        self.right_normals = []

        for i, pnt in enumerate(self.raceline):
            # Get the current point
            psi = pnt[self.PSI_INDEX]

            left_normal = np.array([np.cos(psi), np.sin(psi)])
            right_normal = -left_normal  # This gives: (sin(psi), -cos(psi))
            # Append to the lists
            self.left_normals.append(left_normal)
            self.right_normals.append(right_normal)

        self.left_normals = np.stack(self.left_normals)
        self.right_normals = np.stack(self.right_normals) 

    def get_bound_distances(self, points: np.ndarray):

        dist_left = np.linalg.norm(points[:, None, :] - self.left_trackbound[None, :, :], axis=2)
        dist_right = np.linalg.norm(points[:, None, :] - self.right_trackbound[None, :, :], axis=2)

        min_left = np.min(dist_left, axis=1)
        min_right = np.min(dist_right, axis=1)

        return min_left, min_right


    def get_index_at_s(self, s: float):
        """
        Get the index of the closest point on the raceline to a given arc-length s.
        :param s: Arc-length in meters
        :return: Index of the closest point on the raceline
        """
        # Find the index of the closest point on the raceline
        idx = np.argmin(np.abs(self.raceline[:, self.S_INDEX] - s))
        return idx

    def get_point_at_s(self, s: float):
        """
        Get the point on the raceline at a given arc-length s.
        :param s: Arc-length in meters
        :return: Point on the raceline (x, y)
        """
        # Find the index of the closest point on the raceline
        idx = np.argmin(np.abs(self.raceline[:, self.S_INDEX] - s))
        return self.raceline[idx, self.X_INDEX:self.Y_INDEX+1]

    def set_velocity(self, velocity:np.ndarray, start=0, end=-1, indices =None):
        end = end if end>=0 else len(self) + end + 1
        #import ipdb; ipdb.set_trace()
        if indices is None:
            if start<=end:
                #indices = np.linspace(start, end, velocity.shape[0], dtype=int)
                indices = np.arange(start, min(self.raceline.shape[0], end+1))
            else:
                indices_start = np.arange(start, len(self))
                indices_end = np.arange(0, end+1)
                indices = np.concatenate((indices_end, indices_start), axis=0)
        self.raceline[indices, self.VX_INDEX] = velocity
        # Recompute the acceleration profile
        velocity = np.concatenate((self.raceline[:,self.VX_INDEX], self.raceline[0,self.VX_INDEX][None]), axis=0)
        ax_profile = calc_ax_profile(velocity, self.raceline[:,self.S_INDEX])
        self.raceline[indices, self.AX_INDEX] = ax_profile[indices]

    def set_position(self, position:np.ndarray, start=0, end=-1):
        start_ = max(0, start-1)
        end = end if end>=0 else len(self) + end + 1

        num_points = position.shape[0]
        insert_arr = np.zeros((num_points, 13))
        insert_arr[:, self.X_INDEX:self.Y_INDEX+1] = position
        original_velocity = self.raceline[:,self.VX_INDEX]
        #import ipdb; ipdb.set_trace()
        if start<=end:
            self.raceline = np.concatenate((self.raceline[:start], insert_arr, self.raceline[end+1:]), axis=0)
            indices = np.arange(start, min(self.raceline.shape[0], end+1))
        else:
            dist_to_end = len(self) - start
            dist_to_start = end+1
            self.raceline = np.concatenate((insert_arr[-dist_to_start:], self.raceline[end+1:start], insert_arr[:dist_to_end],), axis=0)
            indices_end = np.arange(start, len(self))
            indices_start = np.arange(0, end+1)
            indices = np.concatenate((indices_end, indices_start), axis=0)

        
        # Compute new boundary distances
        dist_left, dist_right = self.get_bound_distances(position)

        self.raceline[indices, self.D_LEFT_INDEX] = dist_left
        self.raceline[indices, self.D_RIGHT_INDEX] = dist_right
        # Compute arc length of new raceline 
        d = np.diff(self.raceline[:, self.X_INDEX:self.Y_INDEX+1],axis=0)
        consecutive_diff= np.sqrt(np.sum(np.power(d, 2), axis=1))
        arc_length = np.cumsum(consecutive_diff)
        arc_length = np.insert(arc_length, 0, 0.0)
        self.raceline[:, self.S_INDEX] = arc_length
        # Compute approximate velocity profile
        new_velocity = original_velocity[indices]
        self.set_velocity(new_velocity, indices=indices)

        # TODO: Worry about ddx and ddy
        new_psi, new_kappa, tangvecs = calc_head_curv_num(
            path = self.raceline[:, self.X_INDEX:self.Y_INDEX+1],
            el_lengths = self.raceline[:, self.S_INDEX],
            return_tangvecs=True,
            is_closed=True,
        )

        self.raceline[indices, self.PSI_INDEX] =    new_psi[indices] + np.pi
        self.raceline[indices, self.KAPPA_INDEX] =  new_kappa[indices]
        self.raceline[indices, self.DX_INDEX] =     tangvecs[indices, 0]
        self.raceline[indices, self.DY_INDEX] =     tangvecs[indices, 1]   

        self.compute_trackbounds()
        if self.has_sectors:
            self.update_sectors()

    @property
    def has_sectors(self):
        return self.sectors is not None

    def update_sectors(self):
        if self.sectors[-1]["end_ind"] == self.raceline.shape[0]-1:
            return
        # Assume the sectors were defined relative to the older raceline whose last index is stored
        old_total = self.sectors[-1]["end_ind"]
        new_total = self.raceline.shape[0] - 1
        scale = new_total / old_total

        # Update sectors by scaling their start and end indices
        for sector in self.sectors:
            old_start_ind = sector["start_ind"]
            old_end_ind = sector["end_ind"]
            sector["start_ind"] = int(round(old_start_ind * scale))
            sector["end_ind"] = int(round(old_end_ind * scale))
            old_start_s = sector["start_s"]
            old_end_s = sector["end_s"]
            sector["start_s"] = int(round(old_start_s * scale))
            sector["end_s"] = int(round(old_end_s * scale))

        # Ensure the last sector ends exactly at the new raceline end
        self.sectors[-1]["end_s"] = self.raceline.shape[0] - 1

    def set_sectors_from_dict(self, sectors: dict):
        self.sectors = sectors

    def set_sectors_from_msg(self, msg):
        self.sectors = sector_msg_to_dict(msg)

    def set_sectors_from_file(self, path: str):
        self.sectors = load_sector_file(path)
        #print(self.sectors)

    @classmethod
    def from_wpnt_msg(cls, msg):
        raceline = wpnt_msg_to_array(msg)
        return cls(raceline)
    
    @classmethod
    def from_file(cls, file: str):
        raceline = load_waypoints(file)
        return cls(raceline)
    
    @property
    def has_sectors(self):
        return self.sectors is not None
    
    @classmethod
    def from_array(cls, array: np.ndarray):
        return cls(array)
    
    @property
    def trackbounds(self):
        return self.left_trackbound, self.right_trackbound
    
    @property
    def inverse_trackbounds(self):
        return self.compute_inflated_trackbounds(inverse_bounds=True)
    
    @property
    def normals(self):
        return self.left_normals, self.right_normals

    def to_wpnt_msg(self):
        return array_to_wpnt_msg(self.raceline)
    
    def to_wpnt_marker_msg(self, *args, **kwargs):
        return array_to_wpnt_marker_msg(self.raceline, *args, **kwargs)
    
    def to_sector_marker_msg(self):
        if self.sectors is None: 
            print("No sectors were defined for this raceline!")
            return None
        return sector_dict_to_marker_msg(self.sectors, self.raceline)
    
    def to_sector_msg(self):
        if self.sectors is None:
            print("No sectors were defined for this raceline!")
            return None
        return sector_dict_to_sector_msg(self.sectors)
    

    def __len__(self):
        return self.raceline.shape[0]

    def visualize(self):
        # extract columns
        s       = self.raceline[:, self.S_INDEX]
        x       = self.raceline[:, self.X_INDEX]
        y       = self.raceline[:, self.Y_INDEX]
        psi     = self.raceline[:, self.PSI_INDEX]
        d_left  = self.raceline[:, self.D_LEFT_INDEX]
        d_right = self.raceline[:, self.D_RIGHT_INDEX]
        v       = self.raceline[:, self.VX_INDEX]
        a       = self.raceline[:, self.AX_INDEX]

        # normal vector pointing to the left side of the track
        normal_x = -np.sin(psi)
        normal_y =  np.cos(psi)

        # compute border coordinates
        left_x  = x + normal_x * d_left
        left_y  = y + normal_y * d_left
        right_x = x - normal_x * d_right
        right_y = y - normal_y * d_right

        # set up figure with GridSpec: 2 rows, 2 cols
        fig = plt.figure(figsize=(12, 6))
        gs = fig.add_gridspec(2, 2,
                              width_ratios=[3, 1],
                              height_ratios=[1, 1],
                              wspace=0.3, hspace=0.4)

        ax_track = fig.add_subplot(gs[:, 0])
        ax_vel   = fig.add_subplot(gs[0, 1])
        ax_acc   = fig.add_subplot(gs[1, 1])

        # main track plot
        ax_track.plot(x, y,           'k-',  label='Centerline')
        # ax_track.plot(left_x, left_y,'r--', label='Left border')
        # ax_track.plot(right_x, right_y,'b--', label='Right border')
        ax_track.set_aspect('equal', 'box')
        ax_track.set_xlabel('X [m]')
        ax_track.set_ylabel('Y [m]')
        ax_track.set_title('Raceline and Borders')

        # annotate arc‐length every 5 m
        s_ticks = np.arange(0, s.max(), 5.0)
        for st in s_ticks:
            # find closest index
            idx = np.argmin(np.abs(s - st))
            ax_track.text(
                x[idx], y[idx],
                f"{st:.0f} m",
                fontsize=8,
                ha='center', va='bottom',
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="black", alpha=0.7)
            )

        ax_track.legend(loc='best')

        # velocity profile
        ax_vel.plot(s, v, '-')
        ax_vel.set_xlabel('Track length s [m]')
        ax_vel.set_ylabel('Velocity [m/s]')
        ax_vel.set_title('Velocity Profile')
        ax_vel.grid(True)

        # acceleration profile
        ax_acc.plot(s, a, '-')
        ax_acc.set_xlabel('Track length s [m]')
        ax_acc.set_ylabel('Acceleration [m/s²]')
        ax_acc.set_title('Acceleration Profile')
        ax_acc.grid(True)

        plt.show()

    def save(self, file: str):
        save_waypoints(file, self.raceline)
        if self.has_sectors:
            raceline_name = Path(file).stem
            sector_file = Path(file).parent / f"{raceline_name}_sectors.json"
            save_sectors(sector_file, self.sectors)
