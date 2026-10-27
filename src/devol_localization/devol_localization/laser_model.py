from numpy import cos, sin, ndarray, round as np_round, clip, abs, sum as np_sum


class LaserModel:
    @staticmethod
    def likelihood_field_range_finder(
        z_dists: ndarray, z_thetas: ndarray, x: ndarray, map_dict: dict, std: float = 0.1
    ) -> float:
        map_data = map_dict['grid']
        info = map_dict['info']
        res = info.resolution

        # Project end points to global frame
        ray_thetas = x[2] + z_thetas
        pts_x = x[0] + z_dists * cos(ray_thetas)
        pts_y = x[1] + z_dists * sin(ray_thetas)

        # Conversion to map indices
        cols: ndarray = np_round((pts_x - info.origin.position.x) / res).astype(int)
        rows: ndarray = np_round((pts_y - info.origin.position.y) / res).astype(int)
        height, width = map_data.shape

        # Out of bounds handling
        clipped_cols: ndarray = clip(cols, 0, width - 1)
        clipped_rows: ndarray = clip(rows, 0, height - 1)
        out_of_bounds_distance: float = (abs(cols - clipped_cols) + abs(rows - clipped_rows)) * res

        # Distance lookup
        dists: ndarray = map_data[clipped_rows, clipped_cols] + out_of_bounds_distance

        # Find the sum of the log probabilities
        log_prob: float = np_sum(-(dists**2) / (2 * std**2))
        return float(log_prob)
