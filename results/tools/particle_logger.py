"""Logs a 400-particle subsample of /devol_drive/pf_particles (stamp, x, y) to an npz, for offline video rendering."""
import sys, signal, numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

out = sys.argv[1]
rclpy.init()
node = Node('particle_logger', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
stamps, clouds = [], []
rng = np.random.default_rng(0)

def cb(msg):
    p = np.array([(q.position.x, q.position.y) for q in msg.poses], dtype=np.float32)
    if len(p) > 400:
        p = p[rng.choice(len(p), 400, replace=False)]
    stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
    clouds.append(np.pad(p, ((0, 400 - len(p)), (0, 0)), constant_values=np.nan))

node.create_subscription(PoseArray, '/devol_drive/pf_particles', cb, 10)
grid = {}

def map_cb(msg):
    i = msg.info
    grid.update(data=np.array(msg.data, dtype=np.int8).reshape(i.height, i.width), resolution=i.resolution,
                origin=np.array([i.origin.position.x, i.origin.position.y]))

node.create_subscription(OccupancyGrid, '/devol_drive/projected_map', map_cb,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))

def save(*_):
    np.savez_compressed(out, t=np.array(stamps), xy=np.array(clouds, dtype=np.float16), **{'map_' + k: v for k, v in grid.items()})
    raise SystemExit(0)

signal.signal(signal.SIGTERM, save); signal.signal(signal.SIGINT, save)
try:
    rclpy.spin(node)
except SystemExit:
    pass
