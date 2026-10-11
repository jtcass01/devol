# lunar

A procedurally generated 100 m x 100 m patch of lunar highlands, with craters, boulders, lunar gravity,
a black sky with stars and a low sun. Everything else in this folder is written by
[`scripts/generate_lunar_world.py`](../../scripts/generate_lunar_world.py); change the generator and
re-run it rather than editing the files.

```bash
python3 src/devol_gazebo/scripts/generate_lunar_world.py                  # regenerates this folder (seed 7)
python3 src/devol_gazebo/scripts/generate_lunar_world.py --preset mare --out /tmp/lunar_mare
python3 src/devol_gazebo/scripts/generate_lunar_world.py --stats-only     # slope report only
ros2 launch devol_sim motion_planner_sim.launch.py maze:=lunar planner:=none
```

## Slopes

The terrain is matched to the Lunar Orbiter Laser Altimeter slope statistics of Rosenburg et al. (2011),
*Global surface slopes and roughness of the Moon from the Lunar Orbiter Laser Altimeter*, JGR 116,
E02001, Table 2: median bidirectional slope at the 17 m baseline of 7.5 deg in the highlands (7.6 deg
at the south pole) and 2.0 deg in the maria, with median Hurst exponents of 0.95 and 0.76.

It is built from a fractional Brownian surface with the measured Hurst exponent (self-affine up to the
measured ~1 km breakover), an equilibrium crater population (N(>=D) = 0.08 D^-2 per m^2, D from
1.5 m to 40 m, fresh to degraded), and power-law boulders 0.25-2 m with extra blocks on the rims of
fresh craters. Slopes above the 35 deg angle of repose are relaxed, as loose regolith would. The
roughness amplitude is then fitted so the 17 m median matches LOLA. `slope_report.txt` has the result:

| Baseline | 1 m | 2 m | 5 m | 10 m | 17 m | 25 m |
|---|---|---|---|---|---|---|
| Median slope (deg) | 11.3 | 10.5 | 9.3 | 8.3 | **7.5** (LOLA 7.5) | 6.9 |

10 % of the area is steeper than 20 deg and 0.7 % steeper than 30 deg (crater walls). The slopes
below 17 m are an extrapolation of the LOLA fit (LOLA does not resolve them); the `mare` preset gives
2.0 deg at 17 m and about 5 deg at 1 m.

## Files

| File | What it is |
|---|---|
| `maze_world.sdf` | The world (named `maze_world` like the others, so the launch files and kidnapper work unchanged). Gravity 1.62 m/s^2, wheel-regolith friction 0.7, sun 20 deg above the horizon. |
| `meshes/terrain.obj` | Height field, 0.5 m grid, visual and collision. |
| `meshes/rocks.obj` | 134 partly buried boulders, visual and collision. |
| `sky/star_dome.obj`, `sky/stars.png` | Unlit starfield sphere, 450 m radius. Gazebo Jetty's `<sky>` element only toggles its built-in daylight skybox (`<cubemap_uri>` is not passed to the renderer), so the stars are geometry instead. 450 m is past every sensor's range (lidars 25 m and 130 m, robot cameras clip at 100 m), so only the GUI camera sees it; robot cameras see a black sky. |
| `poses.csv` | Spawn at the origin on gentle ground and three goals 8-15 m apart, each reachable by a straight drive with no slope above 25 deg and no boulders. |
| `static_world.pcd` | The map: ASCII PCD, x y z, 265,746 points in the world (= `map`) frame. Terrain every 0.2 m over the full 100 m x 100 m, boulders every 0.05 m (their buried parts left out). Same format and 11-line header as `factory/static_world.pcd`, so `pointcloud_publisher` and `kidnap_targets.voxels_from_pcd` load it unchanged. It spans x -58 to 42 m, y -44 to 56 m and z -11.5 to 6.9 m; the spawn point is the origin. |
| `slope_report.txt` | Slope statistics of the generated terrain. |

## Using it with the current stack

The cloud loads into the existing pipeline, but that pipeline assumes a flat floor: `octomap_server`
marks everything between z = 0.1 m and 5 m as occupied, so on this terrain hills become walls and
hollows become free space in `projected_map`, and the 2D lidar hits the ground on every up-slope.
The localization and planning code needs terrain-aware handling (for example obstacles relative to
the local ground height) before its results here mean anything.
