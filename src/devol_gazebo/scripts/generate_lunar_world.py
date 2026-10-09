#!/usr/bin/env python3
"""Procedural lunar Gazebo world with the real Moon's slope statistics, and its static point cloud.

  python3 src/devol_gazebo/scripts/generate_lunar_world.py                # writes worlds/lunar
  python3 src/devol_gazebo/scripts/generate_lunar_world.py --preset mare --out /tmp/lunar_mare
  python3 src/devol_gazebo/scripts/generate_lunar_world.py --stats-only   # slope report only

Needs numpy and Pillow. The output is deterministic for a given --seed and --preset.

The terrain is a height field on a regular grid, built from three layers:
1. Fractional Brownian surface (spectral synthesis) with the Hurst exponent LOLA measured for the
   terrain type, so slopes change with baseline the way they do on the Moon.
2. Craters with an equilibrium size-frequency distribution, N(>=D) = c D^-2 per m^2, each a
   parabolic bowl with a raised rim and ejecta falloff (h_rim (r/R)^-3). Depth/diameter runs
   from 0.17 (fresh; walls near the 30-35 deg angle of repose) down to 0.03 (degraded), skewed
   to degraded as in an equilibrium population.
3. Boulders (a separate mesh, partly buried) with a power-law size distribution, plus extra blocks
   on the rims of fresh craters.
The fBm amplitude is then scaled until the median slope at the 17 m baseline matches LOLA
(Rosenburg
et al. 2011, JGR 116, E02001, Table 2: highlands 7.5 deg, maria 2.0 deg; Hurst 0.95 and 0.76;
breakover 0.98 and 0.53 km). Slopes steeper than the 35 deg angle of repose are relaxed by mass
wasting. Slope
is the bidirectional one: the gradient magnitude from height differences over the baseline in x
and y.

Outputs (in --out):
  maze_world.sdf           world 'maze_world' (the name the launch files and kidnapper expect)
  meshes/terrain.obj, meshes/rocks.obj, meshes/*.mtl, meshes/regolith.jpg
  sky/star_dome.obj, sky/stars.png   unlit starfield sphere (radius 450 m) for the black sky
  poses.csv                spawn and three goals on drivable ground (format of the other worlds)
  static_world.pcd         ASCII x y z cloud of the terrain and boulders in the world (= map) frame
  slope_report.txt         slope statistics of the generated terrain against the LOLA targets
"""

import argparse
import os

import numpy as np
from PIL import Image

PRESETS = {
    # Rosenburg et al. 2011, Table 2: median bidirectional slope at the 17 m baseline and the
    # median Hurst exponent. Highlands also match the south polar region (7.6 deg, H 0.95).
    # Breakover: the median baseline beyond which the topography stops being self-affine.
    'highlands': {
        'slope17_deg': 7.5,
        'hurst': 0.95,
        'breakover_m': 980.0,
        'crater_c': 0.08,
        'rock_c': 4.7e-4,
    },
    'mare': {
        'slope17_deg': 2.0,
        'hurst': 0.76,
        'breakover_m': 530.0,
        'crater_c': 0.08,
        'rock_c': 2.0e-4,
    },
}
BASELINE_M = 17.0


# ---------------------------------------------------------------------------- terrain layers
def fbm_surface(n, cell, hurst, breakover, rng):
    """Zero-mean fractional Brownian surface on an n x n grid with unit RMS slope at one cell.

    Synthesised by FFT on a periodic grid `breakover` m wide and cropped, so it does not tile. The
    spectrum is self-affine (power ~ k^-(2H+2)) down to the breakover wavenumber and flat below it,
    the way LOLA finds the Moon stops being self-affine beyond about a kilometre."""
    m = max(2 * n, int(round(breakover / cell)))
    kx = np.fft.fftfreq(m, d=cell)
    ky = np.fft.rfftfreq(m, d=cell)
    k = np.maximum(np.hypot(*np.meshgrid(kx, ky, indexing='ij')), 1.0 / breakover)
    amp = k ** -(hurst + 1.0)
    amp[0, 0] = 0.0
    noise = rng.standard_normal(amp.shape) + 1j * rng.standard_normal(amp.shape)
    h = np.fft.irfft2(amp * noise, s=(m, m))[:n, :n]
    h -= h.mean()
    gx, gy = np.gradient(h, cell)
    return h / np.sqrt(np.mean(gx**2 + gy**2))


def relax_slopes(h, cell, max_deg, iterations=200):
    """Mass wasting: move material downhill wherever the slope to a grid neighbour exceeds the
    angle of repose, so the steepest ground ends near max_deg like loose regolith does."""
    limit = np.tan(np.radians(max_deg)) * cell
    h = h.copy()
    for _ in range(iterations):
        moved = False
        for axis in (0, 1):
            d = np.diff(h, axis=axis)
            excess = np.where(np.abs(d) > limit, (np.abs(d) - limit) * np.sign(d) / 2.0, 0.0)
            if not excess.any():
                continue
            moved = True
            pad = [(0, 0), (0, 0)]
            pad[axis] = (0, 1)
            h += 0.5 * np.pad(excess, pad)
            pad[axis] = (1, 0)
            h -= 0.5 * np.pad(excess, pad)
        if not moved:
            break
    return h


def crater_layer(x, y, rng, c, d_min, d_max, margin):
    """Height field of an equilibrium crater population over the grid x, y (m)."""
    x0, x1, y0, y1 = x.min() - margin, x.max() + margin, y.min() - margin, y.max() + margin
    area = (x1 - x0) * (y1 - y0)
    count = rng.poisson(c * area * (d_min**-2 - d_max**-2))
    # Inverse-CDF sample of N(>=D) ~ D^-2 truncated to [d_min, d_max].
    u = rng.random(count)
    diam = (d_min**-2 - u * (d_min**-2 - d_max**-2)) ** -0.5
    order = np.argsort(-diam)  # big (old) first, so small fresh craters are superposed on them
    h = np.zeros_like(x)
    craters = []
    for i in order:
        d = diam[i]
        cx, cy = rng.uniform(x0, x1), rng.uniform(y0, y1)
        fresh = rng.random() ** 2  # equilibrium populations are mostly degraded
        depth = d * (0.03 + 0.14 * fresh)
        rim = 0.25 * depth
        r = d / 2
        sl = (slice(None), slice(None))
        # Only touch the window the crater and its ejecta (to 3 R) cover.
        ix = np.where(np.abs(x[:, 0] - cx) < 3 * r)[0]
        iy = np.where(np.abs(y[0, :] - cy) < 3 * r)[0]
        if len(ix) == 0 or len(iy) == 0:
            continue
        sl = (slice(ix[0], ix[-1] + 1), slice(iy[0], iy[-1] + 1))
        q = np.hypot(x[sl] - cx, y[sl] - cy) / r
        inside = rim + depth * (q**2 - 1.0)
        outside = rim * np.clip(q, 1.0, None) ** -3 - rim / 27.0 * np.clip(q, 1.0, 3.0) / 3.0
        prof = np.where(q < 1.0, inside, np.where(q < 3.0, outside, 0.0))
        # A new crater excavates what was there: inside the rim the old relief is mostly erased.
        erase = np.clip(1.0 - q, 0.0, 1.0) ** 0.5 * (0.3 + 0.7 * fresh)
        h[sl] = h[sl] * (1.0 - erase) + prof
        craters.append((cx, cy, d, fresh))
    return h, craters


def median_slope_deg(h, cell, baseline):
    """Median bidirectional slope (deg) over `baseline` m: gradient magnitude from central height
    differences over the baseline in x and y."""
    s = max(1, int(round(baseline / cell / 2)))
    sx = (h[2 * s :, s:-s] - h[: -2 * s, s:-s]) / (2 * s * cell)
    sy = (h[s:-s, 2 * s :] - h[s:-s, : -2 * s]) / (2 * s * cell)
    return float(np.degrees(np.arctan(np.median(np.hypot(sx, sy)))))


def slope_map_deg(h, cell):
    gx, gy = np.gradient(h, cell)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


# ---------------------------------------------------------------------------- boulders
def icosphere(subdiv):
    t = (1 + 5**0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t)]
    v += [(0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4)]
    f += [(11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8)]
    f += [(3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    v = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(subdiv):
        cache, nf = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                p = v[a] + v[b]
                v.append(p / np.linalg.norm(p))
                cache[key] = len(v) - 1
            return cache[key]

        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    return np.array(v), np.array(f)


def make_rocks(rng, terrain, rock_c, craters, keep_out):
    """Partly buried, irregular boulders. Returns (vertices, faces, list of (x, y, radius))."""
    x0, x1, y0, y1 = terrain.bounds
    area = (x1 - x0) * (y1 - y0)
    d_min, d_max = 0.25, 2.0
    n = rng.poisson(rock_c * area * (d_min**-2.5 - d_max**-2.5))
    u = rng.random(n)
    diam = list((d_min**-2.5 - u * (d_min**-2.5 - d_max**-2.5)) ** (-1 / 2.5))
    pos = [(rng.uniform(x0, x1), rng.uniform(y0, y1)) for _ in range(n)]
    # Fresh craters >= 6 m throw blocks onto their rims.
    for cx, cy, d, fresh in craters:
        if d >= 6.0 and fresh > 0.6:
            for _ in range(rng.poisson(1.5 * d / 6.0)):
                a, rr = rng.uniform(0, 2 * np.pi), d / 2 * rng.uniform(0.9, 1.6)
                pos.append((cx + rr * np.cos(a), cy + rr * np.sin(a)))
                diam.append(float(rng.uniform(0.2, 0.15 * min(d, 8.0))))
    shapes = {1: icosphere(1), 2: icosphere(2)}
    verts, faces, placed = [], [], []
    for (px, py), d in zip(pos, diam):
        if not (x0 + 1 < px < x1 - 1 and y0 + 1 < py < y1 - 1):
            continue
        if any(np.hypot(px - kx, py - ky) < kr + d / 2 for kx, ky, kr in keep_out):
            continue
        base_v, base_f = shapes[2 if d > 0.6 else 1]  # small blocks need fewer faces
        # Low-order random bumps make each block irregular; axes give a flattened, elongated shape.
        bump = 1.0 + 0.18 * np.sin(base_v @ rng.normal(size=3) * 2.0 + rng.uniform(0, 6))
        bump += 0.10 * np.sin(base_v @ rng.normal(size=3) * 4.0 + rng.uniform(0, 6))
        axes = d / 2 * np.array([1.0, rng.uniform(0.6, 0.9), rng.uniform(0.45, 0.75)])
        p = base_v * bump[:, None] * axes
        yaw = rng.uniform(0, 2 * np.pi)
        cz, sz = np.cos(yaw), np.sin(yaw)
        p = p @ np.array([[cz, sz, 0], [-sz, cz, 0], [0, 0, 1]])
        bury = rng.uniform(0.25, 0.45) * 2 * axes[2]
        ground = terrain.height(np.full(len(p), px) + p[:, 0], np.full(len(p), py) + p[:, 1])
        z0 = terrain.height(np.array([px]), np.array([py]))[0]
        p[:, 0] += px
        p[:, 1] += py
        p[:, 2] += max(z0, ground.min() + 0.3 * axes[2]) + axes[2] - bury
        faces.append(base_f + sum(len(q) for q in verts))
        verts.append(p)
        placed.append((px, py, d / 2))
    if not verts:
        return np.zeros((0, 3)), np.zeros((0, 3), int), []
    return np.vstack(verts), np.vstack(faces), placed


# ---------------------------------------------------------------------------- terrain mesh
class Terrain:
    """Regular-grid height field triangulated with the diagonal from (i, j) to (i+1, j+1)."""

    def __init__(self, h, cell, x0, y0):
        self.h, self.cell, self.x0, self.y0 = h, cell, x0, y0
        n = h.shape[0]
        self.bounds = (x0, x0 + (n - 1) * cell, y0, y0 + (n - 1) * cell)

    def height(self, x, y):
        """Exact height of the triangulated surface at x, y (arrays)."""
        n = self.h.shape[0]
        fx = np.clip((np.asarray(x) - self.x0) / self.cell, 0, n - 1 - 1e-9)
        fy = np.clip((np.asarray(y) - self.y0) / self.cell, 0, n - 1 - 1e-9)
        i, j = np.floor(fx).astype(int), np.floor(fy).astype(int)
        u, v = fx - i, fy - j
        h00, h11 = self.h[i, j], self.h[i + 1, j + 1]
        h10, h01 = self.h[i + 1, j], self.h[i, j + 1]
        lower = u >= v  # triangle (00, 10, 11)
        return np.where(
            lower, h00 + u * (h10 - h00) + v * (h11 - h10), h00 + v * (h01 - h00) + u * (h11 - h01)
        )

    def normals(self):
        gx, gy = np.gradient(self.h, self.cell)
        nrm = np.dstack([-gx, -gy, np.ones_like(gx)])
        return nrm / np.linalg.norm(nrm, axis=2, keepdims=True)

    def write_obj(self, path, mtl, uv_tile):
        n = self.h.shape[0]
        ii, jj = np.meshgrid(np.arange(n), np.arange(n), indexing='ij')
        x, y = self.x0 + ii * self.cell, self.y0 + jj * self.cell
        idx = ii * n + jj + 1
        a, b = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel()
        c, d = idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
        with open(path, 'w') as f:
            f.write(f'# generate_lunar_world.py: {n}x{n} height field, {self.cell} m cells\n')
            f.write(f'mtllib {os.path.basename(mtl)}\no terrain\n')
            np.savetxt(f, np.c_[x.ravel(), y.ravel(), self.h.ravel()], fmt='v %.3f %.3f %.3f')
            np.savetxt(f, np.c_[x.ravel(), y.ravel()] / uv_tile, fmt='vt %.4f %.4f')
            np.savetxt(f, self.normals().reshape(-1, 3), fmt='vn %.4f %.4f %.4f')
            f.write('usemtl regolith\n')
            for tri in (np.c_[a, b, c], np.c_[a, c, d]):  # counter-clockwise seen from +z
                np.savetxt(f, np.repeat(tri, 3, axis=1), fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')


def write_rocks_obj(path, mtl, v, f):
    # Per-vertex normals from the faces around each vertex.
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    vn = np.zeros_like(v)
    for k in range(3):
        np.add.at(vn, f[:, k], fn)
    vn /= np.linalg.norm(vn, axis=1, keepdims=True) + 1e-12
    with open(path, 'w') as out:
        out.write(f'# generate_lunar_world.py: {len(f)} boulder faces\n')
        out.write(f'mtllib {os.path.basename(mtl)}\no rocks\n')
        np.savetxt(out, v, fmt='v %.4f %.4f %.4f')
        np.savetxt(out, v[:, :2] / 0.5, fmt='vt %.4f %.4f')
        np.savetxt(out, vn, fmt='vn %.4f %.4f %.4f')
        out.write('usemtl rock\n')
        np.savetxt(out, np.repeat(f + 1, 3, axis=1), fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')


def write_mtl(path, name, texture, kd):
    with open(path, 'w') as f:
        f.write(f'newmtl {name}\nKa 0 0 0\nKd {kd} {kd} {kd}\nKs 0.02 0.02 0.02\nNs 4\nillum 2\n')
        f.write(f'map_Kd {texture}\n')


def regolith_texture(path, rng, size=1024):
    """Seamless grey regolith albedo: multi-scale noise plus small bright and dark grains."""
    k = np.hypot(*np.meshgrid(np.fft.fftfreq(size), np.fft.fftfreq(size), indexing='ij'))
    k[0, 0] = np.inf
    noise = np.fft.ifft2(k**-0.9 * np.exp(2j * np.pi * rng.random((size, size)))).real
    noise = (noise - noise.mean()) / noise.std()
    img = 0.55 + 0.07 * noise + 0.05 * rng.standard_normal((size, size))
    grains = rng.random((size, size))
    img[grains > 0.997] += 0.25
    img[grains < 0.002] -= 0.2
    rgb = np.clip(np.dstack([img, img * 0.985, img * 0.96]) * 255, 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(path, quality=90)


# ---------------------------------------------------------------------------- star dome
# gz-sim (Jetty) only switches its built-in daylight skybox on or off; <sky><cubemap_uri> is not
# passed to the renderer. So the stars are an unlit, inward-facing sphere far outside every
# sensor's range (lidars 25 m and 130 m, robot cameras clip at 100 m), seen only by the GUI camera.
def star_texture(path, rng, width, count):
    """Equirectangular starfield (u = azimuth, v = elevation), stars spread isotropically."""
    height = width // 2
    img = np.zeros((height, width, 3), np.float32)
    z = rng.uniform(-1, 1, count)  # uniform on the sphere
    lon = rng.uniform(-np.pi, np.pi, count)
    lat = np.arcsin(z)
    # Apparent brightness: many faint stars, few bright ones (N(<m) grows ~ 10^(0.5 m)).
    mag = np.log10(rng.random(count)) / 0.5 + 6.0
    flux = np.clip(10 ** (-0.4 * (mag - 3.0)), 0.0, 4.0)
    temp = rng.uniform(-1, 1, count)  # blue-white to orange tint
    tint = np.c_[1 - 0.15 * temp.clip(0), 1 - 0.05 * np.abs(temp), 1 + 0.15 * temp.clip(None, 0)]
    u = (lon + np.pi) / (2 * np.pi) * width
    v = (np.pi / 2 - lat) / np.pi * height
    for s in range(count):
        sigma_v = 0.5 + 0.4 * min(flux[s], 1.0)
        sigma_u = sigma_v / max(np.cos(lat[s]), 0.05)  # keep stars round near the poles
        ru, rv = int(np.ceil(2 * sigma_u)), int(np.ceil(2 * sigma_v))
        for dr in range(-rv, rv + 1):
            rr = int(v[s]) + dr
            if not 0 <= rr < height:
                continue
            for dc in range(-ru, ru + 1):
                cc = int(u[s]) + dc
                w = np.exp(
                    -(((rr + 0.5 - v[s]) / sigma_v) ** 2 + ((cc + 0.5 - u[s]) / sigma_u) ** 2) / 2
                )
                img[rr, cc % width] += flux[s] * w * tint[s]
    rgb = (np.clip(img, 0, 1) ** (1 / 1.4) * 255).astype(np.uint8)
    Image.fromarray(rgb).save(path, optimize=True)


def write_dome_obj(path, mtl, radius, n_lon=96, n_lat=48):
    """UV sphere seen from inside (faces wound to face the centre), equirectangular UVs."""
    lon = np.linspace(-np.pi, np.pi, n_lon + 1)
    lat = np.linspace(np.pi / 2, -np.pi / 2, n_lat + 1)
    lo, la = np.meshgrid(lon, lat)
    d = np.dstack([np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]).reshape(-1, 3)
    uv = np.c_[(lo.ravel() + np.pi) / (2 * np.pi), 1 - (np.pi / 2 - la.ravel()) / np.pi]
    idx = np.arange((n_lat + 1) * (n_lon + 1)).reshape(n_lat + 1, n_lon + 1) + 1
    a, b = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel()
    c, e = idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    with open(path, 'w') as f:
        f.write(f'# generate_lunar_world.py: star dome, radius {radius} m\n')
        f.write(f'mtllib {os.path.basename(mtl)}\no star_dome\n')
        np.savetxt(f, d * radius, fmt='v %.2f %.2f %.2f')
        np.savetxt(f, uv, fmt='vt %.5f %.5f')
        np.savetxt(f, -d, fmt='vn %.4f %.4f %.4f')
        f.write('usemtl stars\n')
        for tri in (np.c_[a, b, c], np.c_[a, c, e]):
            np.savetxt(f, np.repeat(tri, 3, axis=1), fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')


# ---------------------------------------------------------------------------- route and cloud
def drivable(terrain, slope1, rocks, x, y, max_slope, clearance):
    i = int(round((x - terrain.x0) / terrain.cell))
    j = int(round((y - terrain.y0) / terrain.cell))
    r = int(np.ceil(clearance / terrain.cell))
    n = slope1.shape[0]
    if not (r <= i < n - r and r <= j < n - r):
        return False
    if slope1[i - r : i + r + 1, j - r : j + r + 1].max() > max_slope:
        return False
    return all(np.hypot(x - rx, y - ry) > rr + clearance for rx, ry, rr in rocks)


def segment_ok(terrain, slope1, rocks, a, b, max_slope, clearance):
    n = max(2, int(np.hypot(b[0] - a[0], b[1] - a[1]) / 0.25))
    for t in np.linspace(0, 1, n):
        x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        if not drivable(terrain, slope1, rocks, x, y, max_slope, clearance):
            return False
    return True


def pick_route(rng, terrain, slope1, rocks, args):
    """Spawn near the centre and three goals 8-15 m apart, each on gentle ground with a straight,
    rock-free drive from the previous one (the planner may still choose another path)."""
    x0, x1, y0, y1 = terrain.bounds
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    for _ in range(20000):
        route = [(cx + rng.uniform(-10, 10), cy + rng.uniform(-10, 10))]
        if not drivable(terrain, slope1, rocks, *route[0], args.spawn_max_slope, 1.0):
            continue
        for _ in range(3):
            for _ in range(200):
                a = rng.uniform(0, 2 * np.pi)
                rr = rng.uniform(8, 15)
                g = (route[-1][0] + rr * np.cos(a), route[-1][1] + rr * np.sin(a))
                if (
                    np.hypot(g[0] - cx, g[1] - cy) > 22
                    or min(np.hypot(g[0] - p[0], g[1] - p[1]) for p in route) < 6
                ):
                    continue
                if drivable(terrain, slope1, rocks, *g, args.goal_max_slope, 0.8) and segment_ok(
                    terrain, slope1, rocks, route[-1], g, args.path_max_slope, 0.5
                ):
                    route.append(g)
                    break
            else:
                break
        if len(route) == 4:
            return route
    raise RuntimeError('no drivable route found; try another --seed')


def sample_cloud(terrain, rock_v, rock_f, spacing, rock_spacing, rng):
    x0, x1, y0, y1 = terrain.bounds
    gx = np.arange(x0, x1 + 1e-9, spacing)
    gy = np.arange(y0, y1 + 1e-9, spacing)
    xx, yy = np.meshgrid(gx, gy, indexing='ij')
    pts = [np.c_[xx.ravel(), yy.ravel(), terrain.height(xx.ravel(), yy.ravel())]]
    if len(rock_f):
        a, b, c = rock_v[rock_f[:, 0]], rock_v[rock_f[:, 1]], rock_v[rock_f[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        n = rng.poisson(area / rock_spacing**2)
        tri = np.repeat(np.arange(len(rock_f)), n)
        r1, r2 = np.sqrt(rng.random(len(tri))), rng.random(len(tri))
        p = (1 - r1)[:, None] * a[tri] + (r1 * (1 - r2))[:, None] * b[tri]
        p += (r1 * r2)[:, None] * c[tri]
        above = p[:, 2] > terrain.height(p[:, 0], p[:, 1]) + 0.01  # drop the buried part
        pts.append(p[above])
    return np.vstack(pts)


def write_pcd(path, pts):
    # Same 11-line ASCII header as worlds/factory/static_world.pcd (kidnap_targets skips 11 rows).
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n')
        f.write('SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write(f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\n')
        f.write('DATA ascii\n')
        np.savetxt(f, pts, fmt='%.3f')


def world_sdf(args, sun_dir):
    friction = f'<ode><mu>{args.mu}</mu><mu2>{args.mu}</mu2></ode>'

    def mesh_link(name, uri):
        return f"""
        <collision name="{name}_collision">
          <geometry><mesh><uri>{uri}</uri></mesh></geometry>
          <surface><friction>{friction}</friction></surface>
        </collision>
        <visual name="{name}_visual">
          <geometry><mesh><uri>{uri}</uri></mesh></geometry>
        </visual>"""

    terrain = mesh_link('terrain', 'meshes/terrain.obj')
    rocks = mesh_link('rocks', 'meshes/rocks.obj')
    return f"""<?xml version="1.0"?>
<!-- Generated by src/devol_gazebo/scripts/generate_lunar_world.py
     (preset {args.preset}, seed {args.seed}).
     Edit the generator and re-run it rather than editing this file. -->
<sdf version="1.9">
  <world name="maze_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Lunar surface gravity -->
    <gravity>0 0 -1.62</gravity>

    <!-- No atmosphere: black sky, almost no ambient light, hard shadows. The stars are the
         star_dome model below (the built-in <sky> is a daylight skybox, so it stays off). -->
    <scene>
      <ambient>0.06 0.06 0.06 1</ambient>
      <background>0 0 0 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- Sun {args.sun_elevation} deg up, azimuth {args.sun_azimuth} deg from +x. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>1.0 0.98 0.95 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <intensity>1.4</intensity>
      <direction>{sun_dir[0]:.4f} {sun_dir[1]:.4f} {sun_dir[2]:.4f}</direction>
    </light>

    <model name="lunar_terrain">
      <static>true</static>
      <link name="terrain_link">{terrain}{rocks}
      </link>
    </model>

    <!-- Unlit starfield {args.dome_radius:.0f} m away: past every sensor's range, visual only. -->
    <model name="star_dome">
      <static>true</static>
      <link name="dome_link">
        <visual name="stars">
          <cast_shadows>false</cast_shadows>
          <geometry><mesh><uri>sky/star_dome.obj</uri></mesh></geometry>
          <material>
            <lighting>false</lighting>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>sky/stars.png</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument('--out', default=os.path.join(here, '..', 'worlds', 'lunar'))
    p.add_argument('--preset', choices=sorted(PRESETS), default='highlands')
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--size', type=float, default=100.0, help='terrain side length, m')
    p.add_argument('--cell', type=float, default=0.5, help='height-field grid spacing, m')
    p.add_argument('--crater-min', type=float, default=1.5, help='smallest crater diameter, m')
    p.add_argument('--crater-max', type=float, default=40.0, help='largest crater diameter, m')
    p.add_argument('--cloud-spacing', type=float, default=0.2, help='terrain point spacing, m')
    p.add_argument('--rock-spacing', type=float, default=0.05, help='boulder point spacing, m')
    p.add_argument('--repose', type=float, default=35.0, help='angle of repose, deg')
    p.add_argument('--mu', type=float, default=0.7, help='wheel-regolith friction coefficient')
    p.add_argument('--sun-elevation', type=float, default=20.0, help='deg above horizon')
    p.add_argument('--sun-azimuth', type=float, default=135.0, help='deg, from +x toward +y')
    p.add_argument('--stars', type=int, default=20000)
    p.add_argument('--sky-width', type=int, default=4096, help='star texture width, px')
    p.add_argument('--dome-radius', type=float, default=450.0, help='star dome radius, m')
    p.add_argument('--spawn-max-slope', type=float, default=10.0, help='deg within 1 m of spawn')
    p.add_argument('--goal-max-slope', type=float, default=14.0, help='deg within 0.8 m of goals')
    p.add_argument('--path-max-slope', type=float, default=25.0, help='deg along goal-to-goal')
    p.add_argument('--stats-only', action='store_true')
    args = p.parse_args()

    preset = PRESETS[args.preset]
    rng = np.random.default_rng(args.seed)
    n = int(round(args.size / args.cell)) + 1
    half = (n - 1) * args.cell / 2
    ax = np.linspace(-half, half, n)
    x, y = np.meshgrid(ax, ax, indexing='ij')

    craters_h, craters = crater_layer(
        x, y, rng, preset['crater_c'], args.crater_min, args.crater_max, args.crater_max / 2
    )
    fbm = fbm_surface(n, args.cell, preset['hurst'], preset['breakover_m'], rng)
    target = preset['slope17_deg']
    # Craters alone should leave room for the fBm roughness: cap them at 60% of the target slope.
    crater_only = median_slope_deg(craters_h, args.cell, BASELINE_M)
    if crater_only > 0.6 * target:
        craters_h *= 0.6 * target / crater_only

    def surface(amplitude):
        return relax_slopes(craters_h + amplitude * fbm, args.cell, args.repose)

    lo, hi = 0.0, 1.0
    while median_slope_deg(surface(hi), args.cell, BASELINE_M) < target:
        hi *= 2
    for _ in range(30):  # bisection on the fBm amplitude
        mid = (lo + hi) / 2
        if median_slope_deg(surface(mid), args.cell, BASELINE_M) < target:
            lo = mid
        else:
            hi = mid
    h = surface(hi)

    report = [f'preset {args.preset}, seed {args.seed}, {args.size:.0f} m x {args.size:.0f} m, ']
    report[0] += f'{args.cell} m grid, {len(craters)} craters >= {args.crater_min} m'
    report.append(
        f'LOLA target (Rosenburg et al. 2011): median {target} deg at 17 m, H {preset["hurst"]}'
    )
    for b in (1.0, 2.0, 5.0, 10.0, 17.0, 25.0):
        report.append(
            f'  median slope over {b:4.0f} m baseline: {median_slope_deg(h, args.cell, b):5.2f}'
            + ' deg'
        )
    s1 = slope_map_deg(h, args.cell)
    pct = np.percentile(s1, [50, 75, 90, 95, 99, 100])
    report.append(
        '  local slope (one-cell gradient) percentiles 50/75/90/95/99/max: '
        + ' / '.join(f'{v:.1f}' for v in pct)
        + ' deg'
    )
    for lim in (10, 15, 20, 25, 30):
        report.append(f'  area steeper than {lim} deg: {100 * np.mean(s1 > lim):5.1f} %')
    if args.stats_only:
        print('\n'.join(report))
        return

    terrain = Terrain(h, args.cell, -half, -half)
    rock_v, rock_f, rocks = make_rocks(rng, terrain, preset['rock_c'], craters, keep_out=[])
    route = pick_route(rng, terrain, s1, rocks, args)
    # Put the spawn point at the world origin, on the ground.
    sx, sy = route[0]
    sz = float(terrain.height(np.array([sx]), np.array([sy]))[0])
    terrain = Terrain(h - sz, args.cell, -half - sx, -half - sy)
    rock_v = rock_v - [sx, sy, sz]
    rocks = [(rx - sx, ry - sy, rr) for rx, ry, rr in rocks]
    route = [(gx - sx, gy - sy) for gx, gy in route]
    report.append(
        f'{len(rocks)} boulders 0.2-2 m; spawn and goals (x, y): '
        + ', '.join(f'({gx:.2f}, {gy:.2f})' for gx, gy in route)
    )

    out = os.path.abspath(args.out)
    os.makedirs(os.path.join(out, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(out, 'sky'), exist_ok=True)
    tex_rng = np.random.default_rng(args.seed + 1)
    regolith_texture(os.path.join(out, 'meshes', 'regolith.jpg'), tex_rng)
    write_mtl(os.path.join(out, 'meshes', 'terrain.mtl'), 'regolith', 'regolith.jpg', 0.8)
    write_mtl(os.path.join(out, 'meshes', 'rocks.mtl'), 'rock', 'regolith.jpg', 0.6)
    terrain.write_obj(
        os.path.join(out, 'meshes', 'terrain.obj'), os.path.join(out, 'meshes', 'terrain.mtl'), 6.0
    )
    write_rocks_obj(
        os.path.join(out, 'meshes', 'rocks.obj'),
        os.path.join(out, 'meshes', 'rocks.mtl'),
        rock_v,
        rock_f,
    )
    star_texture(
        os.path.join(out, 'sky', 'stars.png'),
        np.random.default_rng(args.seed + 2),
        args.sky_width,
        args.stars,
    )
    with open(os.path.join(out, 'sky', 'star_dome.mtl'), 'w') as f:
        f.write('newmtl stars\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nillum 0\nmap_Kd stars.png\n')
    write_dome_obj(
        os.path.join(out, 'sky', 'star_dome.obj'),
        os.path.join(out, 'sky', 'star_dome.mtl'),
        args.dome_radius,
    )

    el, az = np.radians(args.sun_elevation), np.radians(args.sun_azimuth)
    sun_dir = -np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    with open(os.path.join(out, 'maze_world.sdf'), 'w') as f:
        f.write(world_sdf(args, sun_dir))

    with open(os.path.join(out, 'poses.csv'), 'w') as f:
        f.write('name,x,y,z,yaw\n')
        names = ['robot', 'goal_1', 'goal_2', 'goal_3']
        for k, (gx, gy) in enumerate(route):
            gz = float(terrain.height(np.array([gx]), np.array([gy]))[0])
            nxt = route[min(k + 1, 3)] if k < 3 else route[k]
            prv = route[k - 1] if k else route[k]
            dx, dy = (nxt[0] - gx, nxt[1] - gy) if k < 3 else (gx - prv[0], gy - prv[1])
            dz = 0.2 if k == 0 else 0.1  # spawn above the ground like the factory world
            f.write(f'{names[k]},{gx:.2f},{gy:.2f},{gz + dz:.2f},{np.arctan2(dy, dx):.4f}\n')

    cloud = sample_cloud(
        terrain,
        rock_v,
        rock_f,
        args.cloud_spacing,
        args.rock_spacing,
        np.random.default_rng(args.seed + 3),
    )
    write_pcd(os.path.join(out, 'static_world.pcd'), cloud)
    report.append(f'static_world.pcd: {len(cloud)} points, terrain every {args.cloud_spacing} m, ')
    report[-1] += (
        f'boulders every {args.rock_spacing} m; z {cloud[:, 2].min():.2f} to '
        + f'{cloud[:, 2].max():.2f} m'
    )
    with open(os.path.join(out, 'slope_report.txt'), 'w') as f:
        f.write('\n'.join(report) + '\n')
    print('\n'.join(report))


if __name__ == '__main__':
    main()
