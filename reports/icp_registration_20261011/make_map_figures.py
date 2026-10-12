#!/usr/bin/env python3
"""Render 图2 点云地图 and 图3 体素地图 from the archived SLAM map.

Source of truth is the same archive the ICP study used:
  start/maps/inspection_building.dmap   DRONE_GRID_V2, 0.10 m, odom frame
transformed into the Gazebo world frame with the saved world_from_map metadata,
so both figures line up with the Gazebo scene screenshot and the ICP overlay.

Deliberately different in character:
  fig_point_cloud_map.png  the map drawn the way a PCD viewer draws it
                           (small unlit points, height-coloured)
  fig_voxel_map.png        the same map drawn as an occupancy grid
                           (3-D voxel cubes + per-storey horizontal slices)
"""
from pathlib import Path
import json
import math
import numpy as np

PROJECT = Path(__file__).resolve().parents[2]
DMAP = PROJECT/'start/maps/inspection_building.dmap'
META = PROJECT/'reports/two_floor_patrol_20261009_153401/metadata.json'
OUT = Path(__file__).resolve().parent


def Rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def load_occupied():
    """Occupied voxel centres in the Gazebo world frame."""
    meta = json.loads(META.read_text())
    with DMAP.open() as f:
        h = f.readline().split()
        assert h[0] == 'DRONE_GRID_V2' and h[1] == 'odom', h[:2]
        res = float(h[2])
        origin = np.array(list(map(float, h[3:6])))
        dims = np.array(list(map(int, h[6:9])))
        count = int(h[9])
        saved = np.array(list(map(float, h[10:13])))
        saved_yaw = math.radians(float(h[13]))
        occupied = [a for a, ev, _ in
                    (map(int, f.readline().split()) for _ in range(count)) if ev]
    addr = np.asarray(occupied, dtype=np.int64)
    idx = np.column_stack((addr//(dims[1]*dims[2]),
                           addr//dims[2] % dims[1],
                           addr % dims[2]))
    # identical to map_session.read_preview / run_icp_overlap.load_map_points
    pts = (origin + (idx + .5)*res) @ Rz(saved_yaw).T + saved
    pts = pts @ Rz(math.radians(meta['world_from_map_yaw'])).T + \
        np.array(meta['world_from_map_translation'], float)
    return pts, res, meta


def main():
    world, res, meta = load_occupied()
    print(f'occupied voxels: {len(world)}  res {res} m  '
          f'X {world[:,0].min():.2f}..{world[:,0].max():.2f} '
          f'Y {world[:,1].min():.2f}..{world[:,1].max():.2f} '
          f'Z {world[:,2].min():.2f}..{world[:,2].max():.2f}')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    fam = FontProperties(
        fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
    plt.rcParams.update({'font.family': fam, 'axes.unicode_minus': False,
                         'font.size': 11, 'savefig.dpi': 220})
    XL, YL, ZL = (-30, 30), (-8, 22), (-1.5, 8)

    # ---------------- 图2 点云地图 ------------------------------------
    fig = plt.figure(figsize=(15, 6.6))
    for i, (elev, azim, title) in enumerate(
            [(88, -90, '(a) 俯视图'), (26, -66, '(b) 三维斜视图')]):
        ax = fig.add_subplot(1, 2, i+1, projection='3d')
        sc = ax.scatter(world[:, 0], world[:, 1], world[:, 2], c=world[:, 2],
                        cmap='turbo', s=.25, alpha=.55, linewidths=0)
        ax.set(xlim=XL, ylim=YL, zlim=ZL, xlabel='世界 X / m',
               ylabel='世界 Y / m', zlabel='Z / m', title=title)
        ax.set_box_aspect((XL[1]-XL[0], YL[1]-YL[0], (ZL[1]-ZL[0])*4))
        ax.view_init(elev=elev, azim=azim)
        if i == 1:
            fig.colorbar(sc, ax=ax, shrink=.6, pad=.08, label='高度 Z / m')
    fig.suptitle(f'图 2  SLAM 点云地图（占据体素中心，{len(world):,} 点，'
                 f'分辨率 {res:.2f} m，来自 inspection_building.dmap）', y=.98)
    fig.tight_layout()
    fig.savefig(OUT/'fig_point_cloud_map.png', bbox_inches='tight')
    plt.close(fig)

    # ---------------- 图3 体素地图 ------------------------------------
    # storey levels from the Z histogram of occupied voxels
    hist, edges = np.histogram(world[:, 2], bins=np.arange(-1.5, 8.0, .1))
    zc = (edges[:-1] + edges[1:])/2
    levels = []
    for i in np.argsort(hist)[::-1]:
        if all(abs(zc[i]-z) > 1.5 for z in levels):
            levels.append(zc[i])
            if len(levels) == 2:
                break
    z1, z2 = sorted(levels)
    print(f'storey levels: z = {z1:.2f} m, {z2:.2f} m')

    step = 0.5
    gx = np.arange(XL[0], XL[1]+step, step)
    gy = np.arange(YL[0], YL[1]+step, step)
    gz = np.arange(ZL[0], ZL[1]+step, step)
    H, _ = np.histogramdd(world, bins=(gx, gy, gz))
    filled = H > 0
    # mpl >=3.5 wants full coordinate arrays, not 1-D edge vectors
    GX, GY, GZ = np.meshgrid(gx, gy, gz, indexing='ij')
    print(f'3-D voxel view: {filled.sum():,} filled cells of {step:.1f} m')

    cmap = plt.get_cmap('viridis')
    zmid = (gz[:-1] + gz[1:])/2
    face = cmap(np.clip((zmid - ZL[0])/(ZL[1]-ZL[0]), 0, 1))
    face = np.broadcast_to(face[None, None, :, :], filled.shape + (4,))

    fig = plt.figure(figsize=(16, 7.2))
    gs = fig.add_gridspec(2, 2, width_ratios=(1.15, 1))
    for row, (z, name) in enumerate([(z1, '一层'), (z2, '二层')]):
        ax = fig.add_subplot(gs[row, 1])
        zs = z + .8
        band = world[np.abs(world[:, 2] - zs) <= res]
        ax.scatter(band[:, 0], band[:, 1], s=.05, c='#1f4e79', alpha=.85,
                   linewidths=0)
        ax.set(xlim=XL, ylim=YL, xlabel='世界 X / m', ylabel='世界 Y / m',
               title=f'({chr(98+row)}) {name}水平切片  z ≈ {zs:.1f} m'
                     f'（{len(band):,} 体素）', aspect='equal')
        ax.grid(alpha=.25)

    ax = fig.add_subplot(gs[:, 0], projection='3d')
    ax.voxels(GX, GY, GZ, filled, facecolors=face, edgecolors=None, shade=True)
    ax.set(xlim=XL, ylim=YL, zlim=ZL, xlabel='世界 X / m', ylabel='世界 Y / m',
           zlabel='Z / m',
           title=f'(a) 三维体素立方（按 {step:.1f} m 聚合，{filled.sum():,} 块）')
    ax.set_box_aspect((XL[1]-XL[0], YL[1]-YL[0], (ZL[1]-ZL[0])*4))
    ax.view_init(elev=32, azim=-66)

    fig.suptitle(f'图 3  体素地图（占据栅格，分辨率 {res:.2f} m，'
                 f'{len(world):,} 个占据体素，来自 inspection_building.dmap）', y=.99)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(OUT/'fig_voxel_map.png', bbox_inches='tight')
    plt.close(fig)
    print('wrote fig_point_cloud_map.png and fig_voxel_map.png')


if __name__ == '__main__':
    main()
