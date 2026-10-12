#!/usr/bin/env python3
"""ICP registration and overlap between the saved SLAM voxel map and the Gazebo
ground-truth SDF model of the two-floor building.

Inputs (all read-only, existing project artefacts):
  start/maps/inspection_building.dmap          SLAM occupancy archive (DRONE_GRID_V2, odom frame)
  src/drone_stack/worlds/inspection_building.world   Gazebo ground truth (all-box SDF)
  reports/two_floor_patrol_20261009_153401/metadata.json  map->world transform

Method
  1. Occupied voxel centres -> map frame (same transform map_session.read_preview uses).
  2. map frame -> world frame via world_from_map (translation only, yaw 0) from metadata.
  3. Ground truth surface cloud: every <collision><box> sampled on all six faces.
  4. Point-to-point ICP (SVD closed form) refines the map->world transform.
  5. Overlap = fraction of mapped surface points whose nearest ground-truth surface
     distance is <= tau, for tau in {0.05, 0.10, 0.15, 0.20} m; reported before and
     after ICP, plus the reverse direction (ground truth covered by the map).
"""
from pathlib import Path
import json
import math
import numpy as np
from scipy.spatial import cKDTree

PROJECT = Path(__file__).resolve().parents[2]
DMAP = PROJECT/'start/maps/inspection_building.dmap'
WORLD = PROJECT/'src/drone_stack/worlds/inspection_building.world'
META = PROJECT/'reports/two_floor_patrol_20261009_153401/metadata.json'
OUT = Path(__file__).resolve().parent
TAUS = (0.05, 0.10, 0.15, 0.20)
SURFACE_SAMPLE_M = 0.04   # ground-truth face sampling used only for ICP correspondences


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def load_map_points():
    """Map-frame occupied voxel centres (matches map_session.read_preview)."""
    with DMAP.open() as f:
        h = f.readline().split()
        assert h[0] == 'DRONE_GRID_V2' and h[1] == 'odom', h[:2]
        res = float(h[2])
        origin = np.array(list(map(float, h[3:6])))
        dims = np.array(list(map(int, h[6:9])))
        count = int(h[9])
        saved = np.array(list(map(float, h[10:13])))
        saved_yaw = math.radians(float(h[13]))
        occupied = []
        for _ in range(count):
            address, evidence, free = map(int, f.readline().split())
            if evidence:
                occupied.append(address)
    addresses = np.asarray(occupied, dtype=np.int64)
    idx = np.column_stack((addresses//(dims[1]*dims[2]),
                           addresses//dims[2] % dims[1],
                           addresses % dims[2]))
    return (origin+(idx+.5)*res) @ rotation(saved_yaw).T + saved, res


def euler_matrix(r, p, y):
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_ground_truth():
    """All oriented boxes from the SDF, sampled on their faces."""
    import xml.etree.ElementTree as ET
    world = ET.parse(WORLD).getroot().find('world')
    boxes = []
    for model in world.findall('model'):
        for link in model.findall('link'):
            for col in link.findall('collision'):
                size = col.findtext('geometry/box/size')
                if not size:
                    continue
                pose = list(map(float, col.findtext('pose', '0 0 0 0 0 0').split()))
                lpose = list(map(float, link.findtext('pose', '0 0 0 0 0 0').split()))
                mpose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
                c = np.array(pose[:3]) + np.array(lpose[:3]) + np.array(mpose[:3])
                R = euler_matrix(*pose[3:]) @ euler_matrix(*lpose[3:]) @ euler_matrix(*mpose[3:])
                boxes.append((link.get('name'), c, R, np.array(list(map(float, size.split())))))
    pts = []
    for _, c, R, size in boxes:
        half = size/2.
        n = np.maximum(2, np.ceil(size/SURFACE_SAMPLE_M).astype(int))
        for axis in range(3):
            u = np.linspace(-half[axis], half[axis], n[axis])
            other = [a for a in range(3) if a != axis]
            g0 = np.linspace(-half[other[0]], half[other[0]], n[other[0]])
            g1 = np.linspace(-half[other[1]], half[other[1]], n[other[1]])
            G0, G1 = np.meshgrid(g0, g1, indexing='ij')
            for sign in (-1, 1):
                loc = np.zeros((G0.size, 3))
                loc[:, axis] = sign*half[axis]
                loc[:, other[0]] = G0.ravel()
                loc[:, other[1]] = G1.ravel()
                pts.append(loc @ R.T + c)
    return np.concatenate(pts), boxes


def icp(src, dst_tree, dst, iters=60, sample=40000, trim=0.8, max_corr=0.12, seed=0):
    """Trimmed point-to-point ICP, closed-form SVD update.

    Robustness matters here: the SLAM map only observes part of the ground-truth
    surface (no ceiling, occluded regions), so a plain least-squares fit is dragged
    off by the un-modelled points and diverges. Keeping the best `trim` fraction of
    correspondences within `max_corr` gives a stable refinement from the project's
    own map alignment.
    """
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(src), size=min(sample, len(src)), replace=False)
    P = src[idx].astype(np.float64)
    R = np.eye(3)
    t = np.zeros(3)
    best = (np.inf, R.copy(), t.copy())
    history = []
    stale = 0
    for _ in range(iters):
        Q = P @ R.T + t
        d, nn = dst_tree.query(Q, k=1, workers=1)
        keep = d <= max_corr
        if keep.sum() < 100:
            break
        dk, nk = d[keep], nn[keep]
        thr = np.quantile(dk, trim)
        sel = dk <= thr
        Pk, Tk = Q[keep][sel], dst[nk][sel]
        pc, qc = Pk.mean(0), Tk.mean(0)
        H = (Pk-pc).T @ (Tk-qc)
        U, _, Vt = np.linalg.svd(H)
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        dR = Vt.T @ D @ U.T
        dt = qc - dR @ pc
        R = dR @ R
        t = dR @ t + dt
        # P remains the fixed original source: applying both an in-place
        # update to P and accumulated R/t would transform it twice next time.
        updated, _ = dst_tree.query(P @ R.T+t, k=1, workers=1)
        selected = np.sort(updated[updated <= max_corr])
        score = float(np.sqrt(np.mean(selected[:max(1,int(len(selected)*trim))]**2)))
        history.append(score)
        if score < best[0]-1e-6:
            best = (score, R.copy(), t.copy())
            stale = 0
        else:
            stale += 1
            if stale >= 3:      # refinement has stopped paying off
                break
        if score < 1e-5:
            break
    return best[1], best[2], history


def stats(d):
    return {'mean': float(d.mean()), 'median': float(np.median(d)),
            'rmse': float(np.sqrt(np.mean(d**2))), 'p95': float(np.percentile(d, 95)),
            'max': float(d.max())}


def distance_to_boxes(points, boxes, chunk=20000):
    """Exact point-to-surface distance to every oriented box, minimised over boxes.

    Outside a box: ||max(q,0)|| with q = |R^T(p-c)| - half.
    Inside a box:  min(half - |R^T(p-c)|), i.e. distance to the closest face.
    """
    cs = np.array([b[1] for b in boxes])
    Rs = np.array([b[2] for b in boxes])
    hs = np.array([b[3] for b in boxes])/2.
    out = np.empty(len(points))
    for start in range(0, len(points), chunk):
        block = points[start:start+chunk]
        # Subtract each box centre BEFORE rotating into its local frame.
        # The rotated staircase rails otherwise have incorrect distances.
        loc = np.abs(np.einsum('pbi,bij->pbj',block[:,None,:]-cs[None,:,:],Rs))
        q = loc - hs[None]
        outside = np.linalg.norm(np.maximum(q, 0.), axis=2)   # >0 when outside, 0 when inside
        inside = -q.max(axis=2)                               # >0 when inside, <0 when outside
        out[start:start+chunk] = np.maximum(outside, inside).min(axis=1)
    return out


def overlap(d, tau):
    return float(100.*np.mean(d <= tau))


def main():
    meta = json.loads(META.read_text())
    wmt = np.array(meta['world_from_map_translation'], float)
    wmy = math.radians(meta['world_from_map_yaw'])
    wmR = rotation(wmy)

    map_pts, res = load_map_points()
    gt_pts, boxes = load_ground_truth()

    # map frame -> world frame  (metadata world_from_map)
    world_pts = map_pts @ wmR.T + wmt

    gt_tree = cKDTree(gt_pts)
    result = {'map_occupied_voxels': int(len(map_pts)),
              'ground_truth_box_links': len(boxes),
              'ground_truth_surface_points': int(len(gt_pts)),
              'voxel_resolution_m': res,
              'world_from_map_translation': wmt.tolist(),
              'world_from_map_yaw_deg': meta['world_from_map_yaw'],
              'note': ('SLAM side is the saved 0.10 m occupancy archive (occupancy voxel centres), '
                       'not the raw Faster-LIO PCD; the raw cloud was not archived for this run.'),
              'before_icp': {}, 'after_icp': {}}

    # ---- before ICP: map -> nearest ground-truth surface (exact) --------
    d_before = distance_to_boxes(world_pts, boxes)
    result['before_icp'] = {'distance_m': stats(d_before),
                            'overlap_pct': {str(t): overlap(d_before, t) for t in TAUS}}

    # ---- ICP refinement (sampled correspondences, exact scoring) --------
    R_icp, t_icp, hist = icp(world_pts, gt_tree, gt_pts)
    refined = world_pts @ R_icp.T + t_icp
    d_after = distance_to_boxes(refined, boxes)
    result['after_icp'] = {'distance_m': stats(d_after),
                           'overlap_pct': {str(t): overlap(d_after, t) for t in TAUS},
                           'icp_correction_translation_m': t_icp.tolist(),
                           'icp_correction_translation_norm_m': float(np.linalg.norm(t_icp)),
                           'icp_correction_rotation_deg': float(math.degrees(
                               math.acos(np.clip((np.trace(R_icp)-1)/2, -1, 1)))),
                           'rmse_history': hist}

    # ---- reverse direction: ground truth covered by the map -------------
    map_tree = cKDTree(refined)
    gt_near, _ = map_tree.query(gt_pts, k=1, workers=1)
    result['ground_truth_covered_by_map'] = {
        'distance_m': stats(gt_near),
        'coverage_pct': {str(t): overlap(gt_near, t) for t in TAUS}}

    (OUT/'icp_overlap_result.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))

    # ---- figure ---------------------------------------------------------
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    fname = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
    plt.rcParams.update({'font.family': fname, 'axes.unicode_minus': False, 'font.size': 11,
                         'savefig.dpi': 220, 'axes.grid': True, 'grid.alpha': .2})
    fig, ax = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    ts = [str(t) for t in TAUS]
    b = [result['before_icp']['overlap_pct'][k] for k in ts]
    a = [result['after_icp']['overlap_pct'][k] for k in ts]
    x = np.arange(len(ts))
    ax[0].bar(x-.2, b, .4, label='ICP 配准前', color='#c98a3c')
    ax[0].bar(x+.2, a, .4, label='ICP 配准后', color='#2f7fb5')
    for xi, v in zip(x-.2, b):
        ax[0].text(xi, v+1, f'{v:.1f}', ha='center', fontsize=9)
    for xi, v in zip(x+.2, a):
        ax[0].text(xi, v+1, f'{v:.1f}', ha='center', fontsize=9)
    ax[0].set(xticks=x, xticklabels=[f'τ={t} m' for t in ts], ylim=(0, 108),
              ylabel='重合占比 / %', title='SLAM 占据体素 与 Gazebo 真值模型 的重合度')
    ax[0].legend()
    ax[1].plot(range(1, len(hist)+1), hist, 'o-', ms=3, color='#3b8f6e')
    ax[1].set(xlabel='ICP 迭代次数', ylabel='对应点 RMSE / m',
              title=f'ICP 收敛过程（第 {int(np.argmin(hist))+1} 次达到最小 {min(hist)*1000:.1f} mm，其后按早停准则拒绝）')
    fig.tight_layout()
    fig.savefig(OUT/'icp_overlap.png', bbox_inches='tight')
    plt.close(fig)

    # ---- overlay: SLAM voxels vs ground-truth boxes ---------------------
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    rng = np.random.default_rng(1)
    samp = refined[rng.choice(len(refined), size=min(60000, len(refined)), replace=False)]
    corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])*0.5
    faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    for name, c, R, size in boxes:
        v = (corners*size) @ R.T + c
        poly = Poly3DCollection([[v[i] for i in f] for f in faces], facecolors='#e2a13c',
                                edgecolors='#a8711f', linewidths=.15, alpha=.10)
        ax.add_collection3d(poly)
    ax.scatter(samp[:, 0], samp[:, 1], samp[:, 2], c=samp[:, 2], cmap='viridis', s=.4, alpha=.55)
    ax.set(xlim=(-32, 32), ylim=(-10, 24), zlim=(-1, 8),
           xlabel='世界 X / m', ylabel='世界 Y / m', zlabel='Z / m')
    ax.set_box_aspect((64, 34, 12)); ax.view_init(elev=40, azim=-66)
    ax.set_title('ICP 配准后：SLAM 占据体素（彩色）叠加于 Gazebo 真值模型（橙色线框）')
    fig.tight_layout()
    fig.savefig(OUT/'icp_registration_overlay.png', bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    main()
