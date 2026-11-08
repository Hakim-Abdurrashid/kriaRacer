#!/usr/bin/env python3
"""Race-day raceline generator (RACE_PLAN.md §5.2, §6.2): map -> centerline -> min-curvature line -> speed profile.

usage: make_raceline.py MAP.yaml [--out DIR] [--margin 0.30] [--mu 0.6] [--v-max 7] [--a-acc 4] [--a-brk 5]

Needs only numpy, opencv and pyyaml (runs on the car in a few seconds). Writes to DIR (default: next to the map):
  raceline.csv, raceline_rev.csv       minimum-curvature line, both driving directions
  centerline.csv, centerline_rev.csv   track centerline (the conservative choice for first laps)
  raceline.png                         overlay for a quick visual check (green dot = s=0, arrow = direction)
CSV columns, map frame, ROS conventions (psi = atan2(dy, dx); kappa > 0 turns left):
  s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, w_left_m, w_right_m
vx is the 100% friction-limited profile; race_core multiplies it by its speed scaler.
s = 0 is the point closest to --start (default: the map origin, where slam_toolbox started = the start line).
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml


# ---------------------------------------------------------------------------------------------- map

def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    if not os.path.exists(img_path):          # maps copied from another machine keep an absolute path
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), os.path.basename(meta['image']))
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f'cannot read map image {img_path}')
    p = img.astype(np.float32) / 255.0
    occ_prob = p if int(meta.get('negate', 0)) else 1.0 - p
    free = occ_prob < float(meta.get('free_thresh', 0.25))
    return img, free, float(meta['resolution']), [float(v) for v in meta['origin'][:2]]


class Grid:
    """Pixel <-> map-frame conversion (map_server convention: origin = lower-left corner of the image)."""

    def __init__(self, shape, res, origin):
        self.H, self.W = shape
        self.res = res
        self.ox, self.oy = origin

    def to_xy(self, cr):                       # (col, row) float pixels -> (x, y) metres
        return np.stack([self.ox + (cr[:, 0] + 0.5) * self.res,
                         self.oy + (self.H - 1 - cr[:, 1] + 0.5) * self.res], 1)

    def to_cr(self, xy):
        return np.stack([(xy[:, 0] - self.ox) / self.res - 0.5,
                         self.H - 1 - ((xy[:, 1] - self.oy) / self.res - 0.5)], 1)

    def sample(self, field, xy):
        cr = np.rint(self.to_cr(xy)).astype(int)
        c = np.clip(cr[:, 0], 0, self.W - 1)
        r = np.clip(cr[:, 1], 0, self.H - 1)
        return field[r, c]


def extract_track(free, grid, start_xy):
    """Return (track mask, island mask, outside mask). The track must be a closed ring of free space."""
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    f = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_OPEN, k3)
    n, lab, st, _ = cv2.connectedComponentsWithStats(f, connectivity=4)
    if n < 2:
        sys.exit('no free space in the map')
    c, r = np.rint(grid.to_cr(np.array([start_xy])))[0].astype(int)
    k = lab[r, c] if 0 <= r < grid.H and 0 <= c < grid.W else 0
    if k == 0:                                 # start not on free space: take the largest free region
        k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    track = (lab == k).astype(np.uint8)
    inv = (1 - track).astype(np.uint8)
    n2, lab2, st2, _ = cv2.connectedComponentsWithStats(inv, connectivity=4)
    border = set(np.unique(np.concatenate([lab2[0], lab2[-1], lab2[:, 0], lab2[:, -1]]))) - {0}
    holes = [l for l in range(1, n2) if l not in border]
    if not holes:
        sys.exit('the free space is not a closed loop: drive the full lap while mapping so slam_toolbox closes it')
    island = (lab2 == max(holes, key=lambda l: st2[l, cv2.CC_STAT_AREA])).astype(np.uint8)
    outside = np.isin(lab2, list(border)).astype(np.uint8)
    return track, island, outside


# ------------------------------------------------------------------------------------- path helpers

def resample_closed(p, ds):
    seg = np.linalg.norm(np.diff(np.vstack([p, p[:1]]), axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1] - 0.5 * ds, ds)
    t = t * (s[-1] / (len(t) * ds))            # exact uniform spacing around the loop
    x = np.interp(t, s, np.append(p[:, 0], p[0, 0]))
    y = np.interp(t, s, np.append(p[:, 1], p[0, 1]))
    return np.stack([x, y], 1)


def smooth_closed(p, sigma_pts):
    k = max(1, int(3 * sigma_pts))
    w = np.exp(-0.5 * (np.arange(-k, k + 1) / sigma_pts) ** 2)
    w /= w.sum()
    pad = np.vstack([p[-k:], p, p[:k]])
    return np.stack([np.convolve(pad[:, i], w, mode='valid') for i in range(2)], 1)


def tangents(p):
    t = np.roll(p, -1, 0) - np.roll(p, 1, 0)
    return t / np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)


def curvature(p, k=2):
    """Signed curvature from the circle through points i-k, i, i+k (less noisy than 1-step differences)."""
    a, b, c = np.roll(p, k, 0), p, np.roll(p, -k, 0)
    ab, bc, ca = b - a, c - b, a - c
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ca, axis=1)
    return 2.0 * cross / np.maximum(den, 1e-12)


def signed_area(p):
    return 0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])


def widths(p, track, grid, max_w=5.0):
    """Free distance from each point to the track edge along the left and right normals (ray march)."""
    t = tangents(p)
    nrm = np.stack([-t[:, 1], t[:, 0]], 1)
    steps = np.arange(0, max_w, grid.res * 0.5)
    out = []
    for sgn in (1.0, -1.0):
        q = p[:, None, :] + sgn * steps[None, :, None] * nrm[:, None, :]
        inside = grid.sample(track, q.reshape(-1, 2)).reshape(len(p), len(steps)).astype(bool)
        first_out = np.argmin(inside, axis=1)          # index of the first step off the track
        first_out[inside.all(axis=1)] = len(steps) - 1
        out.append(steps[first_out])
    return out[0], out[1]                             # w_left, w_right


# ----------------------------------------------------------------------------------- optimisation

def qp_offsets(ref, nrm, lb, ub, weights):
    """min sum_i w_i * |second difference of (ref + alpha * nrm)|_i^2  s.t. lb <= alpha <= ub.

    Dense primal active-set QP (N is a few hundred points)."""
    N = len(ref)
    D = np.zeros((N, N))
    i = np.arange(N)
    D[i, i] = -2
    D[i, (i + 1) % N] = 1
    D[i, (i - 1) % N] = 1
    DtWD = D.T @ (weights[:, None] * D)
    Hq = nrm[:, 0][:, None] * DtWD * nrm[:, 0][None, :] + nrm[:, 1][:, None] * DtWD * nrm[:, 1][None, :]
    Hq += 1e-6 * np.eye(N)
    fq = nrm[:, 0] * (DtWD @ ref[:, 0]) + nrm[:, 1] * (DtWD @ ref[:, 1])
    alpha = np.zeros(N)
    fixed = np.zeros(N, bool)
    for _ in range(4 * N):
        F = ~fixed
        if F.any():
            rhs = -fq[F] - Hq[np.ix_(F, fixed)] @ alpha[fixed]
            alpha[F] = np.linalg.solve(Hq[np.ix_(F, F)], rhs)
        viol = F & ((alpha < lb - 1e-9) | (alpha > ub + 1e-9))
        g = Hq @ alpha + fq
        release = fixed & (((np.isclose(alpha, lb)) & (g < -1e-9)) | ((np.isclose(alpha, ub)) & (g > 1e-9)))
        if not viol.any() and not release.any():
            break
        alpha = np.clip(alpha, lb, ub)
        fixed = (fixed | viol) & ~release
    return np.clip(alpha, lb, ub)


def min_curvature(ref, track, grid, margin, kappa_max=None, ds=0.2, step=0.10, iters=25):
    """Minimum-curvature line by sequential QPs on the normal offsets (second-difference proxy).

    Each iteration resamples the current line uniformly (the proxy only measures curvature at uniform
    spacing; one big solve bunches points on the inside of tight turns and creates cusps), recomputes
    normals and wall distances, and moves each point at most `step` along its normal, keeping the car
    centre `margin` from the walls. If kappa_max is given, points that turn tighter than the car can
    steer (tan(max_steer) / wheelbase) get more weight in the next solve.
    """
    path = resample_closed(ref, ds)
    w = None
    for _ in range(iters):
        path = resample_closed(path, ds)
        if w is None or len(w) != len(path):
            w = np.ones(len(path))
        wl, wr = widths(path, track, grid)
        t = tangents(path)
        nrm = np.stack([-t[:, 1], t[:, 0]], 1)
        lb = np.clip(-(wr - margin), -step, 0.0)
        ub = np.clip(wl - margin, 0.0, step)
        alpha = qp_offsets(path, nrm, lb, ub, w)
        path = path + alpha[:, None] * nrm
        if kappa_max is not None:
            excess = np.abs(curvature(resample_closed(path, ds), 1)) / kappa_max - 1.0
            if len(excess) == len(w) and excess.max() > 0:
                bump = np.convolve(np.concatenate([excess[-3:], excess, excess[:3]]).clip(0),
                                   np.ones(7) / 7, 'valid')
                w = np.minimum(w * (1.0 + 3.0 * bump), 1e4)
        if np.abs(alpha).max() < 0.003:
            break
    return path


def enforce_clearance(p, edt, grid, margin, iters=25, tol=0.005):
    """Push stretches closer than `margin` (Euclidean, so sharp wall corners count) back along normals.

    Each point moves towards whichever side has more room; the shift field is smoothed along the line
    with a peak-preserving bump so the fix never adds kinks."""
    for _ in range(iters):
        clear = grid.sample(edt, p)
        short = margin - clear
        if short.max() <= tol:
            break
        short = 1.5 * short + 0.01                    # overshoot a little: the smoothing gives some back
        t = tangents(p)
        nrm = np.stack([-t[:, 1], t[:, 0]], 1)
        side = np.sign(grid.sample(edt, p + 0.05 * nrm) - grid.sample(edt, p - 0.05 * nrm))
        shift = np.where(short > 0, short * side, 0.0)
        k = 9
        g = np.exp(-0.5 * (np.arange(-k, k + 1) / 3.0) ** 2)
        g /= g.max()
        pad = np.concatenate([shift[-k:], shift, shift[:k]])
        pos = np.array([np.max(pad[i:i + 2 * k + 1].clip(0) * g) for i in range(len(shift))])
        neg = np.array([np.min(pad[i:i + 2 * k + 1].clip(None, 0) * g) for i in range(len(shift))])
        p = smooth_closed(p + (pos + neg)[:, None] * nrm, 1)
    return p


def speed_profile(p, mu, a_acc, a_brk, v_max, g=9.81):
    """Friction-ellipse limited forward/backward pass on a closed loop. Returns (v, ax, kappa)."""
    kap = curvature(p)
    ka = np.abs(kap)
    ka = np.convolve(np.concatenate([ka[-5:], ka, ka[:5]]), np.ones(11) / 11, 'valid')
    ds = np.linalg.norm(np.roll(p, -1, 0) - p, axis=1)
    alat = mu * g
    vlim = np.minimum(v_max, np.sqrt(alat / np.maximum(ka, 1e-6)))
    v = vlim.copy()
    M = len(v)
    for _ in range(3):
        for i in range(2 * M):
            a, b = i % M, (i + 1) % M
            ay = v[a] ** 2 * ka[a]
            ax = min(a_acc, alat) * math.sqrt(max(0.0, 1 - (ay / alat) ** 2))
            v[b] = min(vlim[b], math.sqrt(v[a] ** 2 + 2 * ax * ds[a]))
        for i in range(2 * M, 0, -1):
            a, b = i % M, (i - 1) % M
            ay = v[a] ** 2 * ka[a]
            ax = min(a_brk, alat) * math.sqrt(max(0.0, 1 - (ay / alat) ** 2))
            v[b] = min(v[b], math.sqrt(v[a] ** 2 + 2 * ax * ds[b]))
    acc = (np.roll(v, -1) ** 2 - v ** 2) / (2 * np.maximum(ds, 1e-6))
    return v, acc, kap


def lap_time(p, v):
    ds = np.linalg.norm(np.roll(p, -1, 0) - p, axis=1)
    return float(np.sum(ds / np.maximum(0.5 * (v + np.roll(v, -1)), 0.05)))


# ----------------------------------------------------------------------------------------- output

def make_line(p, start_xy, track, grid, args, edt=None):
    """Rotate so s=0 is nearest start_xy, then compute everything the CSV needs."""
    i0 = int(np.argmin(np.linalg.norm(p - np.asarray(start_xy), axis=1)))
    p = np.roll(p, -i0, axis=0)
    v, ax, kap = speed_profile(p, args.mu, args.a_acc, args.a_brk, args.v_max)
    t = tangents(p)
    psi = np.arctan2(t[:, 1], t[:, 0])
    wl, wr = widths(p, track, grid)
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    clear = grid.sample(edt, p) if edt is not None else np.minimum(wl, wr)
    return dict(s=s, x=p[:, 0], y=p[:, 1], psi=psi, kappa=kap, v=v, ax=ax, wl=wl, wr=wr, clear=clear,
                length=s[-1] + np.linalg.norm(p[0] - p[-1]), lap=lap_time(p, v))


def write_csv(path, L):
    cols = np.stack([L['s'], L['x'], L['y'], L['psi'], L['kappa'], L['v'], L['ax'], L['wl'], L['wr']], 1)
    np.savetxt(path, cols, fmt='%.4f', delimiter=',',
               header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,w_left_m,w_right_m')


def draw(img, grid, lines, path, scale=3):
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    vis = cv2.resize(vis, (grid.W * scale, grid.H * scale), interpolation=cv2.INTER_NEAREST)

    def px(xy):
        return np.rint((grid.to_cr(xy) + 0.5) * scale).astype(np.int32)

    cl, rl = lines
    cv2.polylines(vis, [px(np.stack([cl['x'], cl['y']], 1))], True, (255, 160, 0), 1)
    pts = px(np.stack([rl['x'], rl['y']], 1))
    vmax = max(rl['v'].max(), 1e-3)
    for i in range(len(pts)):                  # raceline coloured by speed: red slow -> green fast
        f = rl['v'][i] / vmax
        cv2.line(vis, tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]), (0, int(255 * f), int(255 * (1 - f))), 2)
    cv2.circle(vis, tuple(pts[0]), 6, (0, 200, 0), -1)
    j = min(len(pts) - 1, 8)
    cv2.arrowedLine(vis, tuple(pts[0]), tuple(pts[j]), (0, 120, 0), 2, tipLength=0.5)
    cv2.imwrite(path, vis)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('--out', help='output directory (default: the map directory)')
    ap.add_argument('--margin', type=float, default=0.35,
                    help='min distance car centre - wall [m]; 0.15 half width + 0.20 for tracking error')
    ap.add_argument('--mu', type=float, default=0.6, help='friction coefficient for the speed profile')
    ap.add_argument('--a-acc', type=float, default=4.0)
    ap.add_argument('--a-brk', type=float, default=5.0)
    ap.add_argument('--v-max', type=float, default=7.0)
    ap.add_argument('--ds', type=float, default=0.10, help='output spacing [m]')
    ap.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0], help='x y of the start line (map frame)')
    ap.add_argument('--max-steer', type=float, default=0.40, help='steering limit [rad] (measure it on the car)')
    ap.add_argument('--wheelbase', type=float, default=0.33)
    args = ap.parse_args()
    out = args.out or os.path.dirname(os.path.abspath(args.map_yaml))
    os.makedirs(out, exist_ok=True)

    img, free, res, origin = load_map(args.map_yaml)
    grid = Grid(img.shape, res, origin)
    track, island, outside = extract_track(free, grid, args.start)

    # centerline = the set of points equidistant from the island and the outside walls
    d_in = cv2.distanceTransform((1 - island).astype(np.uint8), cv2.DIST_L2, 5)
    d_out = cv2.distanceTransform((1 - outside).astype(np.uint8), cv2.DIST_L2, 5)
    inner_half = ((((d_in - d_out) <= 0) & (outside == 0)) | (island == 1)).astype(np.uint8)
    cnts, _ = cv2.findContours(inner_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cl = grid.to_xy(max(cnts, key=cv2.contourArea)[:, 0, :].astype(float))
    cl = resample_closed(smooth_closed(cl, 3), args.ds)
    cl = resample_closed(smooth_closed(cl, 4), args.ds)
    if signed_area(cl) < 0:                    # base direction: counter-clockwise
        cl = cl[::-1].copy()

    # minimum-curvature line, optimised on a coarser 0.2 m grid then resampled
    wl, wr = widths(cl, track, grid)
    if np.min(wl + wr) < 2 * args.margin:
        print(f'WARNING: narrowest section {np.min(wl + wr):.2f} m is less than 2 x margin; line hugs the centre there')
    # Euclidean clearance to the nearest non-track cell; the optimiser works inside the track eroded by the
    # margin, so wall corners (island tips) are respected, not just the distance along the normals
    edt = cv2.distanceTransform(track, cv2.DIST_L2, 5) * res
    kappa_max = 0.9 * math.tan(args.max_steer) / args.wheelbase    # 10% steering reserve for tracking
    rl = min_curvature(cl, track, grid, args.margin + 0.03, kappa_max)   # +3 cm for the smoothing below
    rl = resample_closed(smooth_closed(resample_closed(rl, args.ds), 2), args.ds)
    rl = resample_closed(enforce_clearance(rl, edt, grid, args.margin), args.ds)
    if np.min(grid.sample(track, rl)) == 0:
        print('WARNING: the raceline leaves the free space somewhere - check raceline.png')
    k_rl = np.abs(curvature(rl)).max()
    if k_rl > math.tan(args.max_steer) / args.wheelbase:
        print(f'WARNING: tightest raceline radius {1 / k_rl:.2f} m needs {math.atan(args.wheelbase * k_rl):.2f} rad '
              f'of steering > max {args.max_steer:.2f}: the car cannot follow it there (track too tight?)')

    lines = {}
    for name, p in (('centerline', cl), ('raceline', rl)):
        for suffix, q in (('', p), ('_rev', p[::-1].copy())):
            L = make_line(q, args.start, track, grid, args, edt)
            write_csv(os.path.join(out, f'{name}{suffix}.csv'), L)
            lines[name + suffix] = L
    draw(img, grid, (lines['centerline'], lines['raceline']), os.path.join(out, 'raceline.png'))

    cw = lines['centerline']
    print(f'track: centerline {cw["length"]:.1f} m, width min/median {np.min(cw["wl"] + cw["wr"]):.2f}/'
          f'{np.median(cw["wl"] + cw["wr"]):.2f} m, tightest radius {1 / np.max(np.abs(cw["kappa"])):.2f} m')
    for k in ('centerline', 'centerline_rev', 'raceline', 'raceline_rev'):
        L = lines[k]
        print(f'{k:15s} {L["length"]:5.1f} m  lap {L["lap"]:5.2f} s at 100% (mu {args.mu}), '
              f'{lap_time(np.stack([L["x"], L["y"]], 1), 0.75 * L["v"]):5.2f} s at 75%, '
              f'v {L["v"].min():.1f}-{L["v"].max():.1f} m/s, min wall clearance {L["clear"].min():.2f} m, '
              f'tightest radius {1 / np.abs(L["kappa"]).max():.2f} m')
    print('wrote', out)


if __name__ == '__main__':
    main()
