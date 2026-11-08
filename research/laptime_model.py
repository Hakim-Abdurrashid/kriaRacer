"""Lap-time model on the team's blaze_track map: centerline vs min-curvature raceline, friction-limited speed profile."""
import sys, math
import numpy as np, cv2

MAP_PNG = sys.argv[1]; OUT = sys.argv[2]
RES = 0.05
img = cv2.imread(MAP_PNG, cv2.IMREAD_GRAYSCALE)
H, W = img.shape
free = (img >= 250).astype(np.uint8)
free = cv2.morphologyEx(free, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
n, lab, st, _ = cv2.connectedComponentsWithStats(free, connectivity=4)
track = (lab == 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])).astype(np.uint8)
inv = (1 - track).astype(np.uint8)
n2, lab2, st2, _ = cv2.connectedComponentsWithStats(inv, connectivity=4)
border = set(np.unique(np.concatenate([lab2[0], lab2[-1], lab2[:, 0], lab2[:, -1]]))) - {0}
holes = [l for l in range(1, n2) if l not in border]
island_lab = max(holes, key=lambda l: st2[l, cv2.CC_STAT_AREA])
island = (lab2 == island_lab).astype(np.uint8)
outside = np.isin(lab2, list(border)).astype(np.uint8)
d_in = cv2.distanceTransform((1 - island).astype(np.uint8), cv2.DIST_L2, 5) * RES
d_out = cv2.distanceTransform((1 - outside).astype(np.uint8), cv2.DIST_L2, 5) * RES
inner_half = (((d_in - d_out) <= 0) & (outside == 0)) | (island == 1)
cnts, _ = cv2.findContours(inner_half.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
cl = max(cnts, key=cv2.contourArea)[:, 0, :].astype(float)          # pixel (col,row)

def resample_closed(pts, ds):
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)]); L = s[-1]
    t = np.arange(0, L, ds)
    x = np.interp(t, s, np.append(pts[:, 0], pts[0, 0])); y = np.interp(t, s, np.append(pts[:, 1], pts[0, 1]))
    return np.stack([x, y], 1), L

def smooth_closed(p, sigma_pts):
    k = int(3 * sigma_pts); w = np.exp(-0.5 * (np.arange(-k, k + 1) / sigma_pts) ** 2); w /= w.sum()
    pad = np.vstack([p[-k:], p, p[:k]])
    return np.stack([np.convolve(pad[:, i], w, mode='valid') for i in range(2)], 1)

def curvature_closed(p):
    d1 = (np.roll(p, -1, 0) - np.roll(p, 1, 0)) / 2; d2 = np.roll(p, -1, 0) - 2 * p + np.roll(p, 1, 0)
    return (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(np.linalg.norm(d1, axis=1) ** 3, 1e-9)

# centerline in metres (pixel units * RES), uniform 0.1 m spacing
clm = cl * RES; clm[:, 1] *= -1                                       # flip row axis -> y up
clm, L_cl = resample_closed(smooth_closed(clm, 3), 0.10)
clm = smooth_closed(clm, 4); clm, L_cl = resample_closed(clm, 0.10)
N = len(clm)
def sample(field, p):
    c = np.clip((p[:, 0] / RES).round().astype(int), 0, W - 1); r = np.clip((-p[:, 1] / RES).round().astype(int), 0, H - 1)
    return field[r, c]
w_in, w_out = sample(d_in, clm), sample(d_out, clm)
tang = np.roll(clm, -1, 0) - np.roll(clm, 1, 0); tang /= np.linalg.norm(tang, axis=1, keepdims=True)
nrm = np.stack([-tang[:, 1], tang[:, 0]], 1)                         # left normal
# which side is the island? test a point along +normal
probe = clm + 0.2 * nrm
left_is_island = np.mean(sample(d_in, probe) < w_in) > 0.5
w_left, w_right = (w_in, w_out) if left_is_island else (w_out, w_in)

# --- min-curvature QP (second-difference proxy), bounds keep car centre >= margin from walls
MARGIN = 0.30                                                         # 0.15 half-width + 0.15 safety
lb = -(w_right - MARGIN); ub = (w_left - MARGIN)
lb = np.minimum(lb, 0); ub = np.maximum(ub, 0)
Dm = np.zeros((N, N)); idx = np.arange(N)
Dm[idx, idx] = -2; Dm[idx, (idx + 1) % N] = 1; Dm[idx, (idx - 1) % N] = 1
DtD = Dm.T @ Dm
Hq = (nrm[:, 0][:, None] * DtD * nrm[:, 0][None, :]) + (nrm[:, 1][:, None] * DtD * nrm[:, 1][None, :]) + 1e-7 * np.eye(N)
fq = nrm[:, 0] * (DtD @ clm[:, 0]) + nrm[:, 1] * (DtD @ clm[:, 1])
alpha = np.zeros(N); fixed = np.zeros(N, bool)
for it in range(200):                                                 # primal active-set
    F = ~fixed
    rhs = -fq[F] - Hq[np.ix_(F, fixed)] @ alpha[fixed]
    alpha[F] = np.linalg.solve(Hq[np.ix_(F, F)], rhs)
    viol = F & ((alpha < lb - 1e-9) | (alpha > ub + 1e-9))
    g = Hq @ alpha + fq
    release = fixed & (((np.isclose(alpha, lb)) & (g < -1e-9)) | ((np.isclose(alpha, ub)) & (g > 1e-9)))
    if not viol.any() and not release.any(): break
    alpha = np.clip(alpha, lb, ub); fixed = (fixed | viol) & ~release
rl, L_rl = resample_closed(clm + alpha[:, None] * nrm, 0.10)
rl = smooth_closed(rl, 2); rl, L_rl = resample_closed(rl, 0.10)

def lap_time(p, mu, a_acc=4.0, a_brk=5.0, v_top=7.0, scale=1.0, g=9.81):
    k = np.abs(curvature_closed(p)); k = np.convolve(np.concatenate([k[-5:], k, k[:5]]), np.ones(11) / 11, 'valid')
    ds = np.linalg.norm(np.roll(p, -1, 0) - p, axis=1)
    alat = mu * g; v = np.minimum(v_top, np.sqrt(alat / np.maximum(k, 1e-6))) * scale
    vlim = v.copy(); M = len(v)
    for _ in range(3):                                                # periodic forward/backward passes w/ friction ellipse
        for i in range(2 * M):
            a, b = i % M, (i + 1) % M
            ay = v[a] ** 2 * k[a]; ax = min(a_acc, mu * g) * math.sqrt(max(0.0, 1 - (ay / alat) ** 2))
            v[b] = min(vlim[b], math.sqrt(v[a] ** 2 + 2 * ax * ds[a]))
        for i in range(2 * M, 0, -1):
            a, b = i % M, (i - 1) % M
            ay = v[a] ** 2 * k[a]; ax = min(a_brk, mu * g) * math.sqrt(max(0.0, 1 - (ay / alat) ** 2))
            v[b] = min(v[b], math.sqrt(v[a] ** 2 + 2 * ax * ds[b]))
    return float(np.sum(ds / np.maximum(0.5 * (v + np.roll(v, -1)), 0.05))), float(v.max()), float(np.mean(v))

print(f"track: centerline length {L_cl:.1f} m, raceline length {L_rl:.1f} m, width min/median {np.min(w_in+w_out):.2f}/{np.median(w_in+w_out):.2f} m")
print(f"tightest radius: centerline {1/np.max(np.abs(curvature_closed(clm))):.2f} m, raceline {1/np.max(np.abs(curvature_closed(rl))):.2f} m")
print(f"current FTG runs (~0.7 m/s mean measured in Oct-9 bags): lap ~{L_cl/0.7:.0f} s -> {int(60/(L_cl/0.7))} full laps / 60 s")
print("mu   | centerline 100%        | raceline 100%          | raceline 75% (h2h)")
for mu in (0.5, 0.7, 0.9):
    c = lap_time(clm, mu); r = lap_time(rl, mu); r75 = lap_time(rl, mu, scale=0.75)
    f = lambda t: f"{t[0]:5.2f}s ({int(60//t[0])} laps, vmax {t[1]:.1f})"
    print(f"{mu:.1f}  | {f(c)} | {f(r)} | {f(r75)}")
# debug image
vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR); vis = cv2.resize(vis, (W * 3, H * 3), interpolation=cv2.INTER_NEAREST)
def to_px(p): return np.stack([p[:, 0] / RES * 3, -p[:, 1] / RES * 3], 1).astype(np.int32)
cv2.polylines(vis, [to_px(clm)], True, (255, 160, 0), 1); cv2.polylines(vis, [to_px(rl)], True, (0, 0, 230), 2)
cv2.imwrite(OUT, vis)
