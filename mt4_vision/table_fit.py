"""Table-plane pixel->robot fit from marker corners + arm touches.

Why this exists: for a pinhole camera viewing a plane, pixel<->plane is
EXACTLY a homography (8 DOF) -- the model family is right. But fitting all
8 DOF to just N marker-center touch points went badly wrong in practice: at
N=5 the two perspective parameters are barely observable, and least squares
explained mm-level touch noise by planting the camera's horizon line INSIDE
the workspace (denominator w = h20*px + h21*py + h22 crossing zero between
the markers) -- a geometrically impossible camera, exact at the data points
and divergent everywhere else.

The fix is to source each part of the model from data rich enough to
determine it:

  1. Perspective (the hard 8-DOF part) comes from the marker CORNERS: every
     marker is a printed square of identical physical size, so 5 markers
     give 20 subpixel corner points with strong, well-spread geometric
     constraints. A small bundle adjustment jointly fits one homography
     (pixels -> a metric table frame) plus each marker's unknown pose in
     that frame. No arm involvement; accuracy limited only by print quality
     and lens distortion (watch the corner RMS).

  2. Alignment (metric table frame -> robot frame) comes from the arm
     touches: a 4-DOF similarity fit from the touched marker centers.
     N touches give 2N equations for 4 unknowns -- heavily overdetermined,
     so touch noise averages instead of leaking into perspective, and a
     single bad touch shows up as an outlier residual instead of silently
     warping the map.

The composed result (similarity . bundle homography) is still a single 3x3
homography, so it drops into Calibration.homography with no downstream
changes. fit_table_map() falls back to the plain affine fit when corners
are unavailable, and refuses any solution whose denominator changes sign
over the workspace (the physical-camera check the naive fit failed).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from mt4_vision.calib import CalibrationError, fit_affine


@dataclass
class TableFitReport:
    kind: str  # "bundle+similarity" or "affine"
    corner_rms_px: float | None = None
    corner_rms_mm: float | None = None
    touch_residuals_mm: dict[int, float] = field(default_factory=dict)
    touch_loo_mm: dict[int, float] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    # The pixel -> metric-plane bundle homography (perspective part only),
    # for storage in Calibration.bundle_homography and reuse by low-DOF
    # refits like the probe-based cube-top similarity.
    bundle_h: list[list[float]] | None = None


def _apply_h(h: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 3x3 homography to Nx2 points."""
    ones = np.ones((pts.shape[0], 1))
    v = (h @ np.hstack([pts, ones]).T).T
    return v[:, :2] / v[:, 2:3]


def fit_similarity_2d(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Least-squares rotation+uniform-scale+translation (no reflection)
    mapping src (Nx2) onto dst (Nx2), as a 3x3 matrix. Umeyama's method."""
    n = src.shape[0]
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    s0, d0 = src - ms, dst - md
    cov = d0.T @ s0 / n
    u, sv, vt = np.linalg.svd(cov)
    d = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])
    r = u @ d @ vt
    var_src = (s0 * s0).sum() / n
    scale = float(np.trace(np.diag(sv) @ d) / var_src)
    t = md - scale * (r @ ms)
    out = np.eye(3)
    out[:2, :2] = scale * r
    out[:2, 2] = t
    return out


def _denominator_sign_ok(h: np.ndarray, pixel_pts: np.ndarray, margin: float = 0.3) -> bool:
    """The physical-camera check: over the pixel bounding box of the data
    (plus margin), the homography's denominator must not change sign -- a
    sign change means the fitted horizon line crosses the workspace, which
    no real camera looking at the table can produce."""
    lo = pixel_pts.min(axis=0)
    hi = pixel_pts.max(axis=0)
    span = hi - lo
    lo -= margin * span
    hi += margin * span
    xs = np.linspace(lo[0], hi[0], 12)
    ys = np.linspace(lo[1], hi[1], 12)
    gx, gy = np.meshgrid(xs, ys)
    w = h[2, 0] * gx + h[2, 1] * gy + h[2, 2]
    return bool((w > 0).all() or (w < 0).all())


def _levenberg_marquardt(residual_fn, x0: np.ndarray, iters: int = 60) -> np.ndarray:
    x = x0.copy()
    r = residual_fn(x)
    cost = float(r @ r)
    lam = 1e-3
    eps = 1e-6
    for _ in range(iters):
        # Numeric Jacobian: ~20 params x ~40 residuals, trivial to compute.
        jac = np.empty((r.size, x.size))
        for j in range(x.size):
            xp = x.copy()
            xp[j] += eps
            jac[:, j] = (residual_fn(xp) - r) / eps
        a = jac.T @ jac
        g = jac.T @ r
        stepped = False
        for _ in range(12):
            try:
                dx = np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), -g)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            rn = residual_fn(x + dx)
            cn = float(rn @ rn)
            if cn < cost:
                x = x + dx
                r, cost = rn, cn
                lam = max(lam * 0.3, 1e-10)
                stepped = True
                break
            lam *= 10
        if not stepped or float(np.abs(dx).max()) < 1e-10:
            break
    return x


def _bundle_fit_plane(
    corner_obs: list[np.ndarray], affine_init: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Jointly fit H (pixels -> metric table frame) + per-marker poses, with
    every marker a rigid square of one shared side length.

    corner_obs: per marker, 4x2 pixel corners in detector order (TL,TR,BR,BL
    relative to the marker's printed orientation).
    affine_init: 3x3 pixel->robot-ish affine used to initialize the metric
    frame (its scale/orientation become the frame's gauge; the later
    similarity to robot absorbs any init error in those).

    Returns (H_bundle, marker_centers_in_plane (Nx2), rms_px, rms_mm).
    Gauge fixing: marker 0's pose and the shared side length are held at
    their initialization -- without this, the plane frame's own similarity
    freedom would make the problem rank-deficient.
    """
    n = len(corner_obs)
    mapped = [_apply_h(affine_init, quad) for quad in corner_obs]

    # One shared handedness for the pixel->plane map: mirror the square
    # template if the map flips orientation (shoelace sign of the mapped
    # quads; template TL->TR->BR->BL is clockwise, i.e. negative).
    def shoelace(q: np.ndarray) -> float:
        return 0.5 * float(
            sum(
                q[k][0] * q[(k + 1) % 4][1] - q[(k + 1) % 4][0] * q[k][1]
                for k in range(4)
            )
        )

    flip = 1.0 if shoelace(mapped[0]) < 0 else -1.0
    template = np.array(
        [[-1.0, flip], [1.0, flip], [1.0, -flip], [-1.0, -flip]]
    )  # times side/2 below

    # Shared side length from the mapped edge lengths; FIXED (gauge).
    edges = []
    for q in mapped:
        for k in range(4):
            edges.append(float(np.linalg.norm(q[(k + 1) % 4] - q[k])))
    side = float(np.median(edges))
    tmpl = template * (side / 2.0)

    # Initial pose per marker via similarity template->mapped corners.
    poses0 = []
    for q in mapped:
        s = fit_similarity_2d(tmpl, q)
        theta = math.atan2(s[1, 0], s[0, 0])
        poses0.append((float(s[0, 2]), float(s[1, 2]), theta))

    # Parameter vector: 8 homography entries (h33=1) + poses of markers 1..n-1.
    h0 = affine_init.flatten()[:8].copy()
    x0 = np.concatenate([h0, np.array(poses0[1:]).flatten()])
    fixed_pose0 = poses0[0]

    all_px = np.vstack(corner_obs)

    def unpack(x: np.ndarray):
        h = np.append(x[:8], 1.0).reshape(3, 3)
        poses = [fixed_pose0]
        rest = x[8:].reshape(-1, 3)
        poses.extend((float(a), float(b), float(c)) for a, b, c in rest)
        return h, poses

    def residual_fn(x: np.ndarray) -> np.ndarray:
        h, poses = unpack(x)
        pred = _apply_h(h, all_px)
        res = []
        for i in range(n):
            tx, ty, th = poses[i]
            c, s = math.cos(th), math.sin(th)
            rot = np.array([[c, -s], [s, c]])
            model = (rot @ tmpl.T).T + np.array([tx, ty])
            res.append(pred[4 * i : 4 * i + 4] - model)
        return np.vstack(res).flatten()

    x = _levenberg_marquardt(residual_fn, x0)
    h, poses = unpack(x)
    res_mm = residual_fn(x).reshape(-1, 2)
    rms_mm = float(np.sqrt((res_mm**2).sum(axis=1).mean()))
    # Pixel-space RMS: divide by the local map scale (mm per px).
    scale_mm_per_px = side / float(
        np.median([np.linalg.norm(q[1] - q[0]) for q in corner_obs])
    )
    rms_px = rms_mm / scale_mm_per_px
    centers = np.array([[p[0], p[1]] for p in poses])
    return h, centers, rms_px, rms_mm


def fit_table_map(
    marker_corners: dict[int, list[list[float]]] | None,
    touch_px: dict[int, tuple[float, float]],
    touch_robot: dict[int, tuple[float, float]],
) -> tuple[list[list[float]], TableFitReport]:
    """Fit the table-plane pixel->robot map.

    marker_corners: marker_id -> 4x2 pixel corners (None/{} disables the
    bundle and falls back to affine-from-centers).
    touch_px / touch_robot: marker_id -> pixel center / TCP-touched robot XY,
    for the markers the arm reached. Keys of touch_* must match each other;
    corner markers may be a superset (extra markers still improve
    perspective even if never touched).
    """
    ids = sorted(touch_px)
    if sorted(touch_robot) != ids:
        raise CalibrationError("touch_px / touch_robot marker ids differ")
    if len(ids) < 3:
        raise CalibrationError(f"need >=3 touched markers, got {len(ids)}")
    centers_px = np.array([touch_px[i] for i in ids], dtype=np.float64)
    robots = np.array([touch_robot[i] for i in ids], dtype=np.float64)

    affine = np.array(fit_affine([tuple(p) for p in centers_px], [tuple(r) for r in robots]))
    report = TableFitReport(kind="affine")

    corner_ids = sorted(marker_corners) if marker_corners else []
    if len(corner_ids) >= 2:
        try:
            corner_obs = [np.array(marker_corners[i], dtype=np.float64) for i in corner_ids]
            h_bundle, plane_centers, rms_px, rms_mm = _bundle_fit_plane(corner_obs, affine)

            # Similarity plane->robot from the touched markers only.
            touched_rows = [corner_ids.index(i) for i in ids if i in corner_ids]
            touched_ids = [i for i in ids if i in corner_ids]
            if len(touched_ids) < 3:
                raise CalibrationError("fewer than 3 touched markers have corners")
            src = plane_centers[touched_rows]
            dst = np.array([touch_robot[i] for i in touched_ids])
            sim = fit_similarity_2d(src, dst)
            h_final = sim @ h_bundle
            h_final /= h_final[2, 2]

            all_px = np.vstack(corner_obs)
            if not _denominator_sign_ok(h_final, all_px):
                raise CalibrationError("bundle fit put the horizon in the workspace")

            fitted = _apply_h(sim, src)
            resid = {
                mid: float(np.linalg.norm(fitted[k] - dst[k]))
                for k, mid in enumerate(touched_ids)
            }
            loo = {}
            if len(touched_ids) >= 4:
                for k, mid in enumerate(touched_ids):
                    keep = [j for j in range(len(touched_ids)) if j != k]
                    s_k = fit_similarity_2d(src[keep], dst[keep])
                    pred = _apply_h(s_k, src[k : k + 1])[0]
                    loo[mid] = float(np.linalg.norm(pred - dst[k]))

            report = TableFitReport(
                kind="bundle+similarity",
                corner_rms_px=round(rms_px, 3),
                corner_rms_mm=round(rms_mm, 3),
                touch_residuals_mm={k: round(v, 2) for k, v in resid.items()},
                touch_loo_mm={k: round(v, 2) for k, v in loo.items()},
                bundle_h=h_bundle.tolist(),
            )
            return h_final.tolist(), report
        except (CalibrationError, np.linalg.LinAlgError) as exc:
            report.notes.append(f"bundle fit unavailable ({exc}); using affine")

    # Affine fallback: no perspective, but no horizon pathology either.
    if not _denominator_sign_ok(affine, centers_px):
        raise CalibrationError("affine fit failed the denominator check (?)")
    fitted = _apply_h(affine, centers_px)
    report.touch_residuals_mm = {
        mid: round(float(np.linalg.norm(fitted[k] - robots[k])), 2)
        for k, mid in enumerate(ids)
    }
    return affine.tolist(), report
