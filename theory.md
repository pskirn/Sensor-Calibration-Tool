# Camera–LiDAR Calibration: Complete Theory Reference

This document is a self-contained technical reference for the sensor calibration
platform. It covers every concept that drives the code — from how a camera forms
an image to how Ceres optimizer finds the rigid transform between sensor frames.

---

## Table of Contents

1. [The Calibration Problem](#1-the-calibration-problem)
2. [Camera Intrinsics](#2-camera-intrinsics)
   - 2.1 Pinhole Camera Model
   - 2.2 The Intrinsic Matrix K
   - 2.3 Lens Distortion
   - 2.4 Undistortion
3. [Intrinsic Calibration with a Checkerboard](#3-intrinsic-calibration-with-a-checkerboard)
   - 3.1 Corner Detection
   - 3.2 Zhang's Method (How calibrateCamera Works)
   - 3.3 Reprojection Error
   - 3.4 PnP — Recovering Board Pose at Runtime
4. [LiDAR Sensor Model](#4-lidar-sensor-model)
   - 4.1 How a Spinning LiDAR Works
   - 4.2 Point Cloud Structure
   - 4.3 ROI Filtering (PassThrough)
   - 4.4 RANSAC Plane Fitting
5. [Extrinsic Calibration](#5-extrinsic-calibration)
   - 5.1 The Rigid Body Transform
   - 5.2 Why Planes?
   - 5.3 The Plane Constraint Equations
   - 5.4 How Many Poses?
   - 5.5 The Cost Function
   - 5.6 Rotation Parametrisation
   - 5.7 Ceres Solver (Planned)
6. [Full Pipeline Walkthrough](#6-full-pipeline-walkthrough)
7. [Key Numbers to Know](#7-key-numbers-to-know)

---

## 1. The Calibration Problem

A robot that carries both a camera and a LiDAR sees the world through two
fundamentally different lenses:

| Sensor  | Output            | Coordinate space | What it measures           |
|---------|-------------------|-----------------|---------------------------|
| Camera  | 2-D image (pixels)| Image plane     | Intensity / colour per ray |
| LiDAR   | 3-D point cloud   | 3-D metric space| Range per laser pulse      |

To fuse information from both sensors — for example, to paint colour onto a
point cloud, or to detect an obstacle in 3-D and draw a bounding box on the
camera image — every point in the LiDAR frame must be mapped to its position
in the camera frame.

That mapping is a **rigid body transform** T (rotation R + translation t):

```
p_camera = R * p_lidar + t
```

Finding R and t is called **extrinsic calibration**.

But before you can solve for extrinsics, you need to know how the camera itself
works — its **intrinsic calibration**.

---

## 2. Camera Intrinsics

### 2.1 Pinhole Camera Model

The simplest useful model of a camera is the **pinhole model**: imagine that all
light from the scene passes through a single infinitesimally small hole (the
*optical centre*) and lands on an image plane on the other side.

A 3-D scene point P = (X, Y, Z) expressed in the **camera coordinate frame**
(origin at the optical centre, Z pointing forward along the optical axis)
projects to the 2-D image point p = (u, v) by the rule:

```
u = fx * (X / Z) + cx
v = fy * (Y / Z) + cy
```

In matrix form (homogeneous coordinates):

```
[u]   [fx  0  cx] [X]
[v] = [ 0 fy  cy] [Y]  * (1/Z)
[1]   [ 0  0   1] [Z]
```

The 3×3 matrix in the middle is called the **camera intrinsic matrix** K.

### 2.2 The Intrinsic Matrix K

```
K = | fx   0  cx |
    |  0  fy  cy |
    |  0   0   1 |
```

| Parameter | Meaning                                                        | Typical value (1080p camera) |
|-----------|----------------------------------------------------------------|------------------------------|
| fx        | Focal length in **pixels**, horizontal axis                    | 800 – 2000 px                |
| fy        | Focal length in **pixels**, vertical axis (fx ≈ fy for square pixels) | 800 – 2000 px          |
| cx        | Horizontal coordinate of the **principal point** (where the optical axis hits the sensor) | image_width / 2 ≈ 960 px |
| cy        | Vertical coordinate of the principal point                     | image_height / 2 ≈ 540 px   |

**fx in pixels vs. focal length in mm:**
A real lens has a physical focal length f_mm (e.g. 8 mm). The sensor has pixels
of physical size s_x and s_y (e.g. 4.65 µm). Then:

```
fx = f_mm / s_x          fy = f_mm / s_y
```

Calibration gives fx, fy directly, you don't need to know f_mm or the pixel size separately.

### 2.3 Lens Distortion

Real lenses are not perfect pinholes. They introduce geometric distortion straight lines in the world appear curved in the image.

**Radial distortion** (barrel or pincushion):

Points are displaced radially outward (barrel, k1 < 0) or inward (pincushion, k1 > 0) from the principal point. Modelled as a polynomial in r² where
r² = x² + y² (normalised coordinates, x = (u - cx)/fx):

```
x_dist = x * (1 + k1*r² + k2*r⁴ + k3*r⁶)
y_dist = y * (1 + k1*r² + k2*r⁴ + k3*r⁶)
```

**Tangential distortion** (decentering):

From the lens not being perfectly centred or parallel to the image sensor:

```
x_dist += 2*p1*x*y + p2*(r² + 2*x²)
y_dist += p1*(r² + 2*y²) + 2*p2*x*y
```

**Full distortion vector** stored by this project: [k1, k2, p1, p2, k3]

For a quality camera with a good lens, k1 is the dominant coefficient;
k2, k3, p1, p2 are often small corrections. Wide-angle lenses have large k1.

### 2.4 Undistortion

Before using any image for geometric computation (e.g. corner detection for calibration), you can undistort it:

```
cv::undistort(distorted_image, undistorted_image, K, dist_coeffs);
```

Or, equivalently, work with the distorted image but apply distortion correction
in the projection formula — which is what `solvePnP` does internally.

---

## 3. Intrinsic Calibration with a Checkerboard

### 3.1 Corner Detection

The calibration board is a flat checkerboard. We detect the **inner corners** the points where four squares meet.

```
cv::findChessboardCorners(image, board_size, corners)
```

This gives corners to roughly pixel accuracy. We then refine to **sub-pixel accuracy** using the intensity gradient in a small window around each corner:

```
cv::cornerSubPix(gray, corners, window_size, ...)
```

Sub-pixel accuracy is important: a 0.1 px reprojection error is only achievable if the input corners are located to 0.1 px precision.

### 3.2 Zhang's Method (How calibrateCamera Works)

Zhang (2000) showed that you can recover K from images of a planar target viewed from at least 3 different orientations.

**Step 1 — Homographies.** For each image, compute the homography H that maps the checkerboard plane (Z = 0 in board coordinates) to the image:

```
[u]       [X]
[v]  ≅  H [Y]     (≅ means "equal up to scale")
[1]       [1]
```

H is a 3×3 matrix with 8 degrees of freedom, estimated from the 2-D–2-D point correspondences.

**Step 2 — Constraints on K.** Each homography H = K [r1 r2 t] (where r1, r2 are the first two columns of the rotation matrix, and t is the translation). Since r1 and r2 are orthonormal columns of a rotation matrix:

```
r1^T r1 = 1,  r2^T r2 = 1,  r1^T r2 = 0
```

Each image gives 2 constraints on K. With N images you get 2N equations; K has 4 unknowns (fx, fy, cx, cy — assuming zero skew), so N ≥ 2 is the theoretical minimum, but N ≥ 10 in practice for numerical stability.

**Step 3 — Closed-form solve.** The equations are linear in the entries of B = K^{-T} K^{-1} (a 3×3 symmetric matrix). Solve for B via linear least squares, then recover K via Cholesky decomposition of B.

**Step 4 — Non-linear refinement.** The closed-form answer seeds a non-linear optimiser (Levenberg-Marquardt) that minimises the total **reprojection error**:

```
E = Σ_i Σ_j || m_ij - m̂(K, dist, R_i, t_i, M_j) ||²
```

where m_ij is the detected 2-D corner, and m̂ is its re-projection through the
full camera model (including distortion).

### 3.3 Reprojection Error

The RMS reprojection error is the most common quality metric for intrinsic calibration:

```
RMS = sqrt( (1/N) * Σ ||detected - reprojected||² )   [pixels]
```

| RMS (pixels) | Quality         |
|------------- |-----------------|
| < 0.3        | Excellent       |
| 0.3 – 0.7    | Good, usable    |
| 0.7 – 1.5    | Acceptable      |
| > 1.5        | Something wrong |

Common causes of high RMS: blurry images, too-few board orientations, board
detected at wrong scale, large lens distortion not being modelled.

### 3.4 PnP — Recovering Board Pose at Runtime

Given the calibrated K and a new image of the checkerboard, **Perspective-n-Point (PnP)** recovers the pose of the board relative to the camera specifically a rotation vector `rvec` and translation vector `tvec`:

```
cv::solvePnP(object_points, image_points, K, dist, rvec, tvec)
```

The rotation vector `rvec` uses the **Rodrigues representation**: the direction of the vector is the rotation axis, and its magnitude (in radians) is the rotation angle. Convert to a 3×3 matrix with `cv::Rodrigues(rvec, R_mat)`.

**Extracting the board plane normal:**

The checkerboard lies in the XY plane of the board frame (Z = 0). The board's Z-axis, expressed in camera coordinates, is the **normal to the board plane**:

```
n_camera = R_mat * [0, 0, 1]^T   =   third column of R_mat
```

This is exactly what `CameraDetector::detect()` computes and returns as the `normal` field of `PlaneObservation`.

---

## 4. LiDAR Sensor Model

### 4.1 How a Spinning LiDAR Works

A mechanical spinning LiDAR (Velodyne HDL-64E, Ouster OS1, etc.) has N laser
emitters arranged vertically at fixed elevation angles φ_1…φ_N (e.g. −15° to
+15° for a 16-beam LiDAR). The whole head spins horizontally, firing all lasers
at each azimuth angle θ.

For a return at range r, azimuth θ, elevation φ:

```
X = r · cos(φ) · cos(θ)
Y = r · cos(φ) · sin(θ)
Z = r · sin(φ)
```

One full rotation (360°) at e.g. 10 Hz with 16 beams × ~1800 azimuth steps =
~28 800 points per frame.

### 4.2 Point Cloud Structure

This project uses PCL's `pcl::PointXYZI`:

| Field | Type    | Description                           |
|-------|---------|---------------------------------------|
| x     | float32 | metres, LiDAR frame X                 |
| y     | float32 | metres, LiDAR frame Y                 |
| z     | float32 | metres, LiDAR frame Z                 |
| intensity | float32 | Return intensity (reflectivity)   |

Points are stored in a `.pcd` file (PCL's native binary or ASCII format).

### 4.3 ROI Filtering (PassThrough)

The raw cloud contains the entire scene. For calibration we need only the
checkerboard region. A **PassThrough filter** clips the cloud to an axis-aligned
box:

```
For each axis A ∈ {x, y, z}:
    keep point P only if: A_min ≤ P.A ≤ A_max
```

In PCL:

```cpp
pcl::PassThrough<pcl::PointXYZI> pass;
pass.setFilterFieldName("x");
pass.setFilterLimits(config_.x_min, config_.x_max);
pass.filter(*cloud);
// repeat for y, z
```

The ROI bounds come from `config/params.yaml` and must be tuned to the physical
setup (how far the board is from the LiDAR, its height above the ground, etc.).

### 4.4 RANSAC Plane Fitting

After ROI filtering, the dominant structure should be the flat checkerboard
backing. **RANSAC** (Random Sample Consensus, Fischler & Bolles 1981) fits a
plane model robustly even when ~50% of points are outliers (floor, edges of
the board, noise).

**The plane model:** a point P lies on the plane if:

```
a·X + b·Y + c·Z + d = 0       where (a,b,c) is the unit normal, d is offset
```

**RANSAC iterations:**

```
Repeat for max_iterations:
    1. Pick 3 random points from the cloud
    2. Fit a plane through them (unique, assuming non-collinear)
    3. Count inliers: points with |a·X + b·Y + c·Z + d| < ransac_threshold
    4. If inlier count > best so far → save this hypothesis

Final step: refit the plane with least squares using ALL inliers of best hypothesis
```

**Why RANSAC works:** even if only 20% of points are true inliers, the
probability that at least one sample of 3 is all-inlier is
1 - (1 - 0.2³)^N ≈ 1 after N = 200 iterations.

**Output:** PCL returns `coefficients->values = [a, b, c, d]`. Normalise (a,b,c)
to unit length to get the plane normal n_lidar. The `PlaneObservation` stores
this plus the filtered inlier points.

---

## 5. Extrinsic Calibration

### 5.1 The Rigid Body Transform

The transform from LiDAR frame to camera frame has **6 degrees of freedom**:
- 3 DOF rotation (roll, pitch, yaw — or equivalently an axis and angle)
- 3 DOF translation (x, y, z offset between sensor origins)

Represented as:

```
R  ∈ SO(3):  3×3 rotation matrix, R^T R = I, det(R) = +1
t  ∈ R³:     3×1 translation vector

p_camera = R · p_lidar + t
```

In homogeneous form (useful for chaining transforms):

```
[p_c]   [R  t] [p_l]
[ 1 ] = [0  1] [ 1 ]
```

### 5.2 Why Planes?

Direct point-to-point matching between camera and LiDAR is hard:
- Camera sees texture; LiDAR sees geometry — no obvious correspondence
- LiDAR resolution is much lower than camera (28 k vs 2 M points)
- Camera has no depth; LiDAR has no colour

Using a **flat calibration target (checkerboard board)** solves this:
- The camera detects the board plane precisely via corner-based PnP
- The LiDAR detects the same board plane via RANSAC
- Both measurements refer to the **same physical plane** — a natural correspondence

We don't need to match individual points; we match **plane descriptions**.

### 5.3 The Plane Constraint Equations

A plane can be described by a unit normal n̂ and a signed distance d from the
origin of a coordinate frame:

```
For the camera:   π_c = (n̂_c, d_c)    — normal and distance in camera frame
For the LiDAR:    π_l = (n̂_l, d_l)    — normal and distance in LiDAR frame
```

Since both refer to the same physical plane, the transform (R, t) must satisfy:

**Normal constraint** (normals are directions — they transform by R only):
```
n̂_c = R · n̂_l
```

**Distance constraint** (the plane passes through a point P_c = R·P_l + t; working through the algebra):
```
d_c = d_l + n̂_c · t
```

Each pose pair gives 4 scalar equations (3 from the normal constraint, 1 from
the distance constraint). The normal has unit magnitude, which removes one
degree of freedom, so effectively **3 independent constraints per pose**.

With 6 unknowns (R in 3 DOF + t in 3 DOF), you need at least **2 poses** in
theory, but at least **3 non-coplanar poses** in practice for a well-conditioned
solution.

### 5.4 How Many Poses?

**Minimum:** 3 poses where the board planes are not parallel to each other
(non-coplanar normals). With fewer, some DOF of the rotation remain unobserved.

**In practice:** 10–30 poses are typical. More poses → more robust estimate,
averaging out measurement noise.

**Good pose diversity:**
- Tilt the board in different directions (don't always keep it face-on)
- Move the board to different depths (closer / farther)
- Place the board in different parts of the overlapping field of view (left,
  right, high, low)
- Avoid all-parallel orientations (degenerate configuration)

### 5.5 The Cost Function

Given N pose pairs {(n̂_c^i, d_c^i), (n̂_l^i, d_l^i)} for i = 1…N, we
minimise:

```
E(R, t) = Σᵢ [ || R·n̂_l^i  −  n̂_c^i ||²   +   λ·(d_l^i + n̂_c^i·t − d_c^i)² ]
```

where λ ≈ 1 balances the two residual types (both are dimensionless or in
consistent units).

This is a **non-linear least squares** problem because R is constrained to
SO(3) (the manifold of proper rotation matrices).

### 5.6 Rotation Parametrisation

You cannot optimise a 3×3 matrix directly — that's 9 numbers with 6 constraints
(orthonormality of columns + positive determinant), making the unconstrained
gradient meaningless.

Two standard parametrisations for optimisation:

**Angle-axis (Rodrigues vector):**
```
ω = θ · k̂        where k̂ is the unit rotation axis, θ is the angle (radians)
```
3 unconstrained numbers. Convert to R with Rodrigues' formula:
```
R = I·cos(θ) + (1-cos(θ))·k̂k̂ᵀ + sin(θ)·[k̂]×
```
where [k̂]× is the skew-symmetric cross-product matrix of k̂.

**Unit quaternion:**
```
q = (qw, qx, qy, qz)    with qw² + qx² + qy² + qz² = 1
```
4 numbers with 1 constraint. Ceres handles the unit-norm manifold automatically
with `QuaternionManifold`. The CalibrationResult helper `getQuaternion()` returns
this representation.

**Euler angles (roll, pitch, yaw):** intuitive but suffer from gimbal lock and
are numerically poor for optimisation. Useful only for display — hence
`getEulerDegrees()` in CalibrationResult is for human reading, not computation.

### 5.7 Ceres Solver (Planned)

Ceres is Google's open-source non-linear least squares library. The calibration
cost functor would look like:

```cpp
struct PlanePairCost {
    const Eigen::Vector3d n_cam, n_lidar;
    const double d_cam, d_lidar;

    template<typename T>
    bool operator()(const T* const aa,   // angle-axis, 3 values
                    const T* const t,    // translation, 3 values
                    T* residual) const {
        // Rotate lidar normal by current angle-axis estimate
        T n_l[3] = { T(n_lidar[0]), T(n_lidar[1]), T(n_lidar[2]) };
        T n_rotated[3];
        ceres::AngleAxisRotatePoint(aa, n_l, n_rotated);

        // Normal residuals: R*n_lidar - n_cam  (3 values)
        residual[0] = n_rotated[0] - T(n_cam[0]);
        residual[1] = n_rotated[1] - T(n_cam[1]);
        residual[2] = n_rotated[2] - T(n_cam[2]);

        // Distance residual: d_lidar + n_cam . t - d_cam  (1 value)
        residual[3] = T(d_lidar)
                    + n_rotated[0]*t[0] + n_rotated[1]*t[1] + n_rotated[2]*t[2]
                    - T(d_cam);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& n_cam,
                                        double d_cam,
                                        const Eigen::Vector3d& n_lidar,
                                        double d_lidar) {
        // 4 residuals, 3 angle-axis params, 3 translation params
        return new ceres::AutoDiffCostFunction<PlanePairCost, 4, 3, 3>(
            new PlanePairCost{n_cam, n_lidar, d_cam, d_lidar});
    }
};

// Building and solving the problem
ceres::Problem problem;
double aa[3] = {0, 0, 0};   // initial guess: identity rotation
double t[3]  = {0, 0, 0};   // initial guess: no translation

for (const auto& pair : plane_pairs) {
    problem.AddResidualBlock(
        PlanePairCost::Create(pair.camera.normal, pair.camera.distance,
                               pair.lidar.normal,  pair.lidar.distance),
        nullptr, aa, t);
}

ceres::Solver::Options opts;
opts.minimizer_type = ceres::TRUST_REGION;
ceres::Solver::Summary summary;
ceres::Solve(opts, &problem, &summary);
```

After solving, convert `aa` back to R with `cv::Rodrigues` or
`ceres::AngleAxisToRotationMatrix`.

---

## 6. Full Pipeline Walkthrough

```
─────────────────────────────────────────────────────────────
PHASE 1: Intrinsics  (run once per camera setup)
  Tool: build/compute_intrinsics
─────────────────────────────────────────────────────────────

  Input folder of N images (N ≥ 10 recommended) of the
  checkerboard, captured from varied angles/distances.

  For each image:
    1. findChessboardCorners  →  rough 2-D corners
    2. cornerSubPix           →  sub-pixel refinement
    3. Generate 3-D board points (known geometry: rows×cols, square_size)

  calibrateCamera(all_3D_pts, all_2D_pts, image_size)
    → K (3×3), dist_coeffs [k1,k2,p1,p2,k3], RMS reprojection error
    → Writes config/camera.yaml

─────────────────────────────────────────────────────────────
PHASE 2: Extrinsic Calibration  (main pipeline)
  Tool: build/camera_lidar_calibration  (run from build/)
─────────────────────────────────────────────────────────────

  Load config/params.yaml  →  CalibrationConfig
  Load config/camera.yaml  →  CameraIntrinsics (K, dist)

  For each frame i (paired image + .pcd file):

    ── CameraDetector::detect(image_i) ──────────────────────
    | findChessboardCorners + cornerSubPix
    | solvePnP(board_3D, corners_2D, K, dist) → rvec, tvec
    | R_board_cam = Rodrigues(rvec)
    | n_cam  = R_board_cam[:, 2]   (board Z-axis in cam frame)
    | d_cam  = tvec · n_cam        (signed dist from cam origin)
    | returns PlaneObservation {n_cam, d_cam, inlier_pts, frame=i}
    ──────────────────────────────────────────────────────────

    ── LidarDetector::detectFromFile(path_i, i) ─────────────
    | pcl::io::loadPCDFile → raw cloud
    | PassThrough(x_min…z_max) → filtered cloud
    | SACSegmentation (RANSAC, plane model, threshold, iters)
    |   → coefficients [a, b, c, d]
    | n_lidar = normalise(a, b, c)
    | d_lidar = d / ||(a,b,c)||
    | returns PlaneObservation {n_lidar, d_lidar, inliers, frame=i}
    ──────────────────────────────────────────────────────────

    Append PlanePair{camera_obs, lidar_obs} to collection

  ── Optimizer (Ceres — not yet implemented) ───────────────
  | Build ceres::Problem from all PlanePairs
  | Minimise Σ ||R·n_l - n_c||² + (d_l + n_c·t - d_c)²
  | Solve → angle-axis aa, translation t
  | Convert aa → R matrix
  | returns CalibrationResult {R, t}
  ──────────────────────────────────────────────────────────

  Write results/calibration.yaml:
    R: [3×3 matrix]
    t: [tx, ty, tz]

  If output.generate_visualization = true:
    Project LiDAR inlier points onto image using K, R, t
    Save detected_frame_N.png to results/
```

---

## 7. Key Numbers to Know

| Quantity | Typical value | Where set |
|---|---|---|
| Calibration board corners | (cols-1) × (rows-1) inner corners | params.yaml board_cols/rows |
| Square size | 0.08 – 0.25 m | params.yaml square_size |
| Minimum calibration images (intrinsics) | 10 (varied angles) | — |
| RMS reprojection error (good) | < 0.5 px | printed by compute_intrinsics |
| RANSAC threshold (LiDAR) | 0.01 – 0.05 m | params.yaml ransac_threshold |
| RANSAC iterations | 100 – 1000 | params.yaml ransac_max_iterations |
| Minimum calibration poses (extrinsics) | 3 (non-coplanar) | — |
| Recommended calibration poses | 15 – 30 | — |
| Extrinsic RMS normal residual (good) | < 0.01 rad | printed by optimizer |
| Extrinsic RMS distance residual (good) | < 0.01 m | printed by optimizer |

---

## References

- Zhang, Z. (2000). A flexible new technique for camera calibration. IEEE TPAMI.
- Fischler, M. & Bolles, R. (1981). Random sample consensus. Communications ACM.
- Zhang, Q. & Pless, R. (2004). Extrinsic calibration of a camera and laser range finder. IROS.
- Geiger, A. et al. (2012). Automatic camera and range sensor calibration using a single shot. ICRA.
- Ceres Solver documentation: http://ceres-solver.org
- PCL documentation: https://pointclouds.org/documentation
