#include "cam_lidar_calib/optimizer.hpp"

#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <stdexcept>

namespace cam_lidar_calib {

namespace {

// Encourage R * n_lidar ≈ n_camera (3 residuals).
struct NormalResidual {
    NormalResidual(const Eigen::Vector3d& n_cam, const Eigen::Vector3d& n_lid)
        : n_cam_(n_cam), n_lid_(n_lid) {}

    template <typename T>
    bool operator()(const T* const q_raw, const T* const /*t_raw*/, T* res) const {
        Eigen::Map<const Eigen::Quaternion<T>> q(q_raw);
        const Eigen::Matrix<T, 3, 1> n_lid_T(T(n_lid_.x()), T(n_lid_.y()), T(n_lid_.z()));
        const Eigen::Matrix<T, 3, 1> rotated = q * n_lid_T;
        res[0] = rotated.x() - T(n_cam_.x());
        res[1] = rotated.y() - T(n_cam_.y());
        res[2] = rotated.z() - T(n_cam_.z());
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& n_cam,
                                       const Eigen::Vector3d& n_lid) {
        return new ceres::AutoDiffCostFunction<NormalResidual, 3, 4, 3>(
            new NormalResidual(n_cam, n_lid));
    }

  private:
    Eigen::Vector3d n_cam_, n_lid_;
};

// For each LiDAR board corner: after applying (R, t), it must lie on the camera plane.
// Residual: n_cam · (R * p_lid + t) - d_cam   (1 residual).
struct PointOnPlaneResidual {
    PointOnPlaneResidual(const Eigen::Vector3d& n_cam, double d_cam,
                         const Eigen::Vector3d& p_lid)
        : n_cam_(n_cam), d_cam_(d_cam), p_lid_(p_lid) {}

    template <typename T>
    bool operator()(const T* const q_raw, const T* const t_raw, T* res) const {
        Eigen::Map<const Eigen::Quaternion<T>> q(q_raw);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(t_raw);
        const Eigen::Matrix<T, 3, 1> p_lid_T(T(p_lid_.x()), T(p_lid_.y()), T(p_lid_.z()));
        const Eigen::Matrix<T, 3, 1> p_cam_pred = q * p_lid_T + t;
        const Eigen::Matrix<T, 3, 1> n_cam_T(T(n_cam_.x()), T(n_cam_.y()), T(n_cam_.z()));
        res[0] = p_cam_pred.dot(n_cam_T) - T(d_cam_);
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& n_cam, double d_cam,
                                       const Eigen::Vector3d& p_lid) {
        return new ceres::AutoDiffCostFunction<PointOnPlaneResidual, 1, 4, 3>(
            new PointOnPlaneResidual(n_cam, d_cam, p_lid));
    }

  private:
    Eigen::Vector3d n_cam_;
    double d_cam_;
    Eigen::Vector3d p_lid_;
};

}  // namespace

CalibrationResult solveExtrinsic(const std::vector<PlaneObservation>& cam_obs,
                                 const std::vector<PlaneObservation>& lid_obs) {
    if (cam_obs.size() != lid_obs.size() || cam_obs.empty())
        throw std::runtime_error("solveExtrinsic: empty or mismatched observation vectors");

    // Eigen quaternion storage order is [x, y, z, w]; initialize to identity.
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    double t[3] = {0.0, 0.0, 0.0};

    ceres::Problem problem;
    problem.AddParameterBlock(q, 4, new ceres::EigenQuaternionManifold());
    problem.AddParameterBlock(t, 3);

    // Down-weight the normal residuals slightly so corner-distance terms (which carry
    // translation information) drive convergence. Both are unitless after normalisation.
    const double normal_weight = 1.0;

    for (size_t i = 0; i < cam_obs.size(); ++i) {
        const auto& c = cam_obs[i];
        const auto& l = lid_obs[i];

        problem.AddResidualBlock(
            NormalResidual::create(c.normal, l.normal),
            new ceres::ScaledLoss(nullptr, normal_weight, ceres::TAKE_OWNERSHIP),
            q, t);

        for (const auto& corner : l.points) {
            problem.AddResidualBlock(
                PointOnPlaneResidual::create(c.normal, c.distance, corner),
                nullptr, q, t);
        }
    }

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.minimizer_progress_to_stdout = false;
    opts.max_num_iterations = 200;
    opts.function_tolerance = 1e-10;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    Eigen::Map<const Eigen::Quaternion<double>> q_eig(q);
    Eigen::Quaterniond q_norm = q_eig.normalized();

    CalibrationResult result;
    result.R = q_norm.toRotationMatrix();
    result.t = Eigen::Vector3d(t[0], t[1], t[2]);
    result.poses = static_cast<int>(cam_obs.size());
    result.error = summary.final_cost;

    // Per-pose residual: distance of transformed LiDAR centroid from camera plane.
    result.residuals.reserve(cam_obs.size());
    for (size_t i = 0; i < cam_obs.size(); ++i) {
        const auto& c = cam_obs[i];
        const auto& l = lid_obs[i];
        Eigen::Vector3d centroid_lid = Eigen::Vector3d::Zero();
        for (const auto& p : l.points) centroid_lid += p;
        if (!l.points.empty()) centroid_lid /= static_cast<double>(l.points.size());
        const Eigen::Vector3d transformed = result.R * centroid_lid + result.t;
        result.residuals.push_back(std::abs(c.normal.dot(transformed) - c.distance));
    }

    std::cout << summary.BriefReport() << std::endl;
    return result;
}

}  // namespace cam_lidar_calib
