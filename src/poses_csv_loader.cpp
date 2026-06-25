#include "cam_lidar_calib/poses_csv_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace cam_lidar_calib {

namespace {

// Parse "a,b,c" into a 3-vector. Throws on malformed line.
Eigen::Vector3d parseTriple(const std::string& line) {
    std::stringstream ss(line);
    double v[3];
    char comma;
    if (!(ss >> v[0] >> comma >> v[1] >> comma >> v[2]))
        throw std::runtime_error("poses.csv: malformed line: " + line);
    return Eigen::Vector3d(v[0], v[1], v[2]);
}

PlaneObservation buildObservation(const Eigen::Vector3d& centroid_mm,
                                  const Eigen::Vector3d& normal,
                                  const std::vector<Eigen::Vector3d>& corners_mm,
                                  SensorType sensor,
                                  int frame_idx) {
    const double MM_TO_M = 1e-3;
    const Eigen::Vector3d centroid_m = centroid_mm * MM_TO_M;

    std::vector<Eigen::Vector3d> corners_m;
    corners_m.reserve(corners_mm.size());
    for (const auto& c : corners_mm) corners_m.push_back(c * MM_TO_M);

    // Plane offset d = n · p_on_plane. Distance can be signed; the optimizer uses it
    // as-is so the sign convention is consistent across both sensors.
    const double d = normal.dot(centroid_m);

    return PlaneObservation(normal, d, sensor, corners_m, frame_idx);
}

}  // namespace

PosesCsvData loadPosesCsv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("poses.csv: cannot open " + path);

    std::vector<std::string> lines;
    std::string ln;
    while (std::getline(in, ln)) {
        if (!ln.empty() && ln.back() == '\r') ln.pop_back();
        if (ln.empty()) continue;
        lines.push_back(ln);
    }

    constexpr int LINES_PER_POSE = 19;
    if (lines.size() % LINES_PER_POSE != 0)
        throw std::runtime_error("poses.csv: line count " + std::to_string(lines.size())
                                 + " is not a multiple of 19");

    const size_t num_poses = lines.size() / LINES_PER_POSE;
    PosesCsvData out;
    out.camera.reserve(num_poses);
    out.lidar.reserve(num_poses);

    for (size_t p = 0; p < num_poses; ++p) {
        const size_t base = p * LINES_PER_POSE;
        // Layout (1-indexed in our earlier analysis, here 0-indexed):
        // 0 cam centroid, 1 cam normal, 2-5 cam corners,
        // 6 lid centroid, 7 lid normal, 8-11 lid corners,
        // 12-17 board metadata, 18 pose index.
        const Eigen::Vector3d cam_centroid = parseTriple(lines[base + 0]);
        const Eigen::Vector3d cam_normal   = parseTriple(lines[base + 1]);
        std::vector<Eigen::Vector3d> cam_corners;
        for (int i = 0; i < 4; ++i) cam_corners.push_back(parseTriple(lines[base + 2 + i]));

        const Eigen::Vector3d lid_centroid = parseTriple(lines[base + 6]);
        const Eigen::Vector3d lid_normal   = parseTriple(lines[base + 7]);
        std::vector<Eigen::Vector3d> lid_corners;
        for (int i = 0; i < 4; ++i) lid_corners.push_back(parseTriple(lines[base + 8 + i]));

        const int frame_idx = static_cast<int>(p);
        out.camera.push_back(buildObservation(cam_centroid, cam_normal, cam_corners,
                                              SensorType::CAMERA, frame_idx));
        out.lidar.push_back(buildObservation(lid_centroid, lid_normal, lid_corners,
                                             SensorType::LIDAR, frame_idx));
    }

    return out;
}

}  // namespace cam_lidar_calib
