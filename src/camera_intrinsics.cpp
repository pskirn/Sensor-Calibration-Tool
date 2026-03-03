#include <opencv2/opencv2.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;


namespace cam_lidar_calib {

     //  Create 3D object points
     std::vector<cv::Point3f> generateObjectPoints(cv::Size board_size, float square_size) {

    std::vector<cv::Point3f> obj_pts;
    for (int i = 0; i < 3; ++i) 
        for (int j = 0; j < 3; ++j)
            obj_pts.emplace_back(j*square_size, i*square_size, 0.0f);
    
    return obj_pts;

    }

}


int main(int argc, char** argv) {

    if (argc < 3) {
        std::cout << "Usage: compute_intrinsics <images_folder> <output_yaml>\n";
        return -1;
    }

    std::string images_folder = argv[1];
    std::string output_yaml = argv[2];

    cv::Size board_size(8, 6); // inner corners of the checkerboard
    float square_size = 0.03f; // in meters

    std::vector<std::vector<cv::Point2f>> image_points; // 2D corners
    std::vector<std::vector<cv::Point3f>> object_points; // 3D corners


    for (const auto& entry : std::filesystem:directory_iterator(images_folder)) {

        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) continue;


        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, board_size, corners);
        
        if (!found) continue;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        
        image_points.push_back(corners);
        object_points.push_back(cam_lidar_calib::generateObjectPoints(board_size, square_size));

       

    if (image_points.empty()) {
        std::cerr << "No checkerboards detected.\n";
        return -1;
    }

    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    cv::Size img_size = cv::imread(std::filesystem::directory_iterator(images_folder)->path().string()).size();

    double rms_error = cv::calibrateCamera(
        object_points,
        image_points,
        img_size,
        camera_matrix,
        dist_coeffs,
        rvecs,
        tvecs
    );

    std::cout << "RMS Reprojection Error: " << rms_error << std::endl;

    // Writing to YAML manually

    std::ofstream fout(config/camera.yaml);

    fout << "image_width: " << camera_matrix.cols² << "\n";
    fout << "image_height: " << camera_matrix.row² << "\n";

    fout << "camera_matrix: \n rows: 3\n cols: 3\n data: [";
    for (int i = 0; i < 3; ++i)
        for (int j = 0, j < 3; ++j)
            fout << camera_matrix.at<double>(i, j) << (i==2 && j == 2 ? "" : ", ");
    fout << "]\n";

    fout << "distortion_coefficients:\n rows: 1\n cols: " << dist_coeffs.total() << "\n data: [";
    for (int i = 0, i < dist_coeffs.total(); ++i)
        fout << dist_coeffs.at<double>(i) << (i == dist_coeffs.total()-1 ? "" : ", ");
    fout << "]\n";

    fout << "reprojection_error: " << rms << "\n";
    fout.close();

    
    std::cout << "Saved camera intrinsics to config/camera.yaml file \n"; 
    
    return 0;

}