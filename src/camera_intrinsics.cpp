#include <opencv4/opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;


namespace cam_lidar_calib {

//  Create 3D object points in board's own coordinate frame
std::vector<cv::Point3f> generateObjectPoints(cv::Size board_size, float square_size) 
{
    std::vector<cv::Point3f> obj_pts;
    for (int i = 0; i < board_size.height; ++i) 
        for (int j = 0; j < board_size.width; ++j)
            obj_pts.emplace_back(j*square_size, i*square_size, 0.0f);
    
    return obj_pts;

    }

}


int main(int argc, char** argv) 
{

    if (argc < 5) {
        std::cout << "Usage: compute_intrinsics <images_folder> <board_cols> <board_rows> <square_size(mts)>\n";
        return -1;
    }

    std::string images_folder = argv[1]; 
    int board_cols = std::stoi(argc[2]);      // inner corners columns
    int board_rows = std::stoi(argc[3]);      // inner corners rows
    float square_size = std::stof(arg[4]);    // in meters

    cv::Size board_size(board_cols, board_rows);

    std::vector<std::vector<cv::Point2f>> image_points; // 2D corners
    std::vector<std::vector<cv::Point3f>> object_points; // 3D corners

    cv::Size img_size;          // will be set from first valid image
    int images_processed = 0;
    int images_detected = 0;


    for (const auto& entry : fs::directory_iterator(images_folder)) 
    {
        // Skip non-image files
        std::string ext = entry.path().extension().string();
        if (ext != ".jpg" && ext != ".jpeg" && ext != ".png" && ext != ".bmp") continue;

        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) continue;
        images_processed++;

        // Store image size from first image ( all images must be same size)
        if (img_size.empty())
            img_size = img.size();

        // Detect checkerboard corners    
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, board_size, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        
        if (!found) {
            std::cout << " Skip, No board found in: " << entry.path().filename() << std::endl;
            continue;
        }

        // Refine to subpixel accuracy
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        
        // Store tis image's data
        image_points.push_back(corners);
        object_points.push_back(objp);
        images_detected++;

        std::cout << "[OK] Detected in: " << entry.path().filename()
                    << "( " << images_detected << "total)" << std::endl;

    }

    std::cout << "Processed " << images_processed << "images, detected board in "  << images_detected << std::endl;


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

     // Print results
    std::cout << "\n========================================" << std::endl;
    std::cout << "INTRINSIC CALIBRATION RESULTS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "RMS Reprojection Error: " << rms_error << " pixels" << std::endl;
    std::cout << "  fx = " << camera_matrix.at<double>(0, 0) << std::endl;
    std::cout << "  fy = " << camera_matrix.at<double>(1, 1) << std::endl;
    std::cout << "  cx = " << camera_matrix.at<double>(0, 2) << std::endl;
    std::cout << "  cy = " << camera_matrix.at<double>(1, 2) << std::endl;

    std::cout << "RMS Reprojection Error: " << rms_error << std::endl;

    if (rms_error > 1.0)
        std::cout << "  WARNING: High error. Add more diverse images." << std::endl;

    // Writing to YAML manually

    std::ofstream fout("config/camera.yaml");

    fout << "image_width: " << img_size.width << "\n";
    fout << "image_height: " << img_size.height << "\n";

    fout << "camera_matrix: \n rows: 3\n cols: 3\n data: [";
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            fout << camera_matrix.at<double>(i, j);
            if (!(i==2 && j == 2)) fout << ", ";
        }
    fout << "]\n";

    fout << "distortion_coefficients:\n rows: 1\n cols: " << dist_coeffs.total() << "\n data: [";
    for (size_t i = 0; i < dist_coeffs.total(); ++i) {
        fout << dist_coeffs.at<double>(i);
        if (i != dist_coeffs.total() - 1 ) fout << ", ";
    }
    fout << "]\n";

    fout << "reprojection_error: " << rms_error << "\n";
    fout.close();

    
    std::cout << "Saved camera intrinsics to config/camera.yaml file \n"; 
    
    return 0;

}