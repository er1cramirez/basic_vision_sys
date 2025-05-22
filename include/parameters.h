#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>
#include <opencv2/highgui.hpp>


namespace UAV_Parameters {
    // Default values
    const bool IS_SIMULATOR = true;

    // Simulation parameters
    const std::string UDP_IP = "127.0.0.1";
    const std::string GZ_CAM_TOPIC = "/world/default/model/iris/link/camera_link/sensor/camera/image";
    const int GZ_MAV_PORT = 14550;

    // Real UAV parameters
    const std::string MAV_SER_DEV = "/dev/ttyUSB0";
    const int SER_BAUD = 57600;


    // Mavlink parameters
    const double COM_FREQ = 50.0;
    const uint8_t SYS_ID = 255;
    const uint8_t COMP_ID = 1;
    const uint8_t TG_ID = 1;
    const uint8_t TG_COMP = 0;

    // Aruco parameters
    const double ARUCO_MARKER_SIZE = 0.5; // Marker size in meters (simulator = 50cm, real = 27cm)    

    const cv::Mat CAM_MAT_SIM = (cv::Mat_<double>(3, 3, CV_64F) << 
        205.46962738037109, 0.0, 320.0,
        0.0, 205.46965599060059, 240.0,
        0.0, 0.0, 1.0);
    const cv::Mat DIST_COEF_SIM = (cv::Mat_<double>(1, 5, CV_64F) << 0.0, 0.0, 0.0, 0.0, 0.0);


    const cv::Mat CAM_MAT_REAl = (cv::Mat_<double>(3, 3, CV_64F) << 
        205.46962738037109, 0.0, 320.0,
        0.0, 205.46965599060059, 240.0,
        0.0, 0.0, 1.0);
    const cv::Mat DIST_COEF_REAL = (cv::Mat_<double>(1, 5, CV_64F) << 0.0, 0.0, 0.0, 0.0, 0.0);


    // PD position control parameters
    const double POSITION_KP = 1.0; // Proportional gain for position control
    const double POSITION_KD = 0.5; // Derivative gain for position control

    // P velocity control parameters
    const double VELOCITY_KP = 1.0; // Proportional gain for velocity control
    
    // EKF parameters
    const double EKF_PROCESS_NOISE_COVARIANCE = 0.01; // Process noise covariance
    const double EKF_MEASUREMENT_NOISE_COVARIANCE = 0.01; // Measurement noise covariance




}
#endif // PARAMETERS_H