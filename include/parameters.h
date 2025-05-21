#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <string>


namespace UAV_Parameters {
    // Default values
    bool IS_SIMULATOR = true;
    std::string UDP_IP = "127.0.0.1";
    std::string GZ_CAM_TOPIC = "/world/default/model/iris/link/camera_link/sensor/camera/image";
    int GZ_MAV_PORT = 14550;
    std::string MAV_SER_DEV = "/dev/ttyUSB0";
    int SER_BAUD = 57600;
    double COM_FREQ = 50.0;

    uint8_t SYS_ID = 255;
    uint8_t COMP_ID = 1;
    uint8_t TG_ID = 1;
    uint8_t TG_COMP = 0;
    
    // Control parameters
    const double POSITION_KP = 1.0; // Proportional gain for position control
    const double POSITION_KD = 0.5; // Derivative gain for position control
    
    // EKF parameters
    const double EKF_PROCESS_NOISE_COVARIANCE = 0.01; // Process noise covariance
    const double EKF_MEASUREMENT_NOISE_COVARIANCE = 0.01; // Measurement noise covariance

}
#endif // PARAMETERS_H