#ifndef ARUCO_POSE_PIPELINE_H
#define ARUCO_POSE_PIPELINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <chrono>
#include <optional>
#include <vector>

/**
 * @brief Camera calibration data
 */
struct CameraCalibration {
    cv::Mat cameraMatrix;      // 3x3 camera matrix
    cv::Mat distCoeffs;        // Distortion coefficients
    
    bool isValid() const {
        return !cameraMatrix.empty() && !distCoeffs.empty();
    }
};

/**
 * @brief Settings for ArUco marker detection and pose estimation
 */
struct ArucoPoseSettings {
    // ArUco dictionary settings
    int dictionaryId;
    
    // Corner refinement settings
    bool useCornerRefinement;
    int cornerRefinementMaxIterations;
    double cornerRefinementMinAccuracy;
    
    // Adaptive threshold parameters
    int adaptiveThreshWinSizeMin;
    int adaptiveThreshWinSizeMax;
    int adaptiveThreshWinSizeStep;
    double adaptiveThreshConstant;
    
    // Marker properties
    double markerSizeMeters;
    
    // Detection quality thresholds
    double maxReprojectionError;
    
    // Debug visualization
    bool debugVisualization;
    
    // Constructor with default values
    ArucoPoseSettings();
};

/**
 * @brief Input frame structure for ArUco detection
 */
struct ArucoInputFrame {
    cv::Mat image;              // Input image
    int sequenceId;             // Frame sequence ID
    
    // Timestamp when the frame was captured
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
    
    ArucoInputFrame() : sequenceId(0) {
        timestamp = std::chrono::steady_clock::now();
    }
};

/**
 * @brief Result of ArUco marker detection and pose estimation
 */
struct ArucoPoseResult {
    // Timing information
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
    double processingTimeMs;
    double fps;
    
    // Detection status
    bool detectionValid;
    int markerId;
    double reprojectionError;
    
    // Corners in image space
    std::vector<cv::Point2f> corners;
    cv::Point2f center;
    
    // 3D pose in camera reference frame
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    
    // Debug visualization
    cv::Mat debugImage;
    
    // Constructor with defaults
    ArucoPoseResult();
    
    /**
     * @brief Apply a transformation to the pose
     * @param transformMatrix 4x4 homogeneous transformation matrix
     */
    void applyTransform(const Eigen::Matrix4d& transformMatrix);
};

/**
 * @brief Pipeline for ArUco marker detection and pose estimation
 */
class ArucoPosePipeline {
public:
    /**
     * @brief Constructor
     */
    ArucoPosePipeline();
    
    /**
     * @brief Destructor
     */
    ~ArucoPosePipeline() = default;
    
    /**
     * @brief Set camera calibration parameters
     * @param calibration Camera calibration data
     */
    void setCalibration(const CameraCalibration& calibration);
    
    /**
     * @brief Update pipeline settings
     * @param settings New settings to apply
     */
    void updateSettings(const ArucoPoseSettings& settings);
    
    /**
     * @brief Process a frame to detect ArUco markers and estimate pose
     * @param frame Input frame
     * @return Detection result
     */
    ArucoPoseResult process(const ArucoInputFrame& frame);
    
    /**
     * @brief Get the last valid detection result
     * @return Optional containing the last valid result, or empty if no valid detection
     */
    std::optional<ArucoPoseResult> getLastValidResult() const;
    
    /**
     * @brief Reset the pipeline state
     */
    void reset();
    
    /**
     * @brief Apply transformation to a pose result (static method)
     * @param result The pose result to transform
     * @param transformMatrix 4x4 homogeneous transformation matrix
     */
    static void applyTransform(ArucoPoseResult& result, const Eigen::Matrix4d& transformMatrix);
    
private:
    // ArUco detector components
    cv::aruco::Dictionary dictionary;
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector;
    
    // Settings
    ArucoPoseSettings settings;
    
    // Camera calibration
    CameraCalibration calibration;
    
    // Last valid detection result
    ArucoPoseResult lastValidResult;
    bool hasLastValidResult;
    
    // Performance metrics
    int totalFramesProcessed;
    int validDetectionsCount;
    double avgProcessingTimeMs;
    double currentFps;
    std::chrono::time_point<std::chrono::steady_clock> lastFrameTimestamp;
    
    // Pre-allocated vectors for marker detection (to avoid reallocations)
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    std::vector<int> markerIds;
    
    // Set up detector parameters from settings
    void setupDetectorParams();
    
    // Convert OpenCV rotation and translation vectors to Eigen pose
    void convertCVToEigenPose(const cv::Vec3d& rvec, const cv::Vec3d& tvec, 
                             Eigen::Vector3d& position, Eigen::Quaterniond& orientation);
    
    // Create debug visualization
    cv::Mat createDebugVisualization(const cv::Mat& inputImage, 
                                 const std::vector<std::vector<cv::Point2f>>& corners,
                                 const std::vector<int>& ids,
                                 bool detectionValid);
    
    // Update FPS calculation
    void updateFPS();
};

#endif // ARUCO_POSE_PIPELINE_H