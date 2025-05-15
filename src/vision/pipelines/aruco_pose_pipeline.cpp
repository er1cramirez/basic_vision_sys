#include "aruco_pose_pipeline.h"
#include <opencv2/calib3d.hpp>
#include <chrono>

// Initialize settings with default values
ArucoPoseSettings::ArucoPoseSettings()
    : dictionaryId(cv::aruco::DICT_5X5_1000),
      useCornerRefinement(true),
      cornerRefinementMaxIterations(30),
      cornerRefinementMinAccuracy(0.01),
      adaptiveThreshWinSizeMin(3),
      adaptiveThreshWinSizeMax(23),
      adaptiveThreshWinSizeStep(10),
      adaptiveThreshConstant(7.0),
      markerSizeMeters(0.5),   // 50cm default marker size
      maxReprojectionError(2.0),
      debugVisualization(true) {
}

// ArucoPoseResult constructor
ArucoPoseResult::ArucoPoseResult()
    : processingTimeMs(0.0),
      fps(0.0),
      detectionValid(false),
      markerId(-1),
      reprojectionError(0.0),
      position(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()) {
    
    timestamp = std::chrono::steady_clock::now();
}

// Apply transformation to pose result
void ArucoPoseResult::applyTransform(const Eigen::Matrix4d& transformMatrix) {
    // Extract rotation matrix and translation vector from the transform
    Eigen::Matrix3d rotMatrix = transformMatrix.block<3,3>(0,0);
    Eigen::Vector3d translation = transformMatrix.block<3,1>(0,3);
    
    // Apply transformation to position
    Eigen::Vector4d homogeneousPosition;
    homogeneousPosition << position, 1.0;
    Eigen::Vector4d transformedPosition = transformMatrix * homogeneousPosition;
    position = transformedPosition.head<3>() / transformedPosition(3);  // Normalize if needed
    
    // Apply rotation to orientation
    Eigen::Quaterniond rotQuaternion(rotMatrix);
    orientation = rotQuaternion * orientation;
}

// ArucoPosePipeline constructor
ArucoPosePipeline::ArucoPosePipeline() 
    : hasLastValidResult(false),
      totalFramesProcessed(0),
      validDetectionsCount(0),
      avgProcessingTimeMs(0.0),
      currentFps(0.0) {
          
    // Initialize the ArUco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(settings.dictionaryId);
    
    // Initialize detector parameters
    detectorParams = cv::aruco::DetectorParameters();
    
    // Create the detector
    detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
    
    // Setup parameters with default settings
    setupDetectorParams();
    
    // Store current time for FPS calculation
    lastFrameTimestamp = std::chrono::steady_clock::now();
    
    // Pre-allocate vectors for marker detection
    markerCorners.reserve(10);     // For up to 10 markers
    rejectedCandidates.reserve(20);
    markerIds.reserve(10);
}

// Apply transformation to a pose result (static method)
void ArucoPosePipeline::applyTransform(ArucoPoseResult& result, const Eigen::Matrix4d& transformMatrix) {
    result.applyTransform(transformMatrix);
}

// Set camera calibration
void ArucoPosePipeline::setCalibration(const CameraCalibration& newCalibration) {
    calibration = newCalibration;
}

// Update pipeline settings
void ArucoPosePipeline::updateSettings(const ArucoPoseSettings& newSettings) {
    settings = newSettings;
    
    // Update dictionary if needed
    if (dictionary.markerSize != settings.dictionaryId) {
        dictionary = cv::aruco::getPredefinedDictionary(settings.dictionaryId);
        detector.setDictionary(dictionary);
    }
    
    // Update detector parameters
    setupDetectorParams();
}

// Set up detector parameters from settings
void ArucoPosePipeline::setupDetectorParams() {
    // Set parameters from settings
    detectorParams.adaptiveThreshWinSizeMin = settings.adaptiveThreshWinSizeMin;
    detectorParams.adaptiveThreshWinSizeMax = settings.adaptiveThreshWinSizeMax;
    detectorParams.adaptiveThreshWinSizeStep = settings.adaptiveThreshWinSizeStep;
    detectorParams.adaptiveThreshConstant = settings.adaptiveThreshConstant;
    
    // Corner refinement settings
    detectorParams.cornerRefinementMethod = settings.useCornerRefinement ? 
        cv::aruco::CORNER_REFINE_SUBPIX : cv::aruco::CORNER_REFINE_NONE;
    detectorParams.cornerRefinementMaxIterations = settings.cornerRefinementMaxIterations;
    detectorParams.cornerRefinementMinAccuracy = settings.cornerRefinementMinAccuracy;
    
    // Update the detector with new parameters
    detector.setDetectorParameters(detectorParams);
}

// Convert OpenCV rotation and translation vectors to Eigen pose
void ArucoPosePipeline::convertCVToEigenPose(const cv::Vec3d& rvec, const cv::Vec3d& tvec, 
                         Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
    // Convert translation vector to Eigen
    position = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
    
    // Convert rotation vector to quaternion
    cv::Mat rotMat;
    cv::Rodrigues(rvec, rotMat);
    
    Eigen::Matrix3d eigenRotMat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            eigenRotMat(i, j) = rotMat.at<double>(i, j);
        }
    }
    
    orientation = Eigen::Quaterniond(eigenRotMat);
    orientation.normalize();
}

// Create debug visualization
cv::Mat ArucoPosePipeline::createDebugVisualization(const cv::Mat& inputImage, 
                             const std::vector<std::vector<cv::Point2f>>& corners,
                             const std::vector<int>& ids,
                             bool detectionValid) {
    if (!settings.debugVisualization) {
        return cv::Mat();
    }
    
    cv::Mat visual = inputImage.clone();
    
    // Draw detected markers
    if (!corners.empty()) {
        cv::aruco::drawDetectedMarkers(visual, corners, ids);
        
        // Draw validity status
        std::string statusText = detectionValid ? "VALID" : "INVALID";
        cv::Scalar statusColor = detectionValid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        
        cv::putText(visual, statusText, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, statusColor, 2);
        
        // Draw a cross in the center of the frame
        int centerX = visual.cols / 2;
        int centerY = visual.rows / 2;
        int lineLength = 20; // Length of each arm of the cross

        // Draw horizontal line
        cv::line(visual, 
                 cv::Point(centerX - lineLength, centerY),
                 cv::Point(centerX + lineLength, centerY),
                 cv::Scalar(0, 0, 255), // Red color (BGR)
                 2); // Thickness

        // Draw vertical line
        cv::line(visual, 
                 cv::Point(centerX, centerY - lineLength),
                 cv::Point(centerX, centerY + lineLength),
                 cv::Scalar(0, 0, 255), // Red color (BGR)
                 2); // Thickness
        // If detection is valid and we have pose information, draw axes
        if (detectionValid && calibration.isValid() && !corners[0].empty()) {
            // Convert Eigen pose back to OpenCV for visualization
            Eigen::AngleAxisd angleAxis(lastValidResult.orientation);
            cv::Vec3d rvec(angleAxis.axis().x() * angleAxis.angle(),
                          angleAxis.axis().y() * angleAxis.angle(),
                          angleAxis.axis().z() * angleAxis.angle());
            
            cv::Vec3d tvec(lastValidResult.position.x(),
                          lastValidResult.position.y(),
                          lastValidResult.position.z());
            
            cv::drawFrameAxes(visual, calibration.cameraMatrix, calibration.distCoeffs,
                             rvec, tvec, settings.markerSizeMeters * 0.5, 2);
        }
    } else {
        cv::putText(visual, "NO MARKER", cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    
    // Draw FPS and other information
    cv::putText(visual, 
              "FPS: " + std::to_string(static_cast<int>(currentFps)), 
              cv::Point(10, 60), 
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    
    cv::putText(visual, 
              "Frames: " + std::to_string(totalFramesProcessed), 
              cv::Point(10, 90), 
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    
    // Draw processing time
    cv::putText(visual, 
              "Processing: " + std::to_string(static_cast<int>(avgProcessingTimeMs)) + " ms", 
              cv::Point(10, 120), 
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    
    return visual;
}

// Update FPS calculation
void ArucoPosePipeline::updateFPS() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastFrameTimestamp).count();
    
    if (elapsed > 0) {
        currentFps = 0.7 * currentFps + 0.3 * (1000.0 / elapsed);
    }
    
    lastFrameTimestamp = now;
}

// Process a frame for ArUco detection
ArucoPoseResult ArucoPosePipeline::process(const ArucoInputFrame& frame) {
    // Start timing
    auto startTime = std::chrono::steady_clock::now();
    
    // Initialize result
    ArucoPoseResult result;
    result.timestamp = frame.timestamp;
    
    // Check if we have a valid image
    if (frame.image.empty()) {
        result.detectionValid = false;
        
        // Update processing time
        auto endTime = std::chrono::steady_clock::now();
        result.processingTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            endTime - startTime).count();
        
        // Update FPS
        updateFPS();
        result.fps = currentFps;
        
        return result;
    }
    
    // Make sure we have grayscale image for detection
    cv::Mat grayImage;
    if (frame.image.channels() == 1) {
        grayImage = frame.image;
    } else {
        cv::cvtColor(frame.image, grayImage, cv::COLOR_BGR2GRAY);
    }
    
    // Clear vectors but maintain capacity
    markerCorners.clear();
    rejectedCandidates.clear();
    markerIds.clear();
    
    // Detect markers
    detector.detectMarkers(grayImage, markerCorners, markerIds, rejectedCandidates);
    
    // Process detected markers
    if (!markerIds.empty()) {
        // We're only interested in the first marker (single ArUco mode)
        result.markerId = markerIds[0];
        
        // Store corner points
        result.corners.assign(markerCorners[0].begin(), markerCorners[0].end());
        
        // Calculate center
        result.center = cv::Point2f(0, 0);
        for (const auto& corner : result.corners) {
            result.center.x += corner.x;
            result.center.y += corner.y;
        }
        if (!result.corners.empty()) {
            result.center.x /= result.corners.size();
            result.center.y /= result.corners.size();
        }
        
        // Estimate pose if calibration is valid
        if (calibration.isValid()) {
            // Estimate pose for single marker
            std::vector<cv::Vec3d> rvecs, tvecs;
            std::vector<std::vector<cv::Point2f>> singleMarkerCorners = {markerCorners[0]};
            
            cv::aruco::estimatePoseSingleMarkers(
                singleMarkerCorners,
                settings.markerSizeMeters,
                calibration.cameraMatrix,
                calibration.distCoeffs,
                rvecs, tvecs);
            
            if (!rvecs.empty() && !tvecs.empty()) {
                // Convert OpenCV pose to Eigen
                convertCVToEigenPose(rvecs[0], tvecs[0], result.position, result.orientation);
                
                // Calculate reprojection error
                std::vector<cv::Point2f> projectedPoints;
                std::vector<cv::Point3f> objectPoints = {
                    cv::Point3f(-settings.markerSizeMeters/2, settings.markerSizeMeters/2, 0),
                    cv::Point3f(settings.markerSizeMeters/2, settings.markerSizeMeters/2, 0),
                    cv::Point3f(settings.markerSizeMeters/2, -settings.markerSizeMeters/2, 0),
                    cv::Point3f(-settings.markerSizeMeters/2, -settings.markerSizeMeters/2, 0)
                };
                
                cv::projectPoints(
                    objectPoints, rvecs[0], tvecs[0], 
                    calibration.cameraMatrix, calibration.distCoeffs,
                    projectedPoints);
                
                // Calculate reprojection error
                double totalError = 0.0;
                for (size_t i = 0; i < 4; i++) {
                    totalError += cv::norm(markerCorners[0][i] - projectedPoints[i]);
                }
                result.reprojectionError = totalError / 4.0;
                
                // Check if detection is valid based on thresholds
                result.detectionValid = 
                    result.reprojectionError <= settings.maxReprojectionError;
                
                // Update last valid result
                if (result.detectionValid) {
                    lastValidResult = result;
                    hasLastValidResult = true;
                    validDetectionsCount++;
                }
            }
        }
    } else {
        // No markers detected
        result.detectionValid = false;
    }
    
    // Finish timing
    auto endTime = std::chrono::steady_clock::now();
    result.processingTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime).count();
    
    // Update average processing time with exponential moving average
    avgProcessingTimeMs = 0.9 * avgProcessingTimeMs + 0.1 * result.processingTimeMs;
    
    // Update FPS
    updateFPS();
    result.fps = currentFps;
    
    // Update frame count
    totalFramesProcessed++;
    
    // Create debug visualization
    if (settings.debugVisualization) {
        cv::Mat inputImageForDebug = frame.image;
        if (inputImageForDebug.empty() && !grayImage.empty()) {
            cv::cvtColor(grayImage, inputImageForDebug, cv::COLOR_GRAY2BGR);
        }
        
        if (!inputImageForDebug.empty()) {
            result.debugImage = createDebugVisualization(
                inputImageForDebug, 
                markerCorners, 
                markerIds, 
                result.detectionValid
            );
        }
    }
    
    return result;
}

// Get the last valid detection result
std::optional<ArucoPoseResult> ArucoPosePipeline::getLastValidResult() const {
    if (hasLastValidResult) {
        return lastValidResult;
    }
    return std::nullopt;
}

// Reset the pipeline state
void ArucoPosePipeline::reset() {
    hasLastValidResult = false;
    totalFramesProcessed = 0;
    validDetectionsCount = 0;
    avgProcessingTimeMs = 0.0;
}