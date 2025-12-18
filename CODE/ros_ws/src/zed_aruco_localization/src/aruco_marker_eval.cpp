#include <rclcpp/rclcpp.hpp>
#include "zed_aruco_localization/srv/start_aruco_evaluation.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "aruco.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Node for evaluating ArUco marker detection accuracy
 * 
 * This node subscribes to camera images, detects ArUco markers,
 * and collects pose statistics for evaluation purposes.
 */
class ArucoEvaluationNode : public rclcpp::Node
{
public:
    ArucoEvaluationNode() : Node("aruco_evaluation")
    {
        clock_ = this->get_clock();
        
        // Load marker size from configuration
        initializeParameters();
        
        // Set up service for starting evaluation runs
        service_ = this->create_service<zed_aruco_localization::srv::StartArucoEvaluation>(
            "start_evaluation",
            std::bind(&ArucoEvaluationNode::startMeasurementCb, this, _1, _2)
        );
        
        RCLCPP_INFO(this->get_logger(), "ArUco evaluation node started");
        
        // Subscribe to camera feed
        cam_sub_ = image_transport::create_camera_subscription(
            this,
            "/zed/zed_node/rgb/color/rect/image",
            std::bind(&ArucoEvaluationNode::imageCb, this, std::placeholders::_1, std::placeholders::_2),
            "raw"
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to camera topic");
    }

private:
    // ==================== Configuration Parameters ====================
    double marker_size_m_;  ///< Physical size of ArUco marker in meters
    
    // ==================== Measurement State ====================
    bool measuring_ = false;           ///< Flag indicating if measurement is active
    int samples_target_ = 0;           ///< Target number of frames to collect
    float current_distance_ = 0.0f;    ///< Ground truth distance for current test (meters)
    float current_angle_ = 0.0f;       ///< Ground truth angle for current test (degrees)
    
    // ==================== Detection Statistics ====================
    int frames_attempted_ = 0;         ///< Total frames processed in current measurement
    int frames_with_marker_ = 0;       ///< Frames where marker was successfully detected
    
    // ==================== Collected Pose Data ====================
    std::vector<double> xs_;           ///< X positions (lateral, meters)
    std::vector<double> ys_;           ///< Y positions (vertical, meters)
    std::vector<double> zs_;           ///< Z positions (depth, meters)
    std::vector<double> yaws_;         ///< Yaw angles (rotation, radians)
    
    // ==================== ROS Communication ====================
    image_transport::CameraSubscriber cam_sub_;  ///< Camera image subscriber
    rclcpp::Clock::SharedPtr clock_;             ///< ROS clock for throttling
    rclcpp::Service<zed_aruco_localization::srv::StartArucoEvaluation>::SharedPtr service_;
    
    /**
     * @brief Initialize and validate node parameters
     */
    void initializeParameters()
    {
        this->declare_parameter<double>("general.marker_size");

        if (!this->get_parameter("general.marker_size", marker_size_m_)) {
            RCLCPP_FATAL(
                get_logger(),
                "Required parameter 'general.marker_size' not set. Check aruco_loc.yaml"
            );
            throw std::runtime_error("Missing required parameter: general.marker_size");
        }

        if (marker_size_m_ <= 0.0) {
            RCLCPP_FATAL(
                get_logger(),
                "Invalid marker size: %.3f m. Must be > 0.",
                marker_size_m_
            );
            throw std::runtime_error("Invalid marker size");
        }

        RCLCPP_INFO(
            get_logger(),
            "Using ArUco marker size from YAML: %.3f m",
            marker_size_m_
        );
    }
    
    /**
     * @brief Service callback to start a new evaluation measurement
     * 
     * @param request Contains distance, angle, and number of samples
     * @param response Returns success status and message
     */
    void startMeasurementCb(
        const std::shared_ptr<zed_aruco_localization::srv::StartArucoEvaluation::Request> request,
        std::shared_ptr<zed_aruco_localization::srv::StartArucoEvaluation::Response> response
    )
    {
        // Store test parameters
        current_distance_ = request->distance_m;
        current_angle_ = request->angle_deg;
        samples_target_ = request->num_samples;

        // Reset measurement state
        frames_attempted_ = 0;
        frames_with_marker_ = 0;
        xs_.clear();
        ys_.clear();
        zs_.clear();
        yaws_.clear();
        
        // Arm measurement
        measuring_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Measurement armed: distance=%.2f m, angle=%.2f deg, samples=%d",
            current_distance_, current_angle_, samples_target_
        );

        response->success = true;
        response->message = "Evaluation started successfully";
    }

    /**
     * @brief Camera callback - processes images and detects ArUco markers
     * 
     * @param img Camera image message
     * @param info Camera calibration information
     */
    void imageCb(
        const sensor_msgs::msg::Image::ConstSharedPtr &img,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info
    )
    {
        // Only process images when measurement is active
        if (!measuring_) return;

        frames_attempted_++;

        // Convert ROS image to OpenCV format (BGRA -> Grayscale for detection)
        cv::Mat bgra(
            img->height,
            img->width,
            CV_8UC4,
            const_cast<unsigned char*>(img->data.data())
        );

        cv::Mat gray;
        cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);

        // Detect ArUco markers in the image
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        
        auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
        cv::aruco::detectMarkers(gray, dictionary, corners, ids);
        
        // If no markers detected, skip this frame
        if (ids.empty()) return;

        // Extract camera intrinsics from CameraInfo message
        cv::Matx33d K;
        K(0, 0) = info->k[0];  // fx
        K(1, 1) = info->k[4];  // fy
        K(0, 2) = info->k[2];  // cx
        K(1, 2) = info->k[5];  // cy
        K(2, 2) = 1.0;

        cv::Vec4d dist_coeffs(0, 0, 0, 0);  // Assuming rectified image (no distortion)

        // Estimate 6DOF pose of detected markers
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            corners,
            marker_size_m_,
            K,
            dist_coeffs,
            rvecs,
            tvecs
        );
        
        // Process first detected marker (index 0)
        int idx = 0;
        
        // Extract position (translation vector)
        double x = tvecs[idx][0];
        double y = tvecs[idx][1];
        double z = tvecs[idx][2];

        // Convert rotation vector to rotation matrix
        cv::Mat R;
        cv::Rodrigues(rvecs[idx], R);

        // Calculate yaw angle from rotation matrix
        double yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));

        // Log pose information (throttled to 1 Hz)
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,  // milliseconds
            "POSE | x=%.3f m, y=%.3f m, z=%.3f m, yaw=%.2f deg",
            x, y, z, yaw * 180.0 / M_PI
        );
        
        // Store pose data
        xs_.push_back(x);
        ys_.push_back(y);
        zs_.push_back(z);
        yaws_.push_back(yaw);

        frames_with_marker_++;

        // Check if we've collected enough samples
        if (frames_attempted_ >= samples_target_) {
            measuring_ = false;

            RCLCPP_INFO(
                get_logger(),
                "Detection test complete. Detected in %d / %d frames (%.1f%% success rate)",
                frames_with_marker_,
                frames_attempted_,
                100.0 * frames_with_marker_ / frames_attempted_
            );
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoEvaluationNode>());
    rclcpp::shutdown();
    return 0;
}
