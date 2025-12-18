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

class ArucoEvaluationNode : public rclcpp::Node

{
public:
    ArucoEvaluationNode() : Node("aruco_evaluation")
    {
        clock_ = this->get_clock();
        
        this->declare_parameter<double>("general.marker_size");

        if (!this->get_parameter("general.marker_size", marker_size_m_)) {
            RCLCPP_FATAL(
                get_logger(),
                "Required parameter 'general.marker_size' not set. "
                "Check aruco_loc.yaml"
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

        service_ = this->create_service<
            zed_aruco_localization::srv::StartArucoEvaluation>(
                "start_evaluation",
                std::bind(&ArucoEvaluationNode::startMeasurementCb,
                    this, _1, _2));

        RCLCPP_INFO(this->get_logger(),
            "Aruco evaluation node started");

        cam_sub_ = image_transport::create_camera_subscription(
            this,
            "/zed/zed_node/rgb/color/rect/image",
            std::bind(&ArucoEvaluationNode::imageCb,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            "raw"
        );

        RCLCPP_INFO(this->get_logger(),
            "Subscribed to camera topic");
    }

private:

    bool measuring_ = false;
    int samples_target_ = 0;
    int frames_seen_ = 0;

    float current_distance_ = 0.0f;
    float current_angle_ = 0.0f;
    
    int frames_attempted_ = 0;
    int frames_with_marker_ = 0;
    
    double marker_size_m_;
    
    std::vector<double> xs_;
    std::vector<double> ys_;
    std::vector<double> zs_;
    std::vector<double> yaws_;
    
    image_transport::CameraSubscriber cam_sub_;

    rclcpp::Clock::SharedPtr clock_;
    
    rclcpp::Service<zed_aruco_localization::srv::StartArucoEvaluation>::SharedPtr service_;
    
    void startMeasurementCb(
        const std::shared_ptr<zed_aruco_localization::srv::StartArucoEvaluation::Request> request,
        std::shared_ptr<zed_aruco_localization::srv::StartArucoEvaluation::Response> response
    )
    {
        current_distance_ = request->distance_m;
        current_angle_ = request->angle_deg;
        samples_target_ = request->num_samples;

        frames_seen_ = 0;
        measuring_ = true;

        RCLCPP_INFO(this->get_logger(),
            "Measurement armed: distance=%.2f m, angle=%.2f deg, samples=%d",
            current_distance_, current_angle_, samples_target_);

            response->success = true;
            response->message = "Service call received";
    }

    void imageCb(
        const sensor_msgs::msg::Image::ConstSharedPtr &img,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info)
    {
        if (!measuring_) return;

        frames_attempted_++;

        cv::Mat bgra(
            img->height,
            img->width,
            CV_8UC4,
            const_cast<unsigned char*>(img->data.data())
        );

        cv::Mat gray;
        cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::Mat image_bgra(
            img->height,
            img->width,
            CV_8UC4,
            const_cast<unsigned char*>(img->data.data())
        );

        cv::Mat image_bgr;
        cv::cvtColor(image_bgra, image_bgr, cv::COLOR_BGRA2BGR);
        
        auto dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

        cv::aruco::detectMarkers(gray, dictionary, corners, ids);
        
        if (ids.empty()) return;

        int idx = 0;

        cv::Matx33d K;
        K(0,0) = info->k[0];
        K(1,1) = info->k[4];
        K(0,2) = info->k[2];
        K(1,2) = info->k[5];
        K(2,2) = 1.0;

        cv::Vec4d dist_coeffs(0, 0, 0, 0);

        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::estimatePoseSingleMarkers(
            corners,
            marker_size_m_,
            K,
            dist_coeffs,
            rvecs,
            tvecs
        );
        
        cv::aruco::drawAxis(
            image_bgr,
            K,
            dist_coeffs,
            rvecs[idx],
            tvecs[idx],
            0.1
        );

        double x = tvecs[idx][0];
        double y = tvecs[idx][1];
        double z = tvecs[idx][2];

        cv::Mat R;
        cv::Rodrigues(rvecs[idx], R);

        double yaw = std::atan2(
            R.at<double>(1,0),
            R.at<double>(0,0)
        );

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,  // milliseconds
            "POSE | x=%.3f m, y=%.3f m, z=%.3f m, yaw=%.2f deg",
            x,
            y,
            z,
            yaw * 180.0 / M_PI
        );
        
        xs_.push_back(x);
        ys_.push_back(y);
        zs_.push_back(z);
        yaws_.push_back(yaw);

        frames_with_marker_++;

        if (frames_attempted_ >= samples_target_) {
            measuring_ = false;

            RCLCPP_INFO(get_logger(),
                "Detection Test Complete. Detected in %d / %d frames.",
                frames_with_marker_,
                frames_attempted_
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
