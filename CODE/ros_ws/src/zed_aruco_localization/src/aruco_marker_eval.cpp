#include <rclcpp/rclcpp.hpp>
#include "zed_aruco_localization/srv/start_aruco_evaluation.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class ArucoEvaluationNode : public rclcpp::Node

{
public:
    ArucoEvaluationNode() : Node("aruco_evaluation")
    {
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

    image_transport::CameraSubscriber cam_sub_;

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

        frames_seen_++;

        if (frames_seen_ % 30 == 0) {
            RCLCPP_INFO(get_logger(),
                "Receiving frames... %d / %d",
                frames_seen_, samples_target_);
        }

        if (frames_seen_ >= samples_target_) {
            measuring_ = false;
            RCLCPP_INFO(get_logger(),
                "Frame collection complete (no processing yet)");
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
