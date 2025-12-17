#include <rclcpp/rclcpp.hpp>
#include "zed_aruco_localization/srv/start_aruco_evaluation.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ArucoEvaluationNode : public rclcpp::Node

{
public:
  ArucoEvaluationNode()
  : Node("aruco_evaluation")
  {
    service_ = this->create_service<
      zed_aruco_localization::srv::StartArucoEvaluation>(
        "start_evaluation",
        std::bind(&ArucoEvaluationNode::startMeasurementCb,
                  this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
                "Aruco evaluation node started");
  }

private:
  void startMeasurementCb(
    const std::shared_ptr<
      zed_aruco_localization::srv::StartArucoEvaluation::Request> request,
    std::shared_ptr<
      zed_aruco_localization::srv::StartArucoEvaluation::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "Service called: distance=%.2f m, angle=%.2f deg, samples=%d",
      request->distance_m,
      request->angle_deg,
      request->num_samples);

    response->success = true;
    response->message = "Service call received";
  }

  rclcpp::Service<
    zed_aruco_localization::srv::StartArucoEvaluation>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoEvaluationNode>());
  rclcpp::shutdown();
  return 0;
}
