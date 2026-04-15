import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import os


class SpeechNode(Node):
    def __init__(self):
        super().__init__("speech_node")

        # 🔹 Service (for terminal/manual speech)
        # Call with: ros2 service call /speak std_srvs/srv/Trigger {}
        self.srv = self.create_service(Trigger, "speak", self.handle_speech)

        # 🔹 Subscribe to human alarm
        self.create_subscription(Bool, "/human_alarm/active", self.alarm_callback, 10)

        self.get_logger().info("Speech node ready")

    # 🔊 Service callback (manual speech)
    def handle_speech(self, request, response):
        message = "Please move away from the robot"
        self.get_logger().info(f"Speaking: {message}")
        os.system(f'espeak "{message}"')
        response.success = True
        response.message = "spoken"
        return response

    # 🚨 Human detection trigger
    def alarm_callback(self, msg):
        if msg.data:
            self.get_logger().info("Human detected → speaking")
            os.system('espeak "Please move"')


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
