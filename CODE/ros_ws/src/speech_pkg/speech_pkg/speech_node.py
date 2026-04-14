import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from human_detection.srv import Alarm
import os

class SpeechNode(Node):

    def __init__(self):
        super().__init__('speech_node')

        # 🔹 Service (for terminal/manual speech)
        self.srv = self.create_service(
            Alarm,
            'speak',
            self.handle_speech
        )

        # 🔹 Subscribe to human alarm
        self.create_subscription(
            Bool,
            '/human_alarm/active',
            self.alarm_callback,
            10
        )

        self.get_logger().info("Speech node ready")

    # 🔊 Service callback (manual speech)
    def handle_speech(self, request, response):
        self.get_logger().info(f"Speaking: {request.message}")
        os.system(f'espeak "{request.message}"')

        response.success = True
        return response

    # 🚨 Human detection trigger
    def alarm_callback(self, msg):
        if msg.data:
            self.get_logger().info("Human detected → speaking")
            os.system('espeak "Please move"')