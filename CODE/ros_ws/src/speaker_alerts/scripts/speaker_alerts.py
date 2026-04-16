#!/usr/bin/env python3
# Copyright (c) 2026 Tejas
# Licensed under the Apache License, Version 2.0
"""
Speech service node - provides /speak service for text-to-speech.

Any node can call this service with a message to speak.
Uses espeak for TTS (non-blocking).

Usage:
  ros2 run speaker_alerts speaker_alerts

Service call example:
  ros2 service call /speak speech_pkg/srv/Speak "{message: 'Hello world'}"
"""

import subprocess
import rclpy
from rclpy.node import Node
from speaker_alerts.srv import Speak


class SpeakerAlerts(Node):
    def __init__(self):
        super().__init__("speaker_alerts")

        self.srv = self.create_service(Speak, "speak", self.handle_speak)
        self.speech_proc = None

        self.get_logger().info("Speech node ready - /speak service available")

    def handle_speak(self, request, response):
        """Handle speech request - non-blocking via subprocess."""
        message = request.message.strip()

        if not message:
            self.get_logger().warn("Empty speech request ignored")
            response.success = False
            return response

        # Kill any ongoing speech to avoid overlap
        if self.speech_proc is not None and self.speech_proc.poll() is None:
            self.speech_proc.terminate()
            try:
                self.speech_proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                self.speech_proc.kill()

        # Non-blocking speech
        try:
            self.speech_proc = subprocess.Popen(
                ["espeak", message],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"Speaking: {message}")
            response.success = True
        except FileNotFoundError:
            self.get_logger().error(
                "espeak not found - install with: sudo apt install espeak"
            )
            response.success = False
        except Exception as e:
            self.get_logger().error(f"Speech failed: {e}")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerAlerts()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
