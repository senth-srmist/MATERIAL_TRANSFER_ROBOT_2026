#!/usr/bin/env python3

import cv2
import numpy as np
import yaml
import os
import rclpy
from rclpy.node import Node


class ArucoMarkerGenerator(Node):

    def __init__(self):
        super().__init__("aruco_marker_generator")

        # Declare ROS2 parameters
        self.declare_parameter("output_dir", "aruco_markers")
        self.declare_parameter("num_markers", 1)
        self.declare_parameter("marker_size", 1.0)
        self.declare_parameter("size_unit", "ft")  # Options: cm, in, m, mm, ft
        self.declare_parameter("aruco_dict", "6x6")

        # Get parameters
        self.output_dir = self.get_parameter("output_dir").value
        self.num_markers = self.get_parameter("num_markers").value
        self.marker_size = self.get_parameter("marker_size").value
        self.size_unit = self.get_parameter("size_unit").value

        # Dictionary mapping
        dict_map = {
            "4x4": cv2.aruco.DICT_4X4_250,
            "5x5": cv2.aruco.DICT_5X5_250,
            "6x6": cv2.aruco.DICT_6X6_250,
            "7x7": cv2.aruco.DICT_7X7_250,
        }
        aruco_dict_param = self.get_parameter("aruco_dict").value
        self.aruco_dict = dict_map.get(aruco_dict_param,
                                       cv2.aruco.DICT_6X6_250)

        # Unit conversion to meters
        self.unit_to_meters = {
            "cm": 0.01,
            "in": 0.0254,
            "m": 1.0,
            "mm": 0.001,
            "ft": 0.3048,
        }

        # Convert marker size to meters
        if self.size_unit not in self.unit_to_meters:
            self.get_logger().error(
                f"Invalid size unit '{self.size_unit}'. Using 'cm' as default."
            )
            self.size_unit = "cm"

        self.marker_size_meters = self.marker_size * self.unit_to_meters[
            self.size_unit]

        # Calculate pixel size (assuming 96 DPI for display, but we'll use a fixed conversion)
        # For printing: 1 inch = 300 pixels is common for high quality
        # We'll use a reasonable default of 200 pixels per 10cm (or adjust as needed)
        self.pixels_per_meter = 2000  # This gives good quality markers
        self.marker_size_pixels = int(self.marker_size_meters *
                                      self.pixels_per_meter)

        # Ensure minimum size
        if self.marker_size_pixels < 50:
            self.get_logger().warn(
                f"Calculated pixel size too small ({self.marker_size_pixels}px). Setting to 50px minimum."
            )
            self.marker_size_pixels = 50

        self.get_logger().info(f"ArUco Marker Generator initialized")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"Number of markers: {self.num_markers}")
        self.get_logger().info(
            f"Marker size: {self.marker_size} {self.size_unit} ({self.marker_size_meters:.4f} m)"
        )
        self.get_logger().info(
            f"Pixel size: {self.marker_size_pixels}x{self.marker_size_pixels} px"
        )
        self.get_logger().info(f"Dictionary: {aruco_dict_param}")

    def generate_markers(self):
        """Generate ArUco markers with corresponding YAML configuration files."""

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

        # Get the ArUco dictionary
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(self.aruco_dict)

        self.get_logger().info(
            f"Generating {self.num_markers} ArUco markers...")

        for marker_id in range(self.num_markers):
            # Generate the marker image
            marker_image = cv2.aruco.generateImageMarker(
                aruco_dictionary, marker_id, self.marker_size_pixels)

            # Save the marker image
            image_filename = os.path.join(self.output_dir,
                                          f"marker_{marker_id}.png")
            cv2.imwrite(image_filename, marker_image)

            # Create YAML configuration with size in meters
            yaml_data = {
                f"marker_{marker_id}": {
                    "aruco_id": marker_id,
                    "position": [0.0, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0],
                }
            }

            # Save YAML file
            yaml_filename = os.path.join(self.output_dir,
                                         f"marker_{marker_id}.yaml")
            with open(yaml_filename, "w") as yaml_file:
                yaml.dump(yaml_data,
                          yaml_file,
                          default_flow_style=False,
                          sort_keys=False)

            self.get_logger().info(f"Generated marker {marker_id}")

        self.get_logger().info(f"\n{'=' * 60}")
        self.get_logger().info(f"All markers generated successfully!")
        self.get_logger().info(f"Location: {os.path.abspath(self.output_dir)}")
        self.get_logger().info(
            f"Marker size: {self.marker_size} {self.size_unit} ({self.marker_size_meters:.4f} m)"
        )
        self.get_logger().info(
            f"Image size: {self.marker_size_pixels}x{self.marker_size_pixels} pixels"
        )
        self.get_logger().info(f"{'=' * 60}\n")


def main(args=None):
    rclpy.init(args=args)

    node = ArucoMarkerGenerator()
    node.generate_markers()

    # Shutdown after generation
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
