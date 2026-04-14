#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import smbus2

INA219_ADDRESS = 0x40
BUS_VOLTAGE_REGISTER = 0x02

class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        # Publishers
        self.percentage_pub = self.create_publisher(Float32, '/battery_percentage', 10)
        self.voltage_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.low_battery_pub = self.create_publisher(Bool, '/low_battery_warning', 10)

        self.timer = self.create_timer(1.0, self.update)

        self.bus = smbus2.SMBus(1)

        # Threshold
        self.low_battery_threshold = 20.0  # %

    def read_voltage(self):
        try:
            raw = self.bus.read_word_data(INA219_ADDRESS, BUS_VOLTAGE_REGISTER)

            swapped = ((raw & 0xFF) << 8) | (raw >> 8)
            voltage = (swapped >> 3) * 0.004

            return voltage

        except Exception as e:
            self.get_logger().error(f"I2C Error: {e}")
            return None

    # 🔥 Improved LiPo Non-linear Mapping
    def voltage_to_percentage(self, voltage):
        if voltage is None:
            return 0.0

        # 3S LiPo discharge curve approximation
        curve = [
            (12.6, 100),
            (12.4, 90),
            (12.2, 80),
            (12.0, 70),
            (11.8, 60),
            (11.6, 50),
            (11.4, 40),
            (11.2, 30),
            (11.1, 20),
            (11.0, 10),
            (10.8, 0)
        ]

        # Interpolation
        for i in range(len(curve) - 1):
            v1, p1 = curve[i]
            v2, p2 = curve[i + 1]

            if v2 <= voltage <= v1:
                return p2 + (p1 - p2) * (voltage - v2) / (v1 - v2)

        if voltage >= 12.6:
            return 100.0
        if voltage <= 10.8:
            return 0.0

        return 0.0

    def update(self):
        voltage = self.read_voltage()

        if voltage is None:
            return

        percentage = self.voltage_to_percentage(voltage)

        # Publish voltage
        v_msg = Float32()
        v_msg.data = voltage
        self.voltage_pub.publish(v_msg)

        # Publish percentage
        p_msg = Float32()
        p_msg.data = percentage
        self.percentage_pub.publish(p_msg)

        # Low battery warning
        low_msg = Bool()
        low_msg.data = percentage < self.low_battery_threshold
        self.low_battery_pub.publish(low_msg)

        self.get_logger().info(
            f"Voltage: {voltage:.2f}V | Battery: {percentage:.1f}% | Low: {low_msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()