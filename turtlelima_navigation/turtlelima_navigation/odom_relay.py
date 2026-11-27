"""Republish the Create 3 odometry with a reliable QoS.

The Create 3 publishes /odom as best effort, but bt_navigator and
controller_server subscribe with a reliable QoS (not configurable in Jazzy),
so they never receive it. This node bridges the two.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data


class OdomRelay(Node):

    def __init__(self):
        super().__init__('odom_relay')
        self._pub = self.create_publisher(Odometry, 'odom_out', QoSProfile(depth=10))
        self._sub = self.create_subscription(
            Odometry, 'odom_in', self._pub.publish, qos_profile_sensor_data)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
