import glob
import os
import platform

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class DriveWatchdogNode(Node):
    """Watches for a USB drive (identified by volume label) and publishes an
    emergency-stop signal when it disappears.

    Works unmodified on both macOS (sim, dev machine) and Linux (real robot)
    by searching the platform-appropriate mount locations for the label.
    """

    def __init__(self):
        super().__init__('drive_watchdog_node')

        self.drive_label = self.declare_parameter('drive_label', 'NO NAME').value
        self.emergency_stop_topic = self.declare_parameter(
            'emergency_stop_topic', '/emergency_stop'
        ).value
        self.poll_rate_hz = float(self.declare_parameter('poll_rate_hz', 2.0).value)

        self.stop_pub = self.create_publisher(Bool, self.emergency_stop_topic, 10)

        self.drive_path = None
        self.last_present = None  # None = unknown until first poll

        self.timer = self.create_timer(1.0 / max(1e-6, self.poll_rate_hz), self._poll)

        self.get_logger().info(
            f"Watching for drive labeled '{self.drive_label}'; "
            f"publishing Bool on {self.emergency_stop_topic} (True = stop)"
        )

    def _candidate_paths(self):
        label = self.drive_label
        if platform.system() == 'Darwin':
            return [os.path.join('/Volumes', label)]

        # Linux (real robot / dev machine)
        user = os.environ.get('USER', '')
        candidates = []
        for base in ('/media', '/run/media'):
            if user:
                candidates.append(os.path.join(base, user, label))
            candidates.extend(glob.glob(os.path.join(base, '*', label)))
        return candidates

    def _find_mounted_path(self):
        for path in self._candidate_paths():
            if os.path.exists(path):
                return path
        return None

    def _poll(self):
        if self.drive_path is not None:
            present = os.path.exists(self.drive_path)
            if not present:
                self.drive_path = None
        else:
            self.drive_path = self._find_mounted_path()
            present = self.drive_path is not None

        if present != self.last_present:
            if present:
                self.get_logger().info(f'Drive detected at {self.drive_path}.')
            else:
                self.get_logger().error('Drive removed! Publishing emergency stop.')
            self.last_present = present

        self.stop_pub.publish(Bool(data=not present))


def main(args=None):
    rclpy.init(args=args)
    node = DriveWatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
