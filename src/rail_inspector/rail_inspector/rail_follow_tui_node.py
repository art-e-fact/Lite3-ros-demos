"""Entry point: ROS node + Textual TUI."""

import rclpy

from rail_inspector.rail_follow_tui.app import RailFollowTuiApp
from rail_inspector.rail_follow_tui.ros_state import RosState


def main(args=None) -> None:
    rclpy.init(args=args)
    ros = RosState()
    app = RailFollowTuiApp(ros)
    try:
        app.run()
    finally:
        ros.publish_stop()
        ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
