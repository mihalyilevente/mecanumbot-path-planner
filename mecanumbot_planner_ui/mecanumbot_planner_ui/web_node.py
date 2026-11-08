"""Entry point: MultiThreadedExecutor + Flask daemon thread."""

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from mecanumbot_planner_ui.app import create_app
from mecanumbot_planner_ui.ros_bridge import RosBridge


def main(args=None) -> None:
    rclpy.init(args=args)
    bridge = RosBridge()
    app = create_app(bridge)

    flask_thread = threading.Thread(
        target=lambda: app.run(
            host=bridge.host,
            port=bridge.port,
            threaded=True,
            use_reloader=False,
        ),
        name='planner_ui_flask',
        daemon=True,
    )
    flask_thread.start()
    bridge.get_logger().info(
        f'Planner UI listening on http://{bridge.host}:{bridge.port}')

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(bridge)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
