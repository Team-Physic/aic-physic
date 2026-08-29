"""Composition root for the FinalPolicy browser dashboard."""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
import threading

import rclpy
from controllers import DashboardNode
from models import DashboardState
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from views import serve


def _bounded_int(minimum: int, maximum: int):
    def parse(value: str) -> int:
        parsed = int(value)
        if not minimum <= parsed <= maximum:
            raise argparse.ArgumentTypeError(
                f"must be between {minimum} and {maximum}"
            )
        return parsed

    return parse


def _parse_args(args: list[str] | None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Stream FinalPolicy debug topics to a browser"
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("POLICY_DASHBOARD_HOST", "127.0.0.1"),
        help="HTTP bind address (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=_bounded_int(1, 65535),
        default=os.environ.get("POLICY_DASHBOARD_PORT", "8080"),
        help="HTTP port (default: 8080)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=_bounded_int(1, 100),
        default=os.environ.get("POLICY_DASHBOARD_JPEG_QUALITY", "85"),
        help="JPEG quality from 1 to 100 (default: 85)",
    )
    parser.add_argument(
        "--cable-frame",
        default=os.environ.get("POLICY_DASHBOARD_CABLE_FRAME", ""),
        help="Cable tip TF override (default: derive from current task)",
    )
    argv = sys.argv[1:] if args is None else args
    return parser.parse_known_args(argv)


def main(args: list[str] | None = None) -> None:
    """Run the ROS controller and browser view until shutdown."""

    dashboard_args, ros_args = _parse_args(args)
    rclpy.init(
        args=ros_args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    state = DashboardState()
    node = DashboardNode(
        state,
        jpeg_quality=dashboard_args.jpeg_quality,
        cable_frame=dashboard_args.cable_frame,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin,
        name="policy-dashboard-ros",
        daemon=True,
    )
    spin_thread.start()
    try:
        asyncio.run(
            serve(state, dashboard_args.host, dashboard_args.port, node.get_logger())
        )
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
