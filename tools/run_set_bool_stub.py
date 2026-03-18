#!/usr/bin/env python3
"""Minimal SetBool stub server for decision-tree simulation tests."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool


class SetBoolStub(Node):
    def __init__(self) -> None:
        super().__init__("decision_set_bool_stub")
        self.create_service(SetBool, "set_bool", self.handle_set_bool)
        self.get_logger().info("Serving /set_bool stub")

    def handle_set_bool(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        response.success = True
        response.message = f"stub accepted data={request.data}"
        self.get_logger().info(
            f"Handled /set_bool request: data={str(request.data).lower()}"
        )
        return response


def main() -> None:
    rclpy.init()
    node = SetBoolStub()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
