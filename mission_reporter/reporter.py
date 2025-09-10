#!/usr/bin/env python3

import utm
import json
import requests

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from tf2_ros.buffer import Buffer
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros.transform_listener import TransformListener


# SERVER_URL = "https://robot-tracker.felk.cvut.cz/api/update_data"
SERVER_URL = "http://localhost:5001/api/update_data"


class MissionReporter(Node):
    def __init__(self):
        super().__init__("mission_reporter")
        self.declare_parameter("robot_id", "helhest")
        robot_id = self.get_parameter("robot_id").get_parameter_value().string_value

        self.data = {"robot_id": robot_id}
        self.current_waypoint = 0
        self.pose_gps = None
        self.pose_ekf = None

        # Dummy initial position to have something to send before first GPS message
        self.pose_gps = {"lat": 49.08400100470894, "lon": 17.337373046610274}
        self.pose_ekf = {"lat": 49.08400100470894, "lon": 17.337373046610274}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_sub = self.create_subscription(Path, "/path", self._path_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_callback, 10
        )
        self.ekf_sub = self.create_subscription(
            NavSatFix, "/gps/filtered", self._ekf_callback, 10
        )

    def _send_data_url(self, msg_type):
        data = self.data
        if msg_type == "path":
            data["mission"] = {
                "waypoints": self.waypoints_gps,
                "current_waypoint_index": self.current_waypoint,
            }
            if self.pose_gps and self.pose_ekf:
                data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
            else:
                data["position"] = {"gps": [], "ekf": []}
        elif msg_type == "update":
            data["mission"] = {
                "current_waypoint_index": self.current_waypoint,
            }
            if self.pose_gps and self.pose_ekf:
                data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
            else:
                data["position"] = {"gps": [], "ekf": []}
        else:
            self.get_logger().error(f"Unknown message type: {msg_type}")

        try:
            response = requests.post(
                SERVER_URL,
                headers={"Content-Type": "application/json"},
                data=json.dumps(data),
            )

            # Check response
            if response.status_code == 200:
                self.get_logger().info("Data sent successfully!")
            elif response.status_code == 202:
                self.get_logger().warn("Failed with status: Missing path.")
                self.send_data_url("path")
            else:
                self.get_logger().error(
                    f"Failed to send data. Status code: {response.status_code}"
                )
                self.get_logger().error(response.text)

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error sending request: {e}")

    def _convert_path_latlon(self, msg):
        waypoints_gps = []
        try:
            transform = self.tf_buffer.lookup_transform(
                "utm",
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=5.0),
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform utm to {msg.header.frame_id}: {ex}"
            )
            return None

        for pose in msg.poses:
            pose_utm = do_transform_pose_stamped(pose, transform)
            ret_gps = utm.to_latlon(
                pose_utm.pose.position.x, pose_utm.pose.position.y, 33, "U"
            )
            waypoints_gps.append({"lat": ret_gps[0], "lon": ret_gps[1]})

        return waypoints_gps

    def _gps_callback(self, msg):
        self.pose_gps = {"lat": msg.latitude, "lon": msg.longitude}

    def _ekf_callback(self, msg):
        self.pose_ekf = {"lat": msg.latitude, "lon": msg.longitude}

    def _path_callback(self, msg):
        waypoints = self._convert_path_latlon(msg)
        if waypoints is None:
            return
        self.waypoints_gps = waypoints[::10]
        self._send_data_url("path")


def main():
    rclpy.init()
    reporter = MissionReporter()
    rclpy.spin(reporter)
    reporter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
