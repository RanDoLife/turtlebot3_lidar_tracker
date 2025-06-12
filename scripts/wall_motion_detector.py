#!/usr/bin/env python3
# encoding: utf-8

import rospy
import numpy as np
from std_msgs.msg import Float32, String, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class WallMotionDetector:
    def __init__(self):
        rospy.init_node('wall_motion_detector')

        self.prev_distance = None
        self.threshold = 0.03  # 3 см
        self.fov_deg = 2.5     # ±2.5°

        self.dist_pub = rospy.Publisher('/wall_distance', Float32, queue_size=1)
        self.motion_pub = rospy.Publisher('/wall_motion', String, queue_size=1)
        self.marker_pub = rospy.Publisher('/movement_marker', Marker, queue_size=1)

        rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        fov_rad = np.radians(self.fov_deg)
        mask = np.abs(angles) <= fov_rad
        sector = ranges[mask]

        # Удалим NaN и inf
        sector = sector[np.isfinite(sector)]

        if sector.size == 0:
            rospy.logwarn("Нет валидных значений в секторе ±2.5°")
            return

        distance = float(np.median(sector))  # медиана лучше в шумной среде
        self.dist_pub.publish(Float32(data=distance))

        # Определяем движение
        movement, delta = self.detect_movement(distance)
        self.motion_pub.publish(String(data=movement.upper()))
        self.publish_marker(movement)

        rospy.loginfo(f"{movement.upper()} | Distance: {distance:.3f} m | Δ: {delta:.3f} m")

    def detect_movement(self, current: float):
        if self.prev_distance is None:
            self.prev_distance = current
            return "stable", 0.0

        delta = self.prev_distance - current
        self.prev_distance = current

        if abs(delta) <= self.threshold:
            return "stable", delta
        elif delta > 0:
            return "approaching", delta
        else:
            return "receding", delta

    def publish_marker(self, movement: str):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "motion"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points = [
            Point(0, 0, 1.0),
            Point(-0.3 if movement == "approaching" else 0.3, 0, 1.0)
        ]

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        if movement == "approaching":
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # красный
        elif movement == "receding":
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # зелёный
        else:
            marker.color = ColorRGBA(0.5, 0.5, 0.5, 1.0)  # серый

        self.marker_pub.publish(marker)

def main():
    WallMotionDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
