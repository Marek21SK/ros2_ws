#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker
import rclpy, math, cv2, time
import numpy as np
from cv_bridge import CvBridge

bridge = CvBridge()

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.subscription = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.move(0.5, 0.0)

        self.last_log_time = self.get_clock().now()
        self.log_interval = 5.0

    def laser_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        v = msg.ranges[len(msg.ranges) // 2]

        if elapsed >= self.log_interval:
            self.get_logger().info(f"Vzdialenosť vpredu: {v:.2f} m")
            self.last_log_time = now

        if v < 0.25:
            self.get_logger().info(f"Zastavujem, prekážka vo vzdialenosti: {v:.2f} m")
            self.move(0.0, 0.0)

    def move(self, l, a):
        msg = Twist()
        msg.linear.x = l
        msg.angular.z = a
        self.publisher_.publish(msg)


class ProcessImage(Node):
    def __init__(self):
        super().__init__("tracking_publisher")

        self.subscription_img = self.create_subscription(Image, "/image", self.listener_callback, 10)
        self.subscription_laser = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.publisher_marker = self.create_publisher(Marker, "/marker", 10)

        self.running = True
        self.closest_distance = float("inf")

        # Časovanie pre logy
        self.last_log_time_image = self.get_clock().now()
        self.last_log_time_contours = self.get_clock().now()
        self.last_log_time_events = self.get_clock().now()
        self.last_log_time_lines = self.get_clock().now()
        self.log_interval_image = 5.0
        self.log_interval_contours = 2.0
        self.log_interval_events = 3.0
        self.log_interval_lines = 2.0

        # Sledovanie kužeľu
        self.last_cone_time = time.time()
        self.cone_lost_timeout = 2.5  # čas, po ktorom robot zastaví ak stratí kužeľ
        self.last_offset = 0

        self.mode = "cone"  # "cone" alebo "lines"

    def laser_callback(self, msg):
        front_index = len(msg.ranges) // 2
        distance = msg.ranges[front_index]
        if not math.isnan(distance) and not math.isinf(distance):
            self.closest_distance = distance
        else:
            self.closest_distance = float("inf")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed_image = (now - self.last_log_time_image).nanoseconds / 1e9

        img = bridge.imgmsg_to_cv2(msg, "bgra8")
        # img = cv2.flip(img, 1) #Zrkadlenie obrazu

        if elapsed_image >= self.log_interval_image:
            self.get_logger().info("Obrázok prijatý.")
            self.last_log_time_image = now

        if self.mode == "lines":
            result = self.search_lines(img)
        elif self.mode == "cone":
            result = self.search_cone(img)
        else:
            result = img
            self.get_logger().warn("Neznámy režim detekcie.")

        cv2.imshow("Frame", result)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Ukončujem...")
            self.running = False
        elif key == ord("c"):
            self.mode = "cone"
            self.get_logger().info("Prepnuté na režim: cone")
        elif key == ord("l"):
            self.mode = "lines"
            self.get_logger().info("Prepnuté na režim: lines")

    def search_lines(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        img_bin = cv2.Canny(gray, 30, 100)
        lines = cv2.HoughLines(img_bin, 1, np.pi / 180, 100, None, 0, 0)
        result = cv2.cvtColor(img_bin, cv2.COLOR_GRAY2BGR)

        now = self.get_clock().now()
        elapsed = (now - self.last_log_time_lines).nanoseconds / 1e9
        if elapsed >= self.log_interval_lines:
            self.get_logger().info("Detekujem priamky...")
            self.last_log_time_lines = now

        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a, b = math.cos(theta), math.sin(theta)
                x0, y0 = a * rho, b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
                cv2.line(result, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

        return result

    def search_cone(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_orange = cv2.inRange(img_hsv, (0, 50, 50), (40, 255, 255))
        cv2.imshow("Maska", mask_orange)
        contours, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        result = img.copy()

        now = self.get_clock().now()
        elapsed_contours = (now - self.last_log_time_contours).nanoseconds / 1e9
        elapsed_events = (now - self.last_log_time_events).nanoseconds / 1e9

        twist_msg = Twist()
        cone_detected = False

        if elapsed_contours >= self.log_interval_contours:
            self.get_logger().info(f"[Kužeľ] Počet kontúr: {len(contours)}")
            self.last_log_time_contours = now

        if len(contours) > 0:
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            area = cv2.contourArea(contours[0])
            if elapsed_contours >= self.log_interval_contours:
                self.get_logger().info(f"[Kužeľ] Plocha najväčšej kontúry: {area:.2f}")

            if area > 100:
                cone_detected = True
                self.last_cone_time = time.time()

                cv2.drawContours(result, [contours[0]], -1, (0, 255, 0), 3)
                m = cv2.moments(contours[0])
                if m["m00"] != 0:
                    cx = int(m["m10"] / m["m00"])
                    cy = int(m["m01"] / m["m00"])
                    cv2.circle(result, (cx, cy), 10, (255, 0, 0), -1)

                    image_width = img.shape[1]
                    offset = cx - image_width // 2
                    self.last_offset = offset
                    tolerancia = 40

                    if self.closest_distance < 0.35:
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.0
                        if elapsed_events >= self.log_interval_events:
                            self.get_logger().info(f"[Kužeľ] Zastavenie – prekážka {self.closest_distance:.2f} m")
                            self.last_log_time_events = now

                        # Publikovanie MARKER (work in progress)
                        # marker = Marker()
                        # marker.header.frame_id = "odom"
                        # marker.header.stamp = now.to_msg()
                        # marker.ns = "cones"
                        # marker.id = 0
                        # marker.type = Marker.SPHERE
                        # marker.action = Marker.ADD
                        # marker.pose.position.x = self.closest_distance
                        # marker.pose.position.y = offset * (self.closest_distance / (image_width // 2))
                        # marker.pose.position.z = 0.1
                        # marker.pose.orientation.w = 1.0
                        # marker.scale.x = 0.15
                        # marker.scale.y = 0.15
                        # marker.scale.z = 0.15
                        # marker.color.a = 1.0
                        # marker.color.r = 1.0
                        # marker.color.g = 0.5
                        # marker.color.b = 0.0
                        # self.publisher_marker.publish(marker)

                    else:
                        if abs(offset) < tolerancia:
                            twist_msg.linear.x = 0.20
                            twist_msg.angular.z = 0.0
                        elif offset > 0:
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = -0.2
                        else:
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.2

                    if elapsed_events >= self.log_interval_events:
                        self.get_logger().info(f"[Kužeľ] Offset: {offset}, vzdialenosť: {self.closest_distance:.2f}")
                        self.last_log_time_events = now

        if not cone_detected and (time.time() - self.last_cone_time) < self.cone_lost_timeout:
            twist_msg.linear.x = 0.0
            elapsed_since_lost = time.time() - self.last_cone_time
            if (elapsed_since_lost % 2) < 1:
                twist_msg.angular.z = -0.2 if self.last_offset > 0 else 0.2
            else:
                twist_msg.angular.z = 0.2 if self.last_offset > 0 else -0.2
            if elapsed_events >= self.log_interval_events:
                self.get_logger().info("[Kužeľ] Kužeľ nie je detekovaný, hľadám ho.")
                self.last_log_time_events = now

        elif not cone_detected:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            if elapsed_events >= self.log_interval_events:
                self.get_logger().info("[Kužeľ] Kužeľ nenájdený dlhší čas, zastavujem.")
                self.last_log_time_events = now

        self.publisher_.publish(twist_msg)
        return result

def main(args=None):
    rclpy.init(args=args)
    test_node = ProcessImage()  # MinimalPublisher()

    while rclpy.ok() and test_node.running:
        rclpy.spin_once(test_node, timeout_sec=0.1)

    test_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
