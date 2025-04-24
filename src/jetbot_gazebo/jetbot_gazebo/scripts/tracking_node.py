#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, Point
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker
import rclpy, math, cv2, time
import numpy as np
import tf2_geometry_msgs
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer

bridge = CvBridge()

class ProcessImage(Node):
    def __init__(self):
        super().__init__("tracking_publisher")

        self.subscription_img = self.create_subscription(Image, "/image", self.listener_callback, 10)
        self.subscription_laser = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.publisher_marker = self.create_publisher(Marker, "/marker", 10)

        self.running = True
        self.closest_distance = float("inf")
        self.lidar_ranges = []

        # Inicializácia tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.get_logger().info("tf_buffer a tf_listener inicializované")

        # Časovanie pre výpis logov do konzole
        self.last_log_time_image = self.get_clock().now()
        self.last_log_time_contours = self.get_clock().now()
        self.last_log_time_events = self.get_clock().now()
        self.last_log_time_lines = self.get_clock().now()
        self.last_log_time_timeout = self.get_clock().now()
        self.log_interval_image = 5.0
        self.log_interval_contours = 2.0
        self.log_interval_events = 3.0
        self.log_interval_lines = 2.0
        self.log_interval_timeout = 1.0

        # Sledovanie kužeľa
        self.last_cone_time = 0.0
        self.cone_lost_timeout = 5.0  # čas, po ktorom robot zastaví ak stratí kužeľ
        self.last_offset = 0

        # Hľadanie kužeľa
        self.searching_cone = False
        self.search_angle = 0.0
        self.search_start_time = 0.0
        self.search_state = "idle"
        self.timeout_start_time = 0.0

        # Označenie kužeľov
        self.marked_cones = []
        self.cone_id = 1
        self.distance_cones = 0.5

        # Módy skriptu
        self.mode = "normal"  # "cone", "lines", "normal"

    def laser_callback(self, msg):
        self.lidar_ranges = msg.ranges
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
            # self.get_logger().info("Obrázok prijatý.")
            self.last_log_time_image = now

        if self.mode == "lines":
            result = self.search_lines(img)
        elif self.mode == "cone":
            result = self.search_cone(img)
        elif self.mode == "normal":
            result = img
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
        elif key == ord("n"):
            self.mode = "normal"
            self.get_logger().info("Prepnuté na režim: normal")
        elif key == ord("r"):
            self.search_state = "idle"
            self.searching_cone = False
            self.get_logger().info("Reštartujem hľadanie kužeľa")

    def search_lines(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        median_intensity = np.median(gray)
        lower_threshold = int(max(0, (1.0 - 0.33) * median_intensity))
        upper_threshold = int(min(255, (1.0 + 0.33) * median_intensity))
        img_bin = cv2.Canny(gray, lower_threshold, upper_threshold)
        lines = cv2.HoughLines(img_bin, 1, np.pi / 180, 100, None, 0, 0)
        # result = cv2.cvtColor(img_bin, cv2.COLOR_GRAY2BGR)
        result = img.copy()

        now = self.get_clock().now()
        elapsed = (now - self.last_log_time_lines).nanoseconds / 1e9
        if elapsed >= self.log_interval_lines:
            # self.get_logger().info("Detekujem priamky...")
            self.last_log_time_lines = now

        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a, b = math.cos(theta), math.sin(theta)
                x0, y0 = a * rho, b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
                angle = theta * 180 / np.pi
                if 45 < angle < 135:
                    color = (0, 0, 255)
                else:
                    color = (255, 0, 0)
                cv2.line(result, pt1, pt2, color, 3, cv2.LINE_AA)

        return result

    def new_cone (self, x, y):
        for cone in self.marked_cones:
            dist = math.sqrt((x - cone[0])**2 + (y - cone[1])**2)
            if dist < self.distance_cones:
                return False
        return True

    def search_cone(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_orange = cv2.inRange(img_hsv, (0, 30, 30), (20, 255, 255))
        mask_orange = cv2.GaussianBlur(mask_orange, (5, 5), 0)
        kernel = np.ones((3, 3), np.uint8)
        mask_orange = cv2.dilate(mask_orange, kernel, iterations=1)
        cv2.imshow("Maska", mask_orange)
        contours, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        result = img.copy()

        # Výpočet časovania od posledného logu
        now = self.get_clock().now()
        elapsed_contours = (now - self.last_log_time_contours).nanoseconds / 1e9
        elapsed_events = (now - self.last_log_time_events).nanoseconds / 1e9
        elapsed_timeouts = (now - self.last_log_time_timeout).nanoseconds / 1e9

        twist_msg = Twist()
        cone_detected = False

        if elapsed_contours >= self.log_interval_contours:
            # self.get_logger().info(f"[Kužeľ] Počet kontúr: {len(contours)}")
            self.last_log_time_contours = now

        if len(contours) > 0:
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            area = cv2.contourArea(contours[0])
            if elapsed_contours >= self.log_interval_contours:
               # self.get_logger().info(f"[Kužeľ] Plocha najväčšej kontúry: {area:.2f}")
                self.last_log_time_contours = now

            min_area = 100 if self.closest_distance < 1.0 else 30
            if area > min_area:
                cone_detected = True
                self.last_cone_time = time.time()

                # Logovanie ak je kužeľ nájdený počas hľadania
                if self.search_state in ["turn_left", "turn_right"] and elapsed_events >= self.log_interval_events:
                    self.get_logger().info("[Kužeľ] Kužeľ nájdený pri hľadaní, nasledujem kužeľ")
                    self.last_log_time_events = now

                self.searching_cone = False  # Ak nájde kužeľ, hľadanie sa zastaví
                self.search_state = "idle"  # Reset stavu hľadania

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
                            self.get_logger().info(f"[Kužeľ] Zastavenie – vzdialenosť {self.closest_distance:.2f} m")
                            self.last_log_time_events = now

                        D = self.closest_distance
                        x_local = D
                        tan_fov_half = math.tan(math.radians(40))
                        y_local = -offset * (D * tan_fov_half) / (image_width / 2)
                        z_local = 0.1

                        if elapsed_events >= self.log_interval_events:
                           # self.get_logger().info(f"cx: {cx}, image_width: {image_width}, offset: {offset}, x_local: {x_local:.2f}, y_local: {y_local:.2f}")
                            self.last_log_time_events = now

                        now = self.get_clock().now()
                        point_local = PointStamped()
                        point_local.header.frame_id = "base_link"
                        point_local.header.stamp = now.to_msg()
                        point_local.point.x = x_local
                        point_local.point.y = y_local
                        point_local.point.z = z_local

                        try:
                            if self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time()):
                                transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
                                point_odom = tf2_geometry_msgs.do_transform_point(point_local, transform)

                                if self.new_cone(point_odom.point.x, point_odom.point.y):
                                    # Publikovanie MARKER
                                    marker = Marker()
                                    marker.header.frame_id = "odom"
                                    marker.header.stamp = now.to_msg()
                                    marker.ns = "cones"
                                    marker.id = self.cone_id
                                    marker.type = Marker.SPHERE
                                    marker.action = Marker.ADD
                                    marker.pose.position = point_odom.point
                                    marker.pose.orientation.w = 1.0
                                    marker.scale.x = 0.15
                                    marker.scale.y = 0.15
                                    marker.scale.z = 0.15
                                    marker.color.a = 1.0
                                    marker.color.r = 1.0
                                    marker.color.g = 0.5
                                    marker.color.b = 0.0
                                    self.publisher_marker.publish(marker)

                                    self.marked_cones.append((point_odom.point.x, point_odom.point.y))
                                    self.get_logger().info(f"Označený nový kužeľ s ID {self.cone_id} na pozícii ({point_odom.point.x:.2f}, {point_odom.point.y:.2f})")
                                    self.cone_id += 1

                                    self.searching_cone = True
                                    self.search_start_time = time.time()
                                    self.search_state = "turn_left"  # Spustí nové hľadanie
                            else:
                                self.get_logger().warn("Transformácia z base_link do odom nie je dostupná")
                        except Exception as e:
                            self.get_logger().warn(f"Nemôžem transformovať bod: {e}")

                    else:
                        if abs(offset) < tolerancia:
                            twist_msg.linear.x = 0.20
                            twist_msg.angular.z = 0.0
                        else:
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = -0.2 if offset > 0 else 0.2

                    if elapsed_events >= self.log_interval_events:
                       # self.get_logger().info(f"[Kužeľ] Offset: {offset}, vzdialenosť: {self.closest_distance:.2f}")
                        self.last_log_time_events = now

        # Hľadanie kužeľa ak nie je detekovaný
        if not cone_detected:
            twist_msg.linear.x = 0.0
            angular_speed = 0.25  # Rýchlosť otáčania

            if self.search_state == "idle":
                self.searching_cone = True
                self.search_start_time = time.time()
                self.search_angle = 0.0
                self.search_state = "turn_left"
                if elapsed_events >= self.log_interval_events:
                    self.get_logger().info("[Kužeľ] Začínam hľadanie – otáčam 50° doľava")
                    self.last_log_time_events = now

            elif self.search_state == "turn_left":
                twist_msg.angular.z = angular_speed
                self.search_angle += angular_speed * 0.1
                if self.search_angle >= math.radians(50):
                    self.search_angle = 0.0
                    self.search_state = "turn_right"
                    if elapsed_events >= self.log_interval_events:
                        self.get_logger().info("[Kužeľ] Otočené 50° doľava, otáčam 100° doprava")
                        self.last_log_time_events = now

            elif self.search_state == "turn_right":
                twist_msg.angular.z = -angular_speed
                self.search_angle += angular_speed * 0.1
                if self.search_angle >= math.radians(100):
                    self.search_angle = 0.0
                    self.search_state = "timeout"
                    self.timeout_start_time = time.time()
                    if elapsed_events >= self.log_interval_events:
                        self.get_logger().info("[Kužeľ] Otočené 100° doprava, začínam 5s timeout")
                        self.last_log_time_events = now

            elif self.search_state == "timeout":
                twist_msg.angular.z = 0.0
                if (time.time() - self.timeout_start_time) >= self.cone_lost_timeout:
                    self.search_state = "stopped"
                    if elapsed_events >= self.log_interval_events:
                        self.get_logger().info("[Kužeľ] Kužeľ nenájdený, zastavujem.")
                        self.last_log_time_events = now
                else:
                    if elapsed_timeouts >= self.log_interval_timeout:
                        self.get_logger().info(f"[Kužeľ] Čakám na timeout, zostáva {self.cone_lost_timeout - (time.time() - self.timeout_start_time):.1f}s")
                        self.last_log_time_timeout = now

            elif self.search_state == "stopped":
                twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)
        return result

def main(args=None):
    rclpy.init(args=args)
    test_node = ProcessImage()

    while rclpy.ok() and test_node.running:
        rclpy.spin_once(test_node, timeout_sec=0.1)

    test_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
