import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros
from rclpy.duration import Duration
import os
import time
from std_msgs.msg import String

class ConeDetector(Node):
    def __init__(self):
        super().__init__("cone_detector")

        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
        self.marker_array_publisher = self.create_publisher(MarkerArray, 'cone_marker_array', 10)
        self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.web_logger_pub = self.create_publisher(String, 'web_logger', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_subscription = self.create_subscription(
            Image,
            "oak/rgb/image_raw",
            self.detect_cone,
            10
        )
        self.markers = {}
        self.cone_detected_count = 0
        self.detection_threshold = 6  # Set your detection threshold here
        self.marker_id = 0
        self.save_directory = "/photos"
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

    def is_marker_nearby(self):
        try:
            if not self.markers:
                return False
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0)):
                return False

            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
            robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            for marker_id, marker_position in self.markers.items():
                distance = np.sqrt(
                    (robot_position[0] - marker_position[0])**2 +
                    (robot_position[1] - marker_position[1])**2 +
                    (robot_position[2] - marker_position[2])**2
                )
                if distance < 2.0:  # Define a suitable distance threshold
                    return True
            return False
        except Exception:
            return False

    def detect_cone(self, msg):
        if self.is_marker_nearby():
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError:
            return

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # Define strict HSV ranges for orange, red, and yellow
        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_yellow = np.array([25, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        # Create masks for the color ranges
        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
        red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Combine the masks
        mask = cv2.bitwise_or(cv2.bitwise_or(orange_mask, red_mask1), cv2.bitwise_or(red_mask2, yellow_mask))

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        detected = False
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 70000:  # Increase the area threshold for larger objects
                x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
                cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.publish_cone_marker()
                detected = True

        if detected:
            self.cone_detected_count += 1
            log_msg = String()
            log_msg.data = f"CONE DETECTED AND IMAGE SAVED:"
            self.web_logger_pub.publish(log_msg)

        if self.cone_detected_count >= self.detection_threshold:
            # Create a success message on the image
            cv2.putText(bgr_image, "SUCCESS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        try:
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0)):
                return

            # Get robot position and add to image
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
            robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            position_text = f"Position: x={robot_position[0]:.2f}, y={robot_position[1]:.2f}, z={robot_position[2]:.2f}"
            cv2.putText(bgr_image, position_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Save the processed image with a unique name
            image_path = os.path.join(self.save_directory, f"{position_text}_{self.marker_id}.jpg")
            cv2.imwrite(image_path, bgr_image)



            processed_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
            self.image_publisher.publish(processed_image_msg)
            self.get_logger().info("Published processed image to 'processed_image' topic.")
            time.sleep(1)
        except CvBridgeError:
            pass

    def publish_cone_marker(self):
        try:
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0)):
                return

            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.position.x = trans.transform.translation.x
            marker.pose.position.y = trans.transform.translation.y
            marker.pose.position.z = trans.transform.translation.z
            marker.pose.orientation = trans.transform.rotation
            marker.id = self.marker_id
            self.marker_id += 1
            self.markers[marker.id] = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
            self.marker_publisher.publish(marker)
            self.publish_marker_array()
        except Exception:
            pass

    def publish_marker_array(self):
        marker_array = MarkerArray()
        for marker_id, position in self.markers.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.id = marker_id
            marker_array.markers.append(marker)
        self.marker_array_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ConeDetector()
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker, MarkerArray
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import tf2_ros
# from rclpy.duration import Duration
# import os
# import time

# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")
#         self.get_logger().info("Initializing ConeDetector node")

#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
#         self.marker_array_publisher = self.create_publisher(MarkerArray, 'cone_marker_array', 10)
#         self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )
#         self.markers = {}
#         self.cone_detected_count = 0
#         self.detection_threshold = 6  # Set your detection threshold here
#         self.marker_id = 0
#         self.save_directory = "/photos"

    

#     def is_marker_nearby(self):
#         try:
#             if not self.markers:
#                 return False
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
#             for marker_id, marker_position in self.markers.items():
#                 distance = np.sqrt(
#                     (robot_position[0] - marker_position[0])**2 +
#                     (robot_position[1] - marker_position[1])**2 +
#                     (robot_position[2] - marker_position[2])**2
#                 )
#                 if distance < 2.0:  # Define a suitable distance threshold
#                    # self.get_logger().info(f"Existing marker {marker_id} detected nearby, skipping cone detection")
#                     return True
#             return False
#         except Exception as e:
#           #  self.get_logger().error(f'Could not check for nearby marker: {e}')
#             return False

#     def detect_cone(self, msg):
#         if self.is_marker_nearby():
#             return

#         try:
#             bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#           #  self.get_logger().error(f"Could not convert image: {e}")
#             return

#       #  self.get_logger().info(f"Image size: {bgr_image.shape}")

#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#         # Define strict HSV ranges for orange, red, and yellow
#         lower_orange = np.array([10, 100, 100])
#         upper_orange = np.array([25, 255, 255])
#         lower_red1 = np.array([0, 100, 100])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([160, 100, 100])
#         upper_red2 = np.array([180, 255, 255])
#         lower_yellow = np.array([25, 100, 100])
#         upper_yellow = np.array([35, 255, 255])

#         # Create masks for the color ranges
#         orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
#         yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

#         # Combine the masks
#         mask = cv2.bitwise_or(cv2.bitwise_or(orange_mask, red_mask1), cv2.bitwise_or(red_mask2, yellow_mask))

#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         detected = False
#         for i in range(1, num_labels):
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 70000:  # Increase the area threshold for larger objects
#                 x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
#                 cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 self.publish_cone_marker()
#               #  self.get_logger().info("ORANGE/RED/YELLOW OBJECT SEEN")
#                 detected = True

#         if detected:
#             self.cone_detected_count += 1

#         if self.cone_detected_count >= self.detection_threshold:
#             # Create a success message on the image
#             cv2.putText(bgr_image, "SUCCESS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         try:
#             # Get robot position and add to image
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
#             position_text = f"Position: x={robot_position[0]:.2f}, y={robot_position[1]:.2f}, z={robot_position[2]:.2f}"
#             cv2.putText(bgr_image, position_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#             # Save the processed image with a unique name
#             image_path = os.path.join(self.save_directory, f"processed_image_{self.marker_id}.jpg")
#             cv2.imwrite(image_path, bgr_image)
#          #   self.get_logger().info(f"Processed image saved at: {image_path}")

#             processed_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
#             self.image_publisher.publish(processed_image_msg)
#             time.sleep(1)
#         except CvBridgeError as e:
#          self.get_logger().error(f"Could not convert processed image: {e}")

#     def publish_cone_marker(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = trans.transform.translation.x
#             marker.pose.position.y = trans.transform.translation.y
#             marker.pose.position.z = trans.transform.translation.z
#             marker.pose.orientation = trans.transform.rotation
#             marker.id = self.marker_id
#             self.marker_id += 1
#             self.markers[marker.id] = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
#             self.marker_publisher.publish(marker)
#             self.publish_marker_array()
#             self.log_markers()
#         except Exception as e:
#             self.get_logger().error(f'Could not transform map to base_link: {e}')

#     def publish_marker_array(self):
#         marker_array = MarkerArray()
#         for marker_id, position in self.markers.items():
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = position[0]
#             marker.pose.position.y = position[1]
#             marker.pose.position.z = position[2]
#             marker.id = marker_id
#             marker_array.markers.append(marker)
#         self.marker_array_publisher.publish(marker_array)

#     def log_markers(self):
#         marker_list = "\n".join([f"Marker ID: {marker_id}, Position: {position}" for marker_id, position in self.markers.items()])
#         self.get_logger().info(f"Markers:\n{marker_list}")

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = ConeDetector()
#         rclpy.spin(node)
#     except Exception as e:
#         rclpy.get_logger().error(f"Failed to initialize or spin the node: {e}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker, MarkerArray
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import tf2_ros
# from rclpy.duration import Duration
 
# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")
#         self.get_logger().info("Initializing ConeDetector node")
 
#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
#         self.marker_array_publisher = self.create_publisher(MarkerArray, 'cone_marker_array', 10)
#         self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )
#         self.markers = {}
#         self.cone_detected_count = 0
#         self.detection_threshold = 6  # Set your detection threshold here
#         self.marker_id = 0
 
#     def is_marker_nearby(self):
#         try:
#             if not self.markers:
#                 return False
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
#             for marker_id, marker_position in self.markers.items():
#                 distance = np.sqrt(
#                     (robot_position[0] - marker_position[0])**2 +
#                     (robot_position[1] - marker_position[1])**2 +
#                     (robot_position[2] - marker_position[2])**2
#                 )
#                 if distance < 1.0:  # Define a suitable distance threshold
#                     self.get_logger().info(f"Existing marker {marker_id} detected nearby, skipping cone detection")
#                     return True
#             return False
#         except Exception as e:
#             self.get_logger().error(f'Could not check for nearby marker: {e}')
#             return False
 
#     def detect_cone(self, msg):
#         if self.is_marker_nearby():
#             return
 
#         try:
#             bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert image: {e}")
#             return
 
#         self.get_logger().info(f"Image size: {bgr_image.shape}")
 
#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
 
#         # Define strict HSV ranges for orange, red, and yellow
#         lower_orange = np.array([10, 100, 100])
#         upper_orange = np.array([25, 255, 255])
#         lower_red1 = np.array([0, 100, 100])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([160, 100, 100])
#         upper_red2 = np.array([180, 255, 255])
#         lower_yellow = np.array([25, 100, 100])
#         upper_yellow = np.array([35, 255, 255])
 
#         # Create masks for the color ranges
#         orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
#         yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
 
#         # Combine the masks
#         mask = cv2.bitwise_or(cv2.bitwise_or(orange_mask, red_mask1), cv2.bitwise_or(red_mask2, yellow_mask))
 
#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
 
#         detected = False
#         for i in range(1, num_labels):
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 50000:  # Increase the area threshold for larger objects
#                 x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
#                 cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 self.publish_cone_marker()
#                 self.get_logger().info("ORANGE/RED/YELLOW OBJECT SEEN")
#                 detected = True
 
#         if detected:
#             self.cone_detected_count += 1
 
#         if self.cone_detected_count >= self.detection_threshold:
#             # Create a success message on the image
#             cv2.putText(bgr_image, "SUCCESS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
 
#         try:
#             processed_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
#             self.image_publisher.publish(processed_image_msg)
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert processed image: {e}")
 
#     def publish_cone_marker(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = trans.transform.translation.x
#             marker.pose.position.y = trans.transform.translation.y
#             marker.pose.position.z = trans.transform.translation.z
#             marker.pose.orientation = trans.transform.rotation
#             marker.id = self.marker_id
#             self.marker_id += 1
#             self.markers[marker.id] = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
#             self.marker_publisher.publish(marker)
#             self.publish_marker_array()
#             self.log_markers()
#         except Exception as e:
#             self.get_logger().error(f'Could not transform map to base_link: {e}')
 
#     def publish_marker_array(self):
#         marker_array = MarkerArray()
#         for marker_id, position in self.markers.items():
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = position[0]
#             marker.pose.position.y = position[1]
#             marker.pose.position.z = position[2]
#             marker.id = marker_id
#             marker_array.markers.append(marker)
#         self.marker_array_publisher.publish(marker_array)
 
#     def log_markers(self):
#         marker_list = "\n".join([f"Marker ID: {marker_id}, Position: {position}" for marker_id, position in self.markers.items()])
#         self.get_logger().info(f"Markers:\n{marker_list}")
 
# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = ConeDetector()
#         rclpy.spin(node)
#     except Exception as e:
#         rclpy.get_logger().error(f"Failed to initialize or spin the node: {e}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()
 
# if __name__ == '__main__':
#     main()
## just save the cv as a jpeg when it finds the cone


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker, MarkerArray
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import tf2_ros
# from rclpy.duration import Duration
# import os

# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")
#         self.get_logger().info("Initializing ConeDetector node")

#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
#         self.marker_array_publisher = self.create_publisher(MarkerArray, 'cone_marker_array', 10)
#         self.image_publisher = self.create_publisher(Image, 'cone_finder_image', 10)
        
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )
#         self.markers = {}
#         self.cone_detected_count = 0
#         self.detection_threshold = 6  # Set your detection threshold here
#         self.marker_id = 0

#     def is_marker_nearby(self):
#         try:
#             if not self.markers:
#                 return False
            
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            
#             for marker_id, marker_position in self.markers.items():
#                 distance = np.sqrt(
#                     (robot_position[0] - marker_position[0])**2 +
#                     (robot_position[1] - marker_position[1])**2 +
#                     (robot_position[2] - marker_position[2])**2
#                 )
#                 if distance < 1.0:  # Define a suitable distance threshold
#                     self.get_logger().info(f"Existing marker {marker_id} detected nearby, skipping cone detection")
#                     return True
#             return False
#         except Exception as e:
#             self.get_logger().error(f'Could not check for nearby marker: {e}')
#             return False

#     def detect_cone(self, msg):
#         if self.is_marker_nearby():
#             return

#         try:
#             bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert image: {e}")
#             return

#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#         # Define strict HSV ranges for orange, red, and yellow
#         lower_orange = np.array([10, 100, 100])
#         upper_orange = np.array([25, 255, 255])
#         lower_red1 = np.array([0, 100, 100])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([160, 100, 100])
#         upper_red2 = np.array([180, 255, 255])
#         lower_yellow = np.array([25, 100, 100])
#         upper_yellow = np.array([35, 255, 255])

#         # Create masks for the color ranges
#         orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
#         yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

#         # Combine the masks
#         mask = cv2.bitwise_or(cv2.bitwise_or(orange_mask, red_mask1), cv2.bitwise_or(red_mask2, yellow_mask))

#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         detected = False
#         for i in range(1, num_labels):
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 50000:  # Increase the area threshold for larger objects
#                 x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
#                 cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 self.publish_cone_marker()
#                 self.get_logger().info("ORANGE/RED/YELLOW OBJECT SEEN")
#                 detected = True

#         if detected:
#             self.cone_detected_count += 1
#             self.get_logger().info("Cone detected, saving image.")
#             # Save the current image as a JPEG file
#             try:
#                 image_filename = f"cone_detected_{self.cone_detected_count}.jpeg"
#                 image_path = os.path.join("/photos", image_filename)
#                 cv2.imwrite(image_path, bgr_image)
#                 self.get_logger().info(f"Saved image to {image_path}")
#             except Exception as e:
#                 self.get_logger().error(f"Could not save image: {e}")

#         if self.cone_detected_count >= self.detection_threshold:
#             # Create a success message on the image
#             cv2.putText(bgr_image, "SUCCESS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         # Add robot location to the image
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
#             location_text = f"Location: ({robot_position[0]:.2f}, {robot_position[1]:.2f}, {robot_position[2]:.2f})"
#             cv2.putText(bgr_image, location_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#         except Exception as e:
#             self.get_logger().error(f'Could not get robot location: {e}')

#         try:
#             processed_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
#             self.image_publisher.publish(processed_image_msg)
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert processed image: {e}")

#     def publish_cone_marker(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = trans.transform.translation.x
#             marker.pose.position.y = trans.transform.translation.y
#             marker.pose.position.z = trans.transform.translation.z
#             marker.pose.orientation = trans.transform.rotation
#             marker.id = self.marker_id
#             self.marker_id += 1
#             self.markers[marker.id] = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
#             self.marker_publisher.publish(marker)
#             self.publish_marker_array()
#             self.log_markers()
#         except Exception as e:
#             self.get_logger().error(f'Could not transform map to base_link: {e}')

#     def publish_marker_array(self):
#         marker_array = MarkerArray()
#         for marker_id, position in self.markers.items():
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = position[0]
#             marker.pose.position.y = position[1]
#             marker.pose.position.z = position[2]
#             marker.id = marker_id
#             marker_array.markers.append(marker)
#         self.marker_array_publisher.publish(marker_array)

#     def log_markers(self):
#         marker_list = "\n".join([f"Marker ID: {marker_id}, Position: {position}" for marker_id, position in self.markers.items()])
#         self.get_logger().info(f"Markers:\n{marker_list}")

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = ConeDetector()
#         rclpy.spin(node)
#     except Exception as e:
#         rclpy.get_logger().error(f"Failed to initialize or spin the node: {e}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker, MarkerArray
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import tf2_ros
# from rclpy.duration import Duration

# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")
#         self.get_logger().info("Initializing ConeDetector node")

#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
#         self.marker_array_publisher = self.create_publisher(MarkerArray, 'cone_marker_array', 10)
#         self.image_publisher = self.create_publisher(Image, 'cone_finder_image', 10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )
#         self.markers = {}
#         self.cone_detected_count = 0
#         self.detection_threshold = 6  # Set your detection threshold here
#         self.marker_id = 0

#     def is_marker_nearby(self):
#         try:
#             if not self.markers:
#                 return False
            
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            
#             for marker_id, marker_position in self.markers.items():
#                 distance = np.sqrt(
#                     (robot_position[0] - marker_position[0])**2 +
#                     (robot_position[1] - marker_position[1])**2 +
#                     (robot_position[2] - marker_position[2])**2
#                 )
#                 if distance < 1.0:  # Define a suitable distance threshold
#                     self.get_logger().info(f"Existing marker {marker_id} detected nearby, skipping cone detection")
#                     return True
#             return False
#         except Exception as e:
#             self.get_logger().error(f'Could not check for nearby marker: {e}')
#             return False

#     def detect_cone(self, msg):
#         if self.is_marker_nearby():
#             return

#         try:
#             bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert image: {e}")
#             return

#         self.get_logger().info(f"Image size: {bgr_image.shape}")

#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#         # Define strict HSV ranges for orange, red, and yellow
#         lower_orange = np.array([10, 100, 100])
#         upper_orange = np.array([25, 255, 255])
#         lower_red1 = np.array([0, 100, 100])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([160, 100, 100])
#         upper_red2 = np.array([180, 255, 255])
#         lower_yellow = np.array([25, 100, 100])
#         upper_yellow = np.array([35, 255, 255])

#         # Create masks for the color ranges
#         orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
#         yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

#         # Combine the masks
#         mask = cv2.bitwise_or(cv2.bitwise_or(orange_mask, red_mask1), cv2.bitwise_or(red_mask2, yellow_mask))

#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         detected = False
#         for i in range(1, num_labels):
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 50000:  # Increase the area threshold for larger objects
#                 x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
#                 cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 self.publish_cone_marker()
#                 self.get_logger().info("ORANGE/RED/YELLOW OBJECT SEEN")
#                 detected = True

#         if detected:
#             self.cone_detected_count += 1

#         if self.cone_detected_count >= self.detection_threshold:
#             # Create a success message on the image
#             cv2.putText(bgr_image, "SUCCESS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         # Add robot location to the image
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
#             location_text = f"Location: ({robot_position[0]:.2f}, {robot_position[1]:.2f}, {robot_position[2]:.2f})"
#             cv2.putText(bgr_image, location_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#         except Exception as e:
#             self.get_logger().error(f'Could not get robot location: {e}')

#         try:
#             processed_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
#             self.image_publisher.publish(processed_image_msg)
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert processed image: {e}")

#     def publish_cone_marker(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = trans.transform.translation.x
#             marker.pose.position.y = trans.transform.translation.y
#             marker.pose.position.z = trans.transform.translation.z
#             marker.pose.orientation = trans.transform.rotation
#             marker.id = self.marker_id
#             self.marker_id += 1
#             self.markers[marker.id] = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
#             self.marker_publisher.publish(marker)
#             self.publish_marker_array()
#             self.log_markers()
#         except Exception as e:
#             self.get_logger().error(f'Could not transform map to base_link: {e}')

#     def publish_marker_array(self):
#         marker_array = MarkerArray()
#         for marker_id, position in self.markers.items():
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = position[0]
#             marker.pose.position.y = position[1]
#             marker.pose.position.z = position[2]
#             marker.id = marker_id
#             marker_array.markers.append(marker)
#         self.marker_array_publisher.publish(marker_array)

#     def log_markers(self):
#         marker_list = "\n".join([f"Marker ID: {marker_id}, Position: {position}" for marker_id, position in self.markers.items()])
#         self.get_logger().info(f"Markers:\n{marker_list}")

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = ConeDetector()
#         rclpy.spin(node)
#     except Exception as e:
#         rclpy.get_logger().error(f"Failed to initialize or spin the node: {e}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker, MarkerArray
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import tf2_ros
# from rclpy.duration import Duration

# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")
#         self.get_logger().info("Initializing ConeDetector node")

#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
#         self.marker_array_publisher = self.create_publisher(MarkerArray, 'cone_marker_array', 10)
#         self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )
#         self.markers = {}
#         self.cone_detected_count = 0
#         self.detection_threshold = 6  # Set your detection threshold here
#         self.marker_id = 0

#     def is_marker_nearby(self):
#         try:
#             if not self.markers:
#                 return False
            
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             robot_position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            
#             for marker_id, marker_position in self.markers.items():
#                 distance = np.sqrt(
#                     (robot_position[0] - marker_position[0])**2 +
#                     (robot_position[1] - marker_position[1])**2 +
#                     (robot_position[2] - marker_position[2])**2
#                 )
#                 if distance < 1.0:  # Define a suitable distance threshold
#                     self.get_logger().info(f"Existing marker {marker_id} detected nearby, skipping cone detection")
#                     return True
#             return False
#         except Exception as e:
#             self.get_logger().error(f'Could not check for nearby marker: {e}')
#             return False

#     def detect_cone(self, msg):
#         if self.is_marker_nearby():
#             return

#         try:
#             bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert image: {e}")
#             return

#         self.get_logger().info(f"Image size: {bgr_image.shape}")

#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#         # Define strict HSV ranges for orange, red, and yellow
#         lower_orange = np.array([10, 100, 100])
#         upper_orange = np.array([25, 255, 255])
#         lower_red1 = np.array([0, 100, 100])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([160, 100, 100])
#         upper_red2 = np.array([180, 255, 255])
#         lower_yellow = np.array([25, 100, 100])
#         upper_yellow = np.array([35, 255, 255])

#         # Create masks for the color ranges
#         orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
#         yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

#         # Combine the masks
#         mask = cv2.bitwise_or(cv2.bitwise_or(orange_mask, red_mask1), cv2.bitwise_or(red_mask2, yellow_mask))

#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         detected = False
#         for i in range(1, num_labels):
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 50000:  # Increase the area threshold for larger objects
#                 x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
#                 cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 self.publish_cone_marker()
#                 self.get_logger().info("ORANGE/RED/YELLOW OBJECT SEEN")
#                 detected = True

#         if detected:
#             self.cone_detected_count += 1

#         if self.cone_detected_count >= self.detection_threshold:
#             # Create a success message on the image
#             cv2.putText(bgr_image, "SUCCESS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         try:
#             processed_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
#             self.image_publisher.publish(processed_image_msg)
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert processed image: {e}")

#     def publish_cone_marker(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = trans.transform.translation.x
#             marker.pose.position.y = trans.transform.translation.y
#             marker.pose.position.z = trans.transform.translation.z
#             marker.pose.orientation = trans.transform.rotation
#             marker.id = self.marker_id
#             self.marker_id += 1
#             self.markers[marker.id] = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
#             self.marker_publisher.publish(marker)
#             self.publish_marker_array()
#             self.log_markers()
#         except Exception as e:
#             self.get_logger().error(f'Could not transform map to base_link: {e}')

#     def publish_marker_array(self):
#         marker_array = MarkerArray()
#         for marker_id, position in self.markers.items():
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = position[0]
#             marker.pose.position.y = position[1]
#             marker.pose.position.z = position[2]
#             marker.id = marker_id
#             marker_array.markers.append(marker)
#         self.marker_array_publisher.publish(marker_array)

#     def log_markers(self):
#         marker_list = "\n".join([f"Marker ID: {marker_id}, Position: {position}" for marker_id, position in self.markers.items()])
#         self.get_logger().info(f"Markers:\n{marker_list}")

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = ConeDetector()
#         rclpy.spin(node)
#     except Exception as e:
#         rclpy.get_logger().error(f"Failed to initialize or spin the node: {e}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()





# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import tf2_ros
# from rclpy.duration import Duration

# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")

#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
#         self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )
#         self.cone_detected_count = 0
#         self.detection_threshold = 3  # Set your detection threshold here

#     def detect_cone(self, msg):
#         try:
#             bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert image: {e}")
#             return

#         self.get_logger().info(f"Image size: {bgr_image.shape}")

#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#         # Define strict HSV ranges for orange, red, and yellow
#         lower_orange = np.array([10, 100, 100])
#         upper_orange = np.array([25, 255, 255])
#         lower_red1 = np.array([0, 100, 100])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([160, 100, 100])
#         upper_red2 = np.array([180, 255, 255])
#         lower_yellow = np.array([25, 100, 100])
#         upper_yellow = np.array([35, 255, 255])

#         # Create masks for the color ranges
#         orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
#         yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

#         # Combine the masks
#         mask = cv2.bitwise_or(cv2.bitwise_or(orange_mask, red_mask1), cv2.bitwise_or(red_mask2, yellow_mask))

#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         detected = False
#         for i in range(1, num_labels):
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 50000:  # Increase the area threshold for larger objects
#                 x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
#                 cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 self.publish_cone_marker()
#                 self.get_logger().info("ORANGE/RED/YELLOW OBJECT SEEN")
#                 detected = True

#         if detected:
#             self.cone_detected_count += 1

#         if self.cone_detected_count >= self.detection_threshold:
#             # Create a success message on the image
#             cv2.putText(bgr_image, "SUCCESS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         try:
#             processed_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
#             self.image_publisher.publish(processed_image_msg)
#         except CvBridgeError as e:
#             self.get_logger().error(f"Could not convert processed image: {e}")

#     def publish_cone_marker(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = trans.transform.translation.x
#             marker.pose.position.y = trans.transform.translation.y
#             marker.pose.position.z = trans.transform.translation.z
#             marker.pose.orientation = trans.transform.rotation
#             self.marker_publisher.publish(marker)
#         except Exception as e:
#             self.get_logger().error(f'Could not transform map to base_link: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import tf2_ros
# from rclpy.duration import Duration

# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")

#         self.bridge = CvBridge()
#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )

#     def detect_cone(self, msg):
#         bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         self.get_logger().info(f"Image size: {bgr_image.shape}")

#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#         lower_red1 = np.array([0, 120, 70])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([170, 120, 70])
#         upper_red2 = np.array([180, 255, 255])

#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
#         mask = cv2.bitwise_or(red_mask1, red_mask2)

#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         for i in range(1, num_labels):
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 8000:
#                 self.publish_cone_marker()
#                 self.get_logger().info("RED OBJECT SEEN")

#     def publish_cone_marker(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=1.0))
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.a = 1.0
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.pose.position.x = trans.transform.translation.x
#             marker.pose.position.y = trans.transform.translation.y
#             marker.pose.position.z = trans.transform.translation.z
#             marker.pose.orientation = trans.transform.rotation
#             self.marker_publisher.publish(marker)
#         except Exception as e:
#             self.get_logger().error(f'Could not transform map to base_link: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# from visualization_msgs.msg import Marker
# import numpy as np

# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import OccupancyGrid

# class ConeDetector(Node):
#     def __init__(self):
#         super().__init__("cone_detector")

#         # Creates subscription to RGB camera topic
#         self.camera_subscription = self.create_subscription(
#             Image,
#             "oak/rgb/image_raw",
#             self.detect_cone,
#             10
#         )

#         self.robot_position_subscriber = self.create_subscription(
#             PoseStamped,
#             "robot_position",
#             self.robot_position_update, 
#             10
#         )

#         self.cones_detected = 0

#         self.bridge = CvBridge()

#         self.marker_publisher = self.create_publisher(Marker, 'cone_marker', 10)

#     def robot_position_update(self, msg):
#         self.current_position = msg

#     def detect_cone(self, msg):
#         bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         self.get_logger().info(str(len(bgr_image)))
#         hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        
#         # Defining lower and upper bound HSV values 
#         lower_red1 = np.array([0, 120, 70])
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([170, 120, 70])
#         upper_red2 = np.array([180, 255, 255])

#         # Create masks for red color ranges
#         red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#         red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

#         # Combine masks
#         mask = cv2.bitwise_or(red_mask1, red_mask2)

#          # Find connected components in the mask
#         num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         for i in range(1, num_labels):  
#             area = stats[i, cv2.CC_STAT_AREA]
#             if area > 8000:
#                 self.publish_cone_marker("RED")
                
                
#     def publish_cone_marker(self, colour):
#         marker = Marker()
#         marker.header.frame_id = 'map'
#         marker.header.stamp = self.get_clock().now().to_msg()

#         marker.type = Marker.SPHERE
#         marker.id = self.cones_detected
#         self.cones_detected += 1
        
#         # Set the scale of the marker
#         marker.scale.x = 1.0
#         marker.scale.y = 1.0
#         marker.scale.z = 1.0

#         if colour == "YELLOW":
#             # Set the color
#             marker.color.r = 1.0
#             marker.color.g = 1.0
#             marker.color.b = 0.0
#             marker.color.a = 1.0
#         else:
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.color.a = 1.0

#         # Set the pose of the marker
#         marker.pose.position.x = self.current_robot_position.pose.position.x + 0.5
#         marker.pose.position.y = self.current_robot_position.pose.position.y
#         marker.pose.position.z = 0.0

#         marker.pose.orientation.x = self.current_robot_position.pose.orientation.x
#         marker.pose.orientation.y = self.current_robot_position.pose.orientation.y
#         marker.pose.orientation.z = self.current_robot_position.pose.orientation.z
#         marker.pose.orientation.w = self.current_robot_position.pose.orientation.w

#         self.marker_publisher.publish(marker)
     
# def main(args = None):
#     rclpy.init(args=args)
#     cone_detector = ConeDetector()
#     rclpy.spin(cone_detector)

#     cone_detector.destroy_node()
#     rclpy.shutdown()








# # globals
# field_of_view = 80
# resolution_width = 1280
# # resolution_height = 0
# pixels_per_degree = resolution_width / field_of_view


# # Find clusters of dark pixels with white pixels around them
# def find_clusters(image):
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     # _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)  # Adjust threshold
    
#     ################
#     # Convert Image to Image HSV
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    
#     # Defining lower and upper bound HSV values 
#     lower1=np.array([0, 110, 55])
#     upper1=np.array([50, 255, 220])

#     lower2=np.array([170, 110, 55])
#     upper2=np.array([359, 255, 220])

    
#     # Defining mask for detecting color 
#     binary1 = cv2.inRange(hsv, lower1, upper1) 
#     binary2 = cv2.inRange(hsv, lower2, upper2)
#     binary = binary1 + binary2
#     cv2.imshow("Maskkkk", binary) 
#     ################

#     contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     clusters = []

#     for contour in contours:
#         area = cv2.contourArea(contour)
#         if 400 < area < 200000:  # Adjusted area range for better sensitivity
#             x, y, w, h = cv2.boundingRect(contour)
#             x = max(x - 20, 0)
#             y = max(y - 20, 0)
#             w = min(w + 40, binary.shape[1] - x)
#             h = min(h + 40, binary.shape[0] - y)
#             roi = gray[y:y+h, x:x+w]
#             mean_val = cv2.mean(roi)[0]

#             # Ensure there is a significant white region around the dark cluster
#             # if mean_val > 150 # Lowered threshold for surrounding white area
#             clusters.append(contour)
#     return clusters

# # Crop the bounding box of the detected cluster and center around the number
# def crop_bounding_box(image, contour):
#     x, y, w, h = cv2.boundingRect(contour)
#     cropped = image[y:y+h, x:x+w]

#     # Further crop around the largest dark region (number)
#     gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
#     _, binary_cropped = cv2.threshold(gray_cropped, 128, 255, cv2.THRESH_BINARY_INV)
#     contours, _ = cv2.findContours(binary_cropped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if contours:
#         largest_contour = max(contours, key=cv2.contourArea)
#         x_num, y_num, w_num, h_num = cv2.boundingRect(largest_contour)
#         cx = x_num + w_num // 2
#         cy = y_num + h_num // 2
#         half_size = max(w_num, h_num) // 2
#         x_start = max(cx - half_size, 0)
#         y_start = max(cy - half_size, 0)
#         x_end = min(cx + half_size, cropped.shape[1])
#         y_end = min(cy + half_size, cropped.shape[0])
#         number_cropped = cropped[y_start:y_end, x_start:x_end]

#         # Center the number in a fixed-size frame for display
#         display_size = 1000
#         centered_cropped = np.zeros((display_size, display_size, 3), dtype=np.uint8)
#         start_x = (display_size - (x_end - x_start)) // 2
#         start_y = (display_size - (y_end - y_start)) // 2
#         centered_cropped[start_y:start_y + (y_end - y_start), start_x:start_x + (x_end - x_start)] = number_cropped

#         return centered_cropped, (x_start + x, y_start + y, x_end - x_start, y_end - y_start)
#     return cropped, (x, y, w, h)

# # OpenCV display loop
# def start_cv():

#     global pixels_per_degree

#     cap = cv2.VideoCapture(0)
#     if not cap.isOpened():
#         print("Error: Could not open video device.")
#         return

#     print("Starting video capture...")

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Failed to grab frame")
#             break

#         gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         display_frame = frame.copy()
#         clusters = find_clusters(display_frame)

#         if clusters:
#             contour = clusters[0]
#             cv2.drawContours(display_frame, [contour], -1, (0, 255, 0), 2)
#             cropped, bbox = crop_bounding_box(frame, contour)
#             gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
#             _, thr = cv2.threshold(gray_cropped, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
#             resizedFrame = cv2.resize(thr, (28, 28))
#             # predicted_digit, confidence = predict(model, resizedFrame)
#             # print(f"Prediction: {predicted_digit} with confidence: {confidence}")



#             # if confidence > 0.9:  # Confidence threshold
#             x, y, w, h = bbox
#             cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Add square around the number
#             ############# LIDAR ##########
#             angle_from_left = (x+w/2) / pixels_per_degree
#             lidar_value = 140 + angle_from_left
#             ############# LIDAR ##########
#             cv2.putText(display_frame, f"{x + (w/2)} ({y + h/2})", (10, 60),
#                         cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
#             cv2.putText(display_frame, f"({"LIDAR: ", lidar_value})", (10, 120),
#                         cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
#             cv2.imshow("Cropped", cropped)
#             cv2.imshow("Thresholded", thr)
#         else:
#             # Show empty images if no clusters are detected
#             empty_display = np.zeros((200, 200, 3), dtype=np.uint8)
#             cv2.imshow("Cropped", empty_display)
#             cv2.imshow("Thresholded", empty_display)

#         cv2.imshow("Result", display_frame)

#         key = cv2.waitKey(1) & 0xff
#         if key == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()
#     print("Video capture stopped.")

# # Main function
# def main():
#     start_cv()

# if __name__ == '__main__':
#     main()
