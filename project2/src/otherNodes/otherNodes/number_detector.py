import cv2
import numpy as np
import tensorflow as tf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
from rclpy.duration import Duration
import os
import time

class NumberDetector(Node):
    def __init__(self):
        super().__init__('number_detector')

        # Load the pre-trained TensorFlow model with error handling
        try:
            self.model = tf.keras.models.load_model('model.h5')
            self.get_logger().info("Loaded pre-trained model.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            rclpy.shutdown()

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(Marker, 'number_marker', 10)
        self.marker_array_publisher = self.create_publisher(MarkerArray, 'number_marker_array', 10)
        self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            'oak/rgb/image_raw',  # Adjust the topic name if necessary
            self.detect_number,
            10
        )

        # Create publishers for the processed images
        self.result_pub = self.create_publisher(Image, 'number_detection/result', 10)
        self.cropped_pub = self.create_publisher(Image, 'number_detection/cropped', 10)
        self.thresholded_pub = self.create_publisher(Image, 'number_detection/thresholded', 10)
        self.finally_found_pub = self.create_publisher(Image, 'finally_found', 10)

        self.markers = {}
        self.marker_id = 0
        self.save_directory = "/photos"

    def is_marker_nearby(self):
        try:
            if not self.markers:
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
        except Exception as e:
            self.get_logger().error(f'Could not check for nearby marker: {e}')
            return False

    def detect_number(self, msg):
        if self.is_marker_nearby():
            return

        try:
            # Convert ROS image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            display_frame = frame.copy()

            # Find clusters of interest
            clusters = self.find_clusters(display_frame)

            if clusters:
                contour = clusters[0]
                cv2.drawContours(display_frame, [contour], -1, (255, 0, 0), 1)
                cropped, bbox = self.crop_bounding_box(frame, contour)
                gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
                _, thr = cv2.threshold(gray_cropped, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                resized_frame = cv2.resize(thr, (28, 28))

                # Predict the digit
                predicted_digit, confidence = self.predict(self.model, resized_frame)
                self.get_logger().info(f"Prediction: {predicted_digit} with confidence: {confidence}")

                # Display results if confidence is high
                if confidence > 0.96:
                    x, y, w, h = bbox
                    cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.putText(display_frame, f"{predicted_digit} ({confidence:.2f})", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

                    # Publish the final found image
                    final_msg = self.bridge.cv2_to_imgmsg(display_frame, 'bgr8')
                    final_msg.header.frame_id = msg.header.frame_id
                    self.finally_found_pub.publish(final_msg)
                    self.get_logger().info("Published to 'finally_found' topic.")
                    
                    # Publish marker
                    self.publish_number_marker()

                # Publish the cropped and thresholded images
                cropped_msg = self.bridge.cv2_to_imgmsg(cropped, 'bgr8')
                thresholded_msg = self.bridge.cv2_to_imgmsg(thr, 'mono8')
                cropped_msg.header.frame_id = msg.header.frame_id
                thresholded_msg.header.frame_id = msg.header.frame_id
                self.cropped_pub.publish(cropped_msg)
                self.thresholded_pub.publish(thresholded_msg)
            else:
                self.get_logger().info("No clusters found.")
                empty_image = np.zeros((200, 200), dtype=np.uint8)
                empty_msg = self.bridge.cv2_to_imgmsg(empty_image, 'mono8')
                empty_msg.header.frame_id = msg.header.frame_id
                self.cropped_pub.publish(empty_msg)
                self.thresholded_pub.publish(empty_msg)

            # Publish the result image
            result_msg = self.bridge.cv2_to_imgmsg(display_frame, 'bgr8')
            result_msg.header.frame_id = msg.header.frame_id
            self.result_pub.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f"Error in detect_number: {e}")

        # Wait 3 seconds before looking again
        time.sleep(3)

    def find_clusters(self, image):
        try:
            # Convert image to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Apply binary threshold
            _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)

            # Find contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            clusters = []

            self.get_logger().info(f"Found {len(contours)} contours")  # Debugging

            # Filter contours based on area and shape
            for contour in contours:
                area = cv2.contourArea(contour)
                if 40000 < area < 50000:  # Adjusted area range
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / float(h)
                    if 0.2 < aspect_ratio < 1.0:  # Aspect ratio range for digits
                        clusters.append(contour)

            self.get_logger().info(f"Filtered to {len(clusters)} clusters")  # Debugging

            return clusters
        except Exception as e:
            self.get_logger().error(f"Error in find_clusters: {e}")
            return []

    def crop_bounding_box(self, image, contour):
        try:
            # Crop the bounding box around the largest contour
            x, y, w, h = cv2.boundingRect(contour)
            cropped = image[y:y+h, x:x+w]

            # Convert cropped image to grayscale and find contours
            gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
            _, binary_cropped = cv2.threshold(gray_cropped, 128, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary_cropped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x_num, y_num, w_num, h_num = cv2.boundingRect(largest_contour)
                cx = x_num + w_num // 2
                cy = y_num + h_num // 2
                half_size = max(w_num, h_num) // 2
                x_start = max(cx - half_size, 0)
                y_start = max(cy - half_size, 0)
                x_end = min(cx + half_size, cropped.shape[1])
                y_end = min(cy + half_size, cropped.shape[0])
                number_cropped = cropped[y_start:y_end, x_start:x_end]
                return number_cropped, (x_start + x, y_start + y, x_end - x_start, y_end - y_start)
            return cropped, (x, y, w, h)
        except Exception as e:
            self.get_logger().error(f"Error in crop_bounding_box: {e}")
            return image, (0, 0, image.shape[1], image.shape[0])

    def predict(self, model, img):
        try:
            # Prepare image for prediction
            img = np.expand_dims(img, axis=0)
            img = np.expand_dims(img, axis=-1)
            img = img.astype('float32') / 255.0
            res = model.predict(img)
            confidence = np.max(res)
            predicted_digit = np.argmax(res)
            return predicted_digit, confidence
        except Exception as e:
            self.get_logger().error(f"Error in predict: {e}")
            return -1, 0.0

    def publish_number_marker(self):
        try:
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
            marker.color.r = 0.0
            marker.color.g = 1.0
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
            self.log_markers()
        except Exception as e:
            self.get_logger().error(f'Could not transform map to base_link: {e}')

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
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.id = marker_id
            marker_array.markers.append(marker)
        self.marker_array_publisher.publish(marker_array)

    def log_markers(self):
        marker_list = "\n".join([f"Marker ID: {marker_id}, Position: {position}" for marker_id, position in self.markers.items()])
        self.get_logger().info(f"Markers:\n{marker_list}")

def main(args=None):
    # Initialize the ROS client library and create the node
    rclpy.init(args=args)
    node = NumberDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import cv2
# import numpy as np
# import tensorflow as tf
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import time

# class NumberDetector(Node):
#     def __init__(self):
#         super().__init__('number_detector')
        
#         # Load the pre-trained TensorFlow model with error handling
#         try:
#             self.model = tf.keras.models.load_model('model.h5')
#             print("Loaded pre-trained model.")
#         except Exception as e:
#             print(f"Failed to load model: {e}")
#             rclpy.shutdown()
        
#         # Initialize CvBridge
#         self.bridge = CvBridge()
        
#         # Subscribe to the image topic
#         self.subscription = self.create_subscription(
#             Image,
#             'oak/rgb/image_raw',  # Adjust the topic name if necessary
#             self.detect_number,
#             10
#         )
        
#         # Create publishers for the processed images
#         self.result_pub = self.create_publisher(Image, 'number_detection/result', 10)
#         self.cropped_pub = self.create_publisher(Image, 'number_detection/cropped', 10)
#         self.thresholded_pub = self.create_publisher(Image, 'number_detection/thresholded', 10)
#         self.finally_found_pub = self.create_publisher(Image, 'finally_found', 10)

#     def detect_number(self, msg):
#         try:
#             # Convert ROS image message to OpenCV format
#             frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#             display_frame = frame.copy()
            
#             # Find clusters of interest
#             clusters = self.find_clusters(display_frame)

#             if clusters:
#                 contour = clusters[0]
#                 cv2.drawContours(display_frame, [contour], -1, (255, 0, 0), 1)
#                 cropped, bbox = self.crop_bounding_box(frame, contour)
#                 gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
#                 _, thr = cv2.threshold(gray_cropped, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
#                 resized_frame = cv2.resize(thr, (28, 28))
                
#                 # Predict the digit
#                 predicted_digit, confidence = self.predict(self.model, resized_frame)
#                 print(f"Prediction: {predicted_digit} with confidence: {confidence}")

#                 # Display results if confidence is high
#                 if confidence > 0.9:
#                     x, y, w, h = bbox
#                     cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
#                     cv2.putText(display_frame, f"{predicted_digit} ({confidence:.2f})", (10, 60),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
                    
#                     # Publish the final found image
#                     final_msg = self.bridge.cv2_to_imgmsg(display_frame, 'bgr8')
#                     final_msg.header.frame_id = msg.header.frame_id
#                     self.finally_found_pub.publish(final_msg)
#                     print("Published to 'finally_found' topic.")
                
#                 # Publish the cropped and thresholded images
#                 cropped_msg = self.bridge.cv2_to_imgmsg(cropped, 'bgr8')
#                 thresholded_msg = self.bridge.cv2_to_imgmsg(thr, 'mono8')
#                 cropped_msg.header.frame_id = msg.header.frame_id
#                 thresholded_msg.header.frame_id = msg.header.frame_id
#                 self.cropped_pub.publish(cropped_msg)
#                 self.thresholded_pub.publish(thresholded_msg)
#             else:
#                 print("No clusters found.")
#                 empty_image = np.zeros((200, 200), dtype=np.uint8)
#                 empty_msg = self.bridge.cv2_to_imgmsg(empty_image, 'mono8')
#                 empty_msg.header.frame_id = msg.header.frame_id
#                 self.cropped_pub.publish(empty_msg)
#                 self.thresholded_pub.publish(empty_msg)

#             # Publish the result image
#             result_msg = self.bridge.cv2_to_imgmsg(display_frame, 'bgr8')
#             result_msg.header.frame_id = msg.header.frame_id
#             self.result_pub.publish(result_msg)
#         except Exception as e:
#             print(f"Error in detect_number: {e}")
        
#         # Wait 3 seconds before looking again
#         time.sleep(3)

#     def find_clusters(self, image):
#         try:
#             # Convert image to grayscale
#             gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
#             # Apply binary threshold
#             _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
            
#             # Find contours
#             contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             clusters = []

#             print(f"Found {len(contours)} contours")  # Debugging

#             # Filter contours based on area and shape
#             for contour in contours:
#                 area = cv2.contourArea(contour)
#                 if 1000 < area < 50000:  # Adjusted area range
#                     x, y, w, h = cv2.boundingRect(contour)
#                     aspect_ratio = w / float(h)
#                     if 0.2 < aspect_ratio < 1.0:  # Aspect ratio range for digits
#                         clusters.append(contour)
            
#             print(f"Filtered to {len(clusters)} clusters")  # Debugging
            
#             return clusters
#         except Exception as e:
#             print(f"Error in find_clusters: {e}")
#             return []

#     def crop_bounding_box(self, image, contour):
#         try:
#             # Crop the bounding box around the largest contour
#             x, y, w, h = cv2.boundingRect(contour)
#             cropped = image[y:y+h, x:x+w]

#             # Convert cropped image to grayscale and find contours
#             gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
#             _, binary_cropped = cv2.threshold(gray_cropped, 128, 255, cv2.THRESH_BINARY_INV)
#             contours, _ = cv2.findContours(binary_cropped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             if contours:
#                 largest_contour = max(contours, key=cv2.contourArea)
#                 x_num, y_num, w_num, h_num = cv2.boundingRect(largest_contour)
#                 cx = x_num + w_num // 2
#                 cy = y_num + h_num // 2
#                 half_size = max(w_num, h_num) // 2
#                 x_start = max(cx - half_size, 0)
#                 y_start = max(cy - half_size, 0)
#                 x_end = min(cx + half_size, cropped.shape[1])
#                 y_end = min(cy + half_size, cropped.shape[0])
#                 number_cropped = cropped[y_start:y_end, x_start:x_end]
#                 return number_cropped, (x_start + x, y_start + y, x_end - x_start, y_end - y_start)
#             return cropped, (x, y, w, h)
#         except Exception as e:
#             print(f"Error in crop_bounding_box: {e}")
#             return image, (0, 0, image.shape[1], image.shape[0])

#     def predict(self, model, img):
#         try:
#             # Prepare image for prediction
#             img = np.expand_dims(img, axis=0)
#             img = np.expand_dims(img, axis=-1)
#             img = img.astype('float32') / 255.0
#             res = model.predict(img)
#             confidence = np.max(res)
#             predicted_digit = np.argmax(res)
#             return predicted_digit, confidence
#         except Exception as e:
#             print(f"Error in predict: {e}")
#             return -1, 0.0

# def main(args=None):
#     # Initialize the ROS client library and create the node
#     rclpy.init(args=args)
#     node = NumberDetector()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





# import cv2
# import numpy as np
# import tensorflow as tf
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# class NumberDetector(Node):
#     def __init__(self):
#         super().__init__('number_detector')
        
#         # Load the pre-trained TensorFlow model with error handling
#         try:
#             self.model = tf.keras.models.load_model('model.h5')
#             self.get_logger().info("Loaded pre-trained model.")
#         except Exception as e:
#             self.get_logger().error(f"Failed to load model: {e}")
#             rclpy.shutdown()
        
#         # Initialize CvBridge
#         self.bridge = CvBridge()
        
#         # Subscribe to the image topic
#         self.subscription = self.create_subscription(
#             Image,
#             'oak/rgb/image_raw',  # Adjust the topic name if necessary
#             self.detect_number,
#             10
#         )

#     def detect_number(self, msg):
#         try:
#             # Convert ROS image message to OpenCV format
#             frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#             display_frame = frame.copy()
            
#             # Find clusters of interest
#             clusters = self.find_clusters(display_frame)

#             if clusters:
#                 contour = clusters[0]
#                 cv2.drawContours(display_frame, [contour], -1, (255, 0, 0), 1)
#                 cropped, bbox = self.crop_bounding_box(frame, contour)
#                 gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
#                 _, thr = cv2.threshold(gray_cropped, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
#                 resized_frame = cv2.resize(thr, (28, 28))
                
#                 # Predict the digit
#                 predicted_digit, confidence = self.predict(self.model, resized_frame)
#                 self.get_logger().info(f"Prediction: {predicted_digit} with confidence: {confidence}")

#                 # Display results if confidence is high
#                 if confidence > 0.9:
#                     x, y, w, h = bbox
#                     cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
#                     cv2.putText(display_frame, f"{predicted_digit} ({confidence:.2f})", (10, 60),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
#                 cv2.imshow("Cropped", cropped)
#                 cv2.imshow("Thresholded", thr)
#             else:
#                 cv2.imshow("Cropped", np.zeros((200, 200), dtype=np.uint8))
#                 cv2.imshow("Thresholded", np.zeros((200, 200), dtype=np.uint8))

#             cv2.imshow("Result", display_frame)
#             key = cv2.waitKey(1) & 0xff
#             if key == ord('q'):
#                 cv2.destroyAllWindows()
#                 rclpy.shutdown()
#         except Exception as e:
#             self.get_logger().error(f"Error in detect_number: {e}")

#     def find_clusters(self, image):
#         try:
#             # Convert image to grayscale
#             gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
#             # Apply binary threshold
#             _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
#             cv2.imshow("Binary", binary)  # Debugging
            
#             # Find contours
#             contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             clusters = []

#             self.get_logger().info(f"Found {len(contours)} contours")  # Debugging

#             # Filter contours based on area and shape
#             for contour in contours:
#                 area = cv2.contourArea(contour)
#                 if 1000 < area < 50000:  # Adjusted area range
#                     x, y, w, h = cv2.boundingRect(contour)
#                     aspect_ratio = w / float(h)
#                     if 0.2 < aspect_ratio < 1.0:  # Aspect ratio range for digits
#                         clusters.append(contour)
            
#             self.get_logger().info(f"Filtered to {len(clusters)} clusters")  # Debugging
            
#             return clusters
#         except Exception as e:
#             self.get_logger().error(f"Error in find_clusters: {e}")
#             return []

#     def crop_bounding_box(self, image, contour):
#         try:
#             # Crop the bounding box around the largest contour
#             x, y, w, h = cv2.boundingRect(contour)
#             cropped = image[y:y+h, x:x+w]

#             # Convert cropped image to grayscale and find contours
#             gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
#             _, binary_cropped = cv2.threshold(gray_cropped, 128, 255, cv2.THRESH_BINARY_INV)
#             contours, _ = cv2.findContours(binary_cropped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             if contours:
#                 largest_contour = max(contours, key=cv2.contourArea)
#                 x_num, y_num, w_num, h_num = cv2.boundingRect(largest_contour)
#                 cx = x_num + w_num // 2
#                 cy = y_num + h_num // 2
#                 half_size = max(w_num, h_num) // 2
#                 x_start = max(cx - half_size, 0)
#                 y_start = max(cy - half_size, 0)
#                 x_end = min(cx + half_size, cropped.shape[1])
#                 y_end = min(cy + half_size, cropped.shape[0])
#                 number_cropped = cropped[y_start:y_end, x_start:x_end]
#                 return number_cropped, (x_start + x, y_start + y, x_end - x_start, y_end - y_start)
#             return cropped, (x, y, w, h)
#         except Exception as e:
#             self.get_logger().error(f"Error in crop_bounding_box: {e}")
#             return image, (0, 0, image.shape[1], image.shape[0])

#     def predict(self, model, img):
#         try:
#             # Prepare image for prediction
#             img = np.expand_dims(img, axis=0)
#             img = np.expand_dims(img, axis=-1)
#             img = img.astype('float32') / 255.0
#             res = model.predict(img)
#             confidence = np.max(res)
#             predicted_digit = np.argmax(res)
#             return predicted_digit, confidence
#         except Exception as e:
#             self.get_logger().error(f"Error in predict: {e}")
#             return -1, 0.0

# def main(args=None):
#     # Initialize the ROS client library and create the node
#     rclpy.init(args=args)
#     node = NumberDetector()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
