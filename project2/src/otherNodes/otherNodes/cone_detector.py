import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

class ConeDetector(Node):
    def __init__(self):
        super().__init__("cone_detector")

        # Creates subscription to RGB camera topic
        self.camera_subscription = self.create_subscription(
            Image,
            "oak/rgb/image_raw",
            self.detect_cone,
            10
        )

        self.bridge = CvBridge()

    def detect_cone(self, msg):
        bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.get_logger().info(str(len(bgr_image)))
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        
        # Defining lower and upper bound HSV values 
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color ranges
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine masks
        mask = cv2.bitwise_or(red_mask1, red_mask2)

         # Find connected components in the mask
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        for i in range(1, num_labels):  
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 8000:
                print("cone found")
                ###################### DO STUFF IF CONE IS FOUND


        
def main(args = None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)

    cone_detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()







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