import cv2
import numpy as np
import tensorflow as tf

# Load pre-trained model
model = tf.keras.models.load_model('model.h5')
print("Loaded pre-trained model.")

# Predict digit using image passed in
def predict(model, img):
    img = np.expand_dims(img, axis=0)  # Add batch dimension
    img = np.expand_dims(img, axis=-1)  # Add channel dimension
    img = img.astype('float32') / 255.0  # Normalize pixel values to [0, 1]
    res = model.predict(img)  # Get prediction from the model
    confidence = np.max(res)  # Get the highest confidence score
    predicted_digit = np.argmax(res)  # Get the predicted digit
    return predicted_digit, confidence  # Return the predicted digit and its confidence

# Find clusters of dark pixels with white pixels around them
def find_clusters(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)  # Threshold to get binary image with dark regions as white

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the binary image
    clusters = []

    for contour in contours:  # Iterate through each contour
        area = cv2.contourArea(contour)  # Calculate the area of the contour
        if 50 < area < 5000:  # Check if the contour area is within the specified range
            x, y, w, h = cv2.boundingRect(contour)  # Get bounding box of the contour
            x = max(x - 20, 0)  # Adjust x to expand the bounding box
            y = max(y - 20, 0)  # Adjust y to expand the bounding box
            w = min(w + 40, binary.shape[1] - x)  # Adjust width to expand the bounding box
            h = min(h + 40, binary.shape[0] - y)  # Adjust height to expand the bounding box
            roi = gray[y:y+h, x:x+w]  # Get region of interest (ROI)
            mean_val = cv2.mean(roi)[0]  # Calculate the mean pixel value of the ROI

            # Ensure there is a significant white region around the dark cluster
            if mean_val > 150:  # Check if the mean value is above the threshold
                clusters.append(contour)  # Append the contour to clusters list
    return clusters  # Return the list of clusters

# Crop the bounding box of the detected cluster and center around the number
def crop_bounding_box(image, contour):
    x, y, w, h = cv2.boundingRect(contour)  # Get bounding box of the contour
    cropped = image[y:y+h, x:x+w]  # Crop the image to the bounding box

    # Further crop around the largest dark region (number)
    gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)  # Convert the cropped image to grayscale
    _, binary_cropped = cv2.threshold(gray_cropped, 128, 255, cv2.THRESH_BINARY_INV)  # Threshold the grayscale image
    contours, _ = cv2.findContours(binary_cropped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the thresholded image
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)  # Find the largest contour
        x_num, y_num, w_num, h_num = cv2.boundingRect(largest_contour)  # Get bounding box of the largest contour
        cx = x_num + w_num // 2  # Calculate the center x-coordinate of the number
        cy = y_num + h_num // 2  # Calculate the center y-coordinate of the number
        half_size = max(w_num, h_num) // 2  # Calculate half size for centering the number
        x_start = max(cx - half_size, 0)  # Calculate the start x-coordinate
        y_start = max(cy - half_size, 0)  # Calculate the start y-coordinate
        x_end = min(cx + half_size, cropped.shape[1])  # Calculate the end x-coordinate
        y_end = min(cy + half_size, cropped.shape[0])  # Calculate the end y-coordinate
        number_cropped = cropped[y_start:y_end, x_start:x_end]  # Crop the image to center the number
        return number_cropped, (x_start + x, y_start + y, x_end - x_start, y_end - y_start)  # Return the cropped number and bounding box
    return cropped, (x, y, w, h)  # Return the cropped image and bounding box if no contours are found

# OpenCV display loop
def start_cv(model):
    cap = cv2.VideoCapture(0)  # Open the default camera
    if not cap.isOpened():
        print("Error: Could not open video device.")
        return

    print("Starting video capture...")

    while True:
        ret, frame = cap.read()  # Capture frame-by-frame
        if not ret:
            print("Failed to grab frame")
            break

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale
        display_frame = frame.copy()  # Make a copy of the frame for display purposes
        clusters = find_clusters(display_frame)  # Find clusters in the frame

        if clusters:
            contour = clusters[0]  # Take the first cluster
            cv2.drawContours(display_frame, [contour], -1, (0, 255, 0), 2)  # Draw the contour on the display frame
            cropped, bbox = crop_bounding_box(frame, contour)  # Crop the bounding box around the number
            gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)  # Convert the cropped image to grayscale
            _, thr = cv2.threshold(gray_cropped, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)  # Threshold the grayscale image
            resizedFrame = cv2.resize(thr, (28, 28))  # Resize the thresholded image to 28x28 pixels
            predicted_digit, confidence = predict(model, resizedFrame)  # Predict the digit using the model
            print(f"Prediction: {predicted_digit} with confidence: {confidence}")

            if confidence > 0.9:  # Confidence threshold
                x, y, w, h = bbox
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Add square around the number
                cv2.putText(display_frame, f"{predicted_digit} ({confidence:.2f})", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)  # Put the predicted digit on the display frame
            cv2.imshow("Cropped", cropped)  # Show the cropped image
            cv2.imshow("Thresholded", thr)  # Show the thresholded image
        else:
            # Show empty images if no clusters are detected
            cv2.imshow("Cropped", np.zeros((200, 200), dtype=np.uint8))  # Show an empty image
            cv2.imshow("Thresholded", np.zeros((200, 200), dtype=np.uint8))  # Show an empty image

        cv2.imshow("Result", display_frame)  # Show the result frame

        key = cv2.waitKey(1) & 0xff  # Wait for a key press
        if key == ord('q'):  # Exit if 'q' is pressed
            break

    cap.release()  # Release the capture
    cv2.destroyAllWindows()  # Close all OpenCV windows
    print("Video capture stopped.")

# Main function
def main():
    start_cv(model)  # Start the OpenCV display loop

if __name__ == '__main__':
    main()  # Call the main function
