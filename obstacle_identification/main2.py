import ultralytics
from ultralytics import YOLO
import cv2
import requests

# Load a model
model = YOLO('yolov8s.pt')  # Load an official model

url = "http://192.168.238.106"

# Optional: Set video stream resolution and quality (requires source to support control commands)
try:
    requests.get(url + "/control?var=framesize&val={}".format(8))
    requests.get(url + "/control?var=quality&val={}".format(3))
except Exception as e:
    print("SET_RESOLUTION: Something went wrong:", e)

# Open the video stream
cap = cv2.VideoCapture(url + ":81/stream")

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was successfully read
    if not ret:
        print("Failed to read frame from the camera")
        break

    # Predict with the model on the current frame
    results = model(frame)

    # Process results and visualize (optional)
    for box in results.xyxy:  # Assuming detections are in results.xyxy
        xmin, ymin, xmax, ymax, conf, cls = box  # Extract data from each bounding box
        cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax),
                      int(ymax)), (0, 50, 205), 2)  # Adjusted color
        # Assuming model.names contains class names
        label = f"{model.names[int(cls)]} ({conf:.2f})"
        cv2.putText(frame, label, (int(xmin), int(ymin) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    cv2.imshow('Original Frame with Colored ROIs', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
