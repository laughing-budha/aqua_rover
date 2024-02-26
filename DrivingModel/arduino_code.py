import torch
import time
import serial

model = torch.hub.load('ultralytics/yolov5', './yolov5s.pt')

arduino_port = '/dev/ttyACM0'  # Sample Port
baud_rate = 9600
arduino = serial.Serial(arduino_port, baud_rate)


def process_image(image):
    results = model(image)
    detections = results.pandas().xyxy[0]

    for _, row in detections.iterrows():
        if row['class']:
            if obstacle_blocks_path(row['xmin'], row['xmax'], row['ymin'], row['ymax']):
                return True
    return False


def obstacle_blocks_path(xmin, xmax, ymin, ymax):
    pass


def find_clear_path():
    arduino.write(b'left')
    time.sleep(2)
    arduino.write(b'forward')


def capture_image_from_arduino():
    pass


# Main loop
while True:
    image = capture_image_from_arduino()
    if process_image(image):
        find_clear_path()
    else:
        arduino.write(b'forward')
