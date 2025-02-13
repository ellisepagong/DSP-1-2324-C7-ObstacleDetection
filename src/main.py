# documentation: https://www.ultralytics.com/blog/object-detection-with-a-pre-trained-ultralytics-yolov8-model

# Sends largest object to Arduino
# This is assuming the largest object is the closest as indicated by TOF sensor
# Additional logic will be used once TOF output is confirmed

import threading
import time
from ultralytics import YOLO
import cv2, torch
import serial

# Establish USB connection with Arduino (OUTPUT module)
arduino = serial.Serial(port='COM5', baudrate=9600, timeout=1)  # Starts connection with Arduino

classes_dict = {
    0: 'animal',
    1: 'barrier',
    2: 'bike',
    3: 'crosswalk',
    4: 'hazard-sign',
    5: 'person',
    6: 'pole',
    7: 'stairs',
    8: 'stall',
    9: 'vehicle'
}

max_width = 720  # video pixel width
seg_size = max_width / 5

def handshake_with_output():
    # With continuous stream mode enabled on the OUTPUT module,
    # no handshake is needed. Simply return success.
    print("[CV][HANDSHAKE] Skipping handshake, continuous stream mode enabled.")
    return True

def read_from_output():
    # Continuously read any log messages from OUTPUT module and print them
    while True:
        if arduino.in_waiting:
            line = arduino.readline().decode().strip()
            if line:
                print("[OUTPUT LOG]", line)
        time.sleep(0.1)

def send_to_arduino(largest_boxes):
    # Build a list of class IDs (or -1 for missing detections)
    classes_message = [
        int(data[2].item()) if data is not None else -1
        for data in largest_boxes.values()
    ]
    # Use a space as delimiter (matches OUTPUT module's expected format)
    message = " ".join(map(str, classes_message)) + "\n"
    arduino.write(message.encode())
    print("[CV] Sent to OUTPUT:", message.strip())

def display_pred(img, largest_box):
    # Draw bounding boxes and labels on the image
    for data in largest_box.values():
        if data is not None:
            box, conf, cls = data
            x1, y1, x2, y2 = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)
            text = f'Class: {classes_dict[int(cls)]}, Conf: {conf:.2f}'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            rect_start = (int(x1), int(y1) - text_height - 10)
            rect_end = (int(x1) + text_width, int(y1))
            cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)
            cv2.putText(img, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def assign_segment(x1, x2):
    # Determine image segment based on object's horizontal center
    x = ((x2 - x1) / 2) + x1
    segment_index = int(x // seg_size)
    if segment_index < 0:
        segment_index = 0
    elif segment_index >= 5:
        segment_index = 4
    return segment_index

def main():
    torch.cuda.set_device(0)
    print("[CV] Starting video stream...")
    # Use your video source (camera feed or file)
    video_stream = cv2.VideoCapture("testvid.mp4")
    model = YOLO('model.pt')
    
    # Start a thread to continuously print OUTPUT module logs
    threading.Thread(target=read_from_output, daemon=True).start()
    
    # Skip handshake since the OUTPUT module now works in continuous mode.
    if not handshake_with_output():
        print("[CV] Handshake failed, exiting.")
        return

    print("[CV] Handshake complete, starting main loop.")
    
    while True:
        ret, img = video_stream.read()
        if not ret:
            print("[CV] Failed to grab frame.")
            break
        img = cv2.resize(img, (720, 480))
        results = model(source=img, show=False, conf=0.5, save=True)
        
        for result in results:
            boxes = result.boxes.xyxy
            confs = result.boxes.conf
            classes = result.boxes.cls
            largest_boxes = {i: None for i in range(5)}
            largest_areas = {i: 0 for i in range(5)}
            
            for box, conf, cls in zip(boxes, confs, classes):
                x1, y1, x2, y2 = box
                area = (x2 - x1) * (y2 - y1)
                if classes_dict[int(cls)] == "pole":  # Adjust pole size
                    area *= 0.3
                seg = assign_segment(x1, x2)
                if area > largest_areas[seg]:
                    largest_areas[seg] = area
                    largest_boxes[seg] = (box, conf, cls)
                    
            display_pred(img, largest_boxes)
            send_to_arduino(largest_boxes)
        
        cv2.imshow("Video", img)
        if cv2.waitKey(10) == 27:
            print("[CV] Exiting main loop.")
            break

if __name__ == '__main__':
    main()
