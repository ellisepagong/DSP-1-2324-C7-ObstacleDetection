#documentation: https://www.ultralytics.com/blog/object-detection-with-a-pre-trained-ultralytics-yolov8-model

# Sends largest object to arduino
# This is assuming the largest object is the closest as indicated by TOF sensor
# Additional logic will be used once TOF output is confirmed

from ultralytics import YOLO
import cv2, torch
import serial

arduino = serial.Serial(port='COM5',  baudrate=9600, timeout=1)                                                         # Starts connection with Arduino

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

max_width = 720                                                                                                         # video pixel width
seg_size = max_width / 5

def send_to_arduino(largest_boxes):
    classes_message = [
        int(data[2].item()) if data is not None else "-1"                                                               # Use -1 for segments with no valid box
        for data in largest_boxes.values()
    ]
    message = ",".join(map(str, classes_message)) + "\n"                                                                # Join the list into a single string separated by commas
    arduino.write(message.encode())                                                                                     # Send the complete message to Arduino
    # print(f"Sent: {message.strip()}")                                                                                   # Log the sent message

def display_pred(img, largest_box):                                                                                     # displays bounding box
    for data in largest_box.values():
        if data is not None:
            box, conf, cls = data
            x1, y1, x2, y2 = box

            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)  # Bounding box
            text = f'Class: {classes_dict[int(cls)]}, Conf: {conf:.2f}'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            rect_start = (int(x1), int(y1) - text_height - 10)
            rect_end = (int(x1) + text_width, int(y1))
            cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)  # Class label box
            cv2.putText(img, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                        1)  # Class text

def assign_segment(x1, x2):                                                                                             #Assigns object to image segment
    x = ((x2 - x1) / 2) + x1
    segment_index = int(x // seg_size)
    if segment_index < 0:
        segment_index = 0
    elif segment_index >= 5:
        segment_index = 4
    return segment_index

def main():
    torch.cuda.set_device(0)

    # video_stream = cv2.VideoCapture(1)                                                                                # camera feed
    video_stream = cv2.VideoCapture("testvid.mp4")                                                                      # video source
    model = YOLO('model.pt')

    while True:
        ret, img = video_stream.read()
        img = cv2.resize(img, (720, 480))
        if not ret:
            print("Failed to grab frame")
            break

        results = model(source=img, show=False, conf=0.5, save=True)                                                    # Perform inference on the current frame

        for result in results:
            boxes = result.boxes.xyxy
            confs = result.boxes.conf
            classes = result.boxes.cls

            largest_boxes = {i: None for i in range(5)}
            largest_areas = {i: 0 for i in range(5)}

            for box, conf, cls in zip(boxes, confs, classes):
                x1, y1, x2, y2 = box
                area = (x2 - x1) * (y2 - y1)
                if classes_dict[int(cls)] == "pole":                                                                    # Scale down poles due to generally large size
                    area *= 0.3

                seg = assign_segment(x1, x2)
                if area > largest_areas[seg]:                                                                           # Finds largest object in each segment
                    largest_areas[seg] = area
                    largest_boxes[seg] = (box, conf, cls)

            display_pred(img, largest_boxes)                                                                            # Display largest object in each segment
            send_to_arduino(largest_boxes)                                                                              # Send largest objects to Arduino

        cv2.imshow("Video", img)

        if cv2.waitKey(10) == 27:
                break

if __name__ == '__main__':
    main()