#documentation: https://www.ultralytics.com/blog/object-detection-with-a-pre-trained-ultralytics-yolov8-model

# Sends largest object to arduino
# This is assuming the largest object is the closest as indicated by TOF sensor
# Additional logic will be used once TOF output is confirmed

from ultralytics import YOLO
import cv2, torch
import serial

arduino = serial.Serial(port='COM5',  baudrate=9600, timeout=1)

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
def send_to_arduino(cls, segment_index):
    message = f"{cls},{segment_index}\n"  # Format the data as "cls,segment_index"
    message = f"{segment_index}\n"  # Format the data as "cls,segment_index"
    arduino.write(message.encode())       # Send as bytes
    print(f"Sent: {message}")     # Log the sent message

def display_pred(img, box, conf, cls):  # displays bounding box
    x1, y1, x2, y2 = box

    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)  # Bounding box
    text = f'Class: {classes_dict[int(cls)]}, Conf: {conf:.2f}'
    (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    rect_start = (int(x1), int(y1) - text_height - 10)
    rect_end = (int(x1) + text_width, int(y1))
    cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)  # Class label box
    cv2.putText(img, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                1)  # Class text

def main():
    torch.cuda.set_device(0) # Use GPU

    # video_stream = cv2.VideoCapture(1) # camera feed
    video_stream = cv2.VideoCapture("indoor.mp4") # video

    max_width = 640                                                                                                                          # video pixel width
    seg_size = max_width / 5

    model = YOLO('model.pt')

    while True:
        ret, img = video_stream.read()
        if not ret:
            print("Failed to grab frame")
            break

        results = model(source=img, show=False, conf=0.3, save=True)                                                                         # Perform inference on the current frame

        for result in results:
            boxes = result.boxes.xyxy
            confs = result.boxes.conf
            classes = result.boxes.cls

            largest_box = None
            largest_area = 0

            for box, conf, cls in zip(boxes, confs, classes):
                x1, y1, x2, y2 = box

                area = (x2 - x1) * (y2 - y1)
                if classes_dict[int(cls)] == "pole": # Scale down poles due to generally large size
                    area = area * 0.3

                if area > largest_area:
                    largest_area = area
                    largest_box = (box, conf, cls)

            if largest_box is not None:
                box, conf, cls = largest_box
                x1, y1, x2, y2 = box

                display_pred(img, box, conf, cls)

                # Raspberry pi output simulation
                # Assign Segment
                x = ((x2-x1)/2)+x1
                segment_index = int(x // seg_size)
                if segment_index < 0:
                    segment_index = 0
                elif segment_index >= 5:
                    segment_index = 4


                # Send to arduino
                send_to_arduino(int(cls+1), segment_index+1)

            else: # No detections: send default values
                send_to_arduino(0, 0)


        # Show the frame with predictions
        cv2.imshow("Video", img)

        # Break the loop on 'ESC' key press
        if cv2.waitKey(10) == 27:
            break

    # Release the video stream and close windows

if __name__ == '__main__':
    main()