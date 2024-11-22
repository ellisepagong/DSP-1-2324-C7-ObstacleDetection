#documentation: https://www.ultralytics.com/blog/object-detection-with-a-pre-trained-ultralytics-yolov8-model

# Code to run yolo model

from ultralytics import YOLO
import cv2, torch

def main():
    torch.cuda.set_device(0)

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

    # video_stream = cv2.VideoCapture(1) # camera feed
    video_stream = cv2.VideoCapture("testvid.mp4") # video

    max_width = 640  # for testing videos, should be 1920 assuming 1080p camera feed                                                                                                                        # video pixel width
    seg_size = max_width / 5

    model = YOLO('model.pt')

    while True:
        ret, img = video_stream.read()
        if not ret:
            print("Failed to grab frame")
            break

        results = model(source=img, show=False, conf=0.4, save=True)                                                                         # Perform inference on the current frame

        for result in results:
            boxes = result.boxes.xyxy
            confs = result.boxes.conf
            classes = result.boxes.cls

            for box, conf, cls in zip(boxes, confs, classes):
                x1, y1, x2, y2 = box

                # Class label annotation
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)                          # Bounding box
                text = f'Class: {classes_dict[int(cls)]}, Conf: {conf:.2f}'
                (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                rect_start = (int(x1), int(y1) - text_height - 10)
                rect_end = (int(x1) + text_width, int(y1))
                cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)                                                             # Class label box
                cv2.putText(img, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1) # Class text

                # LOGIC FOR RASPBERRY PI
                x = ((x2-x1)/2)+x1
                segment_index = int(x // seg_size)
                if segment_index < 0:
                    segment_index = 0
                elif segment_index >= 5:
                    segment_index = 4
                duty_cycle1 = int((segment_index / 4) * 100)                                                                                  # PWM for x coordinate, send through PWM pin
                print(f"The object in segment: {segment_index} is sent as duty cycle {duty_cycle1}%")

                duty_cycle2 = int((cls / 9) * 100)                                                                                            # PWM for class, send through PWM pin
                print(f"Sending class {classes_dict[int(cls)]} of value {cls} as duty cycle {duty_cycle2}%")

                # END LOGIC FOR RASPBERRY PI

        # Show the frame with predictions
        cv2.imshow("Video", img)

        # Break the loop on 'ESC' key press
        if cv2.waitKey(10) == 27:
            break

    # Release the video stream and close windows
    video_stream.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()