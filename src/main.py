#documentation: https://www.ultralytics.com/blog/object-detection-with-a-pre-trained-ultralytics-yolov8-model

# Code to run yolo model

from ultralytics import YOLO
import cv2, torch

def main():
    torch.cuda.set_device(0)  #use gpu

    model = YOLO('model.pt') #load pretrained dataset
    video_stream = cv2.VideoCapture(1) # camera feed
    # video_stream = cv2.VideoCapture("indoor.mp4") # video

    if not video_stream.isOpened():
        print('Error in camera')
        exit(0)

    while True:
        ret, img = video_stream.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Perform inference on the current frame
        results = model(source=img, show=False, conf=0.4, save=True)  # Predict using the model
        for result in results:
            boxes = result.boxes.xyxy  # Get bounding box coordinates (x1, y1, x2, y2)
            confs = result.boxes.conf  # Get confidence scores
            classes = result.boxes.cls  # Get class indices

            for box, conf, cls in zip(boxes, confs, classes):
                x1, y1, x2, y2 = box
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)  # Draw bounding box

                # class labels
                text = f'Class: {int(cls)}, Conf: {conf:.2f}'
                (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                rect_start = (int(x1), int(y1) - text_height - 10)
                rect_end = (int(x1) + text_width, int(y1))
                cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)
                cv2.putText(img, text, (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)  # White text

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