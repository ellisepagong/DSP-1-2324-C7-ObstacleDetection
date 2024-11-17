#documentation: https://www.ultralytics.com/blog/object-detection-with-a-pre-trained-ultralytics-yolov8-model

# Code to test trained YOLO models

from ultralytics import YOLO
import cv2, torch

def main():
    torch.cuda.set_device(0)  #use gpu

    model = YOLO('model.pt') #load pretrained dataset
    video_stream = cv2.VideoCapture(1) # camera feed

    if not video_stream.isOpened():
        print('Error in camera')
        exit(0)

    while True:
        ret, img = video_stream.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Perform inference on the current frame
        results = model(source=img, show=True, conf=0.4, save=False)  # Predict using the model

        # classes returns as integers instead of class names; remove save=True from above to use this
        # for result in results:
        #     boxes = result.boxes.xyxy  # Get bounding box coordinates (x1, y1, x2, y2)
        #     confs = result.boxes.conf  # Get confidence scores
        #     classes = result.boxes.cls  # Get class indices
        #
        #     for box, conf, cls in zip(boxes, confs, classes):
        #         x1, y1, x2, y2 = box
        #         cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)  # Draw bounding box
        #         cv2.putText(img, f'Class: {int(cls)}, Conf: {conf:.2f}', (int(x1), int(y1) - 10),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        #
        # # Show the frame with predictions
        # cv2.imshow("Live Video", img)

        # Break the loop on 'ESC' key press
        if cv2.waitKey(10) == 27:
            break

    # Release the video stream and close windows
    video_stream.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()