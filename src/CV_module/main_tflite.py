import numpy as np
from tflite_runtime.interpreter import Interpreter
import cv2
from picamera2 import Picamera2
import time

def preprocess_input(image, model_width, model_height):
    """
    Preprocess the input image to match the model's required input size.
    """
    resized_img = cv2.resize(image, (model_width, model_height))
    normalized_img = resized_img / 255.0  # Normalize if required by your model
    input_tensor = np.expand_dims(normalized_img, axis=0).astype(np.float32)
    return input_tensor

def non_max_suppression(detections, iou_threshold):
    if len(detections) == 0:
        return []
    detections = np.array(detections)
    x1 = detections[:, 2]
    y1 = detections[:, 3]
    x2 = detections[:, 4]
    y2 = detections[:, 5]
    scores = detections[:, 1]
    indices = cv2.dnn.NMSBoxes(
        bboxes=list(zip(x1, y1, x2 - x1, y2 - y1)),
        scores=scores.tolist(),
        score_threshold=0.5,
        nms_threshold=iou_threshold
    )
    indices = indices.flatten() if len(indices) > 0 else []
    return [detections[i] for i in indices]

def process_detections(output_data, img_shape, conf_threshold=0.5, iou_threshold=0.5):
    output_data = np.squeeze(output_data)
    output_data = np.transpose(output_data)
    detections = []
    img_height, img_width = img_shape[:2]
    for detection in output_data:
        x_center, y_center, width, height = detection[0:4]
        class_scores = detection[4:]
        class_id = np.argmax(class_scores)
        score = class_scores[class_id]
        if score > conf_threshold:
            x_center *= img_width
            y_center *= img_height
            width *= img_width
            height *= img_height
            x1 = x_center - width / 2
            y1 = y_center - height / 2
            x2 = x_center + width / 2
            y2 = y_center + height / 2
            detections.append([class_id, score, x1, y1, x2, y2])
    return non_max_suppression(detections, iou_threshold)

def main():
    print("Starting the object detection script...")
    classes_dict = {
        0: 'animal', 1: 'barrier', 2: 'bike', 3: 'crosswalk', 4: 'hazard-sign',
        5: 'person', 6: 'pole', 7: 'stairs', 8: 'stall', 9: 'vehicle'
    }

    # Camera and model configurations
    camera_resolution = (480, 480)
    model_width, model_height = 640, 640

    # Initialize TFLite model
    interpreter = Interpreter(model_path="model_float16.tflite")
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Initialize Picamera2
    picam2 = Picamera2()
    video_config = picam2.create_preview_configuration(main={"size": camera_resolution})
    picam2.configure(video_config)
    picam2.start()

    try:
        while True:
            # Capture a frame after previous processing is done
            frame = picam2.capture_array()

            # Preprocess the frame
            img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            input_tensor = preprocess_input(img, model_width, model_height)

            # Run inference
            start_time = time.time()
            interpreter.set_tensor(input_details[0]['index'], input_tensor)
            interpreter.invoke()
            inference_time = time.time() - start_time
            print(f"Inference Time: {inference_time:.2f} seconds")

            # Process detections
            output_data = interpreter.get_tensor(output_details[0]['index'])
            detections = process_detections(output_data, img.shape)

            # Draw detections
            for detection in detections:
                class_id, score, x1, y1, x2, y2 = detection
                x1, y1 = int(max(0, x1)), int(max(0, y1))
                x2, y2 = int(min(img.shape[1], x2)), int(min(img.shape[0], y2))
                class_name = classes_dict.get(int(class_id), 'Unknown')
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{class_name}: {score:.2f}'
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display the frame
            cv2.imshow("Live Video", img)

            # Exit condition
            if cv2.waitKey(1) == 27:  # Press 'Esc' to exit
                break

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print("Resources released. Program terminated.")

if __name__ == '__main__':
    main()
