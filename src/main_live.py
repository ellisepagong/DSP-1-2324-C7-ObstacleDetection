import threading
import time
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
import serial
from picamera2 import Picamera2

# Establish USB connection with Arduino (OUTPUT module)
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

# Class dictionary (must match your model’s class order)
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

# Display configuration
max_width = 720  
seg_size = max_width / 5

def handshake_with_output():
    print("[CV][HANDSHAKE] Skipping handshake, continuous stream mode enabled.")
    return True

def read_from_output():
    while True:
        if arduino.in_waiting:
            line = arduino.readline().decode('utf-8', errors='replace').strip()
            if line:
                print("[OUTPUT LOG]", line)
        time.sleep(0.1)

def send_to_arduino(largest_boxes):
    classes_message = [
        int(data[2]) if data is not None else -1
        for data in largest_boxes.values()
    ]
    message = " ".join(map(str, classes_message)) + "\n"
    arduino.write(message.encode())
    print("[CV] Sent to OUTPUT:", message.strip())

def display_pred(img, largest_boxes):
    for data in largest_boxes.values():
        if data is not None:
            box, conf, cls = data
            x1, y1, x2, y2 = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)
            text = f'Class: {classes_dict[int(cls)]}, Conf: {conf:.2f}'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            rect_start = (int(x1), int(y1) - text_height - 10)
            rect_end = (int(x1) + text_width, int(y1))
            cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)
            cv2.putText(img, text, (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def assign_segment(x1, x2):
    x = ((x2 - x1) / 2) + x1
    segment_index = int(x // seg_size)
    if segment_index < 0:
        segment_index = 0
    elif segment_index >= 5:
        segment_index = 4
    return segment_index

def preprocess_input(image, input_size):
    resized_img = cv2.resize(image, (input_size, input_size))
    normalized_img = resized_img / 255.0
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
    
    boxes = []
    for i in range(len(detections)):
        boxes.append([int(x1[i]), int(y1[i]), int(x2[i] - x1[i]), int(y2[i] - y1[i])])
    
    indices = cv2.dnn.NMSBoxes(boxes, scores.tolist(), score_threshold=0.23, nms_threshold=iou_threshold)
    indices = indices.flatten() if len(indices) > 0 else []
    return [detections[i] for i in indices]

def process_detections(output_data, input_shape, conf_threshold=0.23, iou_threshold=0.5):
    """
    Assumes TFLite output_data has shape [1, 14, 8400]:
      - First 4 numbers: x_center, y_center, width, height (normalized).
      - Next 10 numbers: class scores.
    """
    output_data = np.squeeze(output_data)
    output_data = np.transpose(output_data)
    
    detections = []
    img_height, img_width = input_shape[:2]
    
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
    
    return detections

def main():
    display_width = 360
    display_height = 360  # 720p by 720p

    print("[CV] Starting camera stream using Picamera2...")
    picam2 = Picamera2()
    # Set preview configuration to a 720x720 resolution
    config = picam2.create_preview_configuration(main={"size": (display_width, display_height)})
    picam2.configure(config)
    picam2.start()

    print("[CV] Loading TFLite model...")
    interpreter = Interpreter(model_path="model_float16_480x480.tflite")
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    input_size = input_details[0]['shape'][1]
    print(f"[CV] Model input size: {input_size}x{input_size}")
    
    threading.Thread(target=read_from_output, daemon=True).start()
    
    if not handshake_with_output():
        print("[CV] Handshake failed, exiting.")
        return

    print("[CV] Handshake complete, starting main loop.")
    prev_frame_time = time.time()
    
    while True:
        frame = picam2.capture_array()
        if frame is None:
            print("[CV] Failed to capture frame from camera.")
            break

        # Force the captured frame into BGR format.
        # Here we assume the incoming image is in RGB.
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # For inference, use the same BGR image.
        input_tensor = preprocess_input(frame, input_size)
        
        current_time = time.time()
        live_fps = 1.0 / (current_time - prev_frame_time) if (current_time - prev_frame_time) > 0 else 0
        prev_frame_time = current_time
        
        start_inference = time.time()
        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()
        inference_time = time.time() - start_inference
        inference_fps = 1.0 / inference_time if inference_time > 0 else 0

        # Output inference time to the console
        print("[CV] Inference time: {:.3f} seconds".format(inference_time))
        
        output_data = interpreter.get_tensor(output_details[0]['index'])
        detections = process_detections(output_data, (input_size, input_size, 3),
                                        conf_threshold=0.23, iou_threshold=0.5)
        
        scale_x = display_width / input_size
        scale_y = display_height / input_size
        
        largest_boxes = {i: None for i in range(5)}
        largest_areas = {i: 0 for i in range(5)}
        
        for detection in detections:
            class_id, score, x1, y1, x2, y2 = detection
            x1_disp = x1 * scale_x
            y1_disp = y1 * scale_y
            x2_disp = x2 * scale_x
            y2_disp = y2 * scale_y
            area = (x2_disp - x1_disp) * (y2_disp - y1_disp)
            if classes_dict[int(class_id)] == "pole":
                area *= 0.3
            seg = assign_segment(x1_disp, x2_disp)
            if area > largest_areas[seg]:
                largest_areas[seg] = area
                largest_boxes[seg] = ((x1_disp, y1_disp, x2_disp, y2_disp), score, class_id)
        
        display_pred(frame, largest_boxes)
        send_to_arduino(largest_boxes)
        
        fps_text = f"Live FPS: {live_fps:.2f} | Inference FPS: {inference_fps:.2f}"
        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.imshow("Video", frame)
        if cv2.waitKey(10) == 27:
            print("[CV] Exiting main loop.")
            break
    
    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
