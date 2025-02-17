import threading
import time
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
import serial

# Establish USB connection with Arduino (OUTPUT module)
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

# Class dictionary (must match your model’s class order)
classes_dict = {
    0: 'animal', 1: 'barrier', 2: 'bike', 3: 'crosswalk', 4: 'hazard-sign',
    5: 'person', 6: 'pole', 7: 'stairs', 8: 'stall', 9: 'vehicle'
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
    classes_message = [int(data[2]) if data else -1 for data in largest_boxes.values()]
    message = " ".join(map(str, classes_message)) + "\n"
    arduino.write(message.encode())
    print("[CV] Sent to OUTPUT:", message.strip())

def display_pred(img, largest_boxes):
    for data in largest_boxes.values():
        if data:
            box, conf, cls = data
            x1, y1, x2, y2 = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)
            text = f'Class: {classes_dict[int(cls)]}, Conf: {conf:.2f}'
            cv2.putText(img, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def assign_segment(x1, x2):
    x = ((x2 - x1) / 2) + x1
    segment_index = int(x // seg_size)
    return max(0, min(segment_index, 4))

def preprocess_input(image, input_size):
    resized_img = cv2.resize(image, (input_size, input_size)) / 255.0
    return np.expand_dims(resized_img, axis=0).astype(np.float32)

def process_detections(output_data, input_shape, conf_threshold=0.5, iou_threshold=0.5):
    output_data = np.squeeze(output_data).T
    detections = []
    img_h, img_w = input_shape[:2]
    for det in output_data:
        x_c, y_c, w, h, *scores = det
        class_id = np.argmax(scores)
        score = scores[class_id]
        if score > conf_threshold:
            x1, y1 = (x_c - w / 2) * img_w, (y_c - h / 2) * img_h
            x2, y2 = (x_c + w / 2) * img_w, (y_c + h / 2) * img_h
            detections.append([class_id, score, x1, y1, x2, y2])
    return detections

def main():
    display_width, display_height = 1280, 720
    print("[CV] Starting video stream from CSI Port 0...")
    video_stream = cv2.VideoCapture(0, cv2.CAP_V4L2)
    video_stream.set(cv2.CAP_PROP_FRAME_WIDTH, display_width)
    video_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, display_height)
    video_stream.set(cv2.CAP_PROP_FPS, 45)
    
    print("[CV] Loading TFLite model...")
    interpreter = Interpreter(model_path="model_float16.tflite")
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    input_size = input_details[0]['shape'][1]
    
    threading.Thread(target=read_from_output, daemon=True).start()
    if not handshake_with_output():
        print("[CV] Handshake failed, exiting.")
        return
    
    print("[CV] Handshake complete, starting main loop.")
    while True:
        ret, frame = video_stream.read()
        if not ret:
            print("[CV] Failed to grab frame.")
            break
        
        frame_disp = cv2.resize(frame, (max_width, int(max_width * 9 / 16)))
        input_tensor = preprocess_input(frame_disp, input_size)
        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        detections = process_detections(output_data, (input_size, input_size, 3))
        
        scale_x, scale_y = max_width / input_size, int(max_width * 9 / 16) / input_size
        largest_boxes = {i: None for i in range(5)}
        largest_areas = {i: 0 for i in range(5)}
        
        for det in detections:
            class_id, score, x1, y1, x2, y2 = det
            x1_disp, y1_disp, x2_disp, y2_disp = x1 * scale_x, y1 * scale_y, x2 * scale_x, y2 * scale_y
            area = (x2_disp - x1_disp) * (y2_disp - y1_disp)
            seg = assign_segment(x1_disp, x2_disp)
            if area > largest_areas[seg]:
                largest_areas[seg] = area
                largest_boxes[seg] = ((x1_disp, y1_disp, x2_disp, y2_disp), score, class_id)
        
        display_pred(frame_disp, largest_boxes)
        send_to_arduino(largest_boxes)
        cv2.imshow("Video", frame_disp)
        if cv2.waitKey(10) == 27:
            print("[CV] Exiting main loop.")
            break
    
    video_stream.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
