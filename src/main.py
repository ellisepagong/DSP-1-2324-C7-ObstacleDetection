import threading
import time
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
import serial
import csv
import re
from picamera2 import Picamera2

# Establish USB connection with Arduino (OUTPUT module)
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
time.sleep(3)  # Wait 3 seconds for Arduino to reset


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
display_width = 720  
display_height = 720
max_width = 720  
seg_size = max_width / 5

# Global variables for sharing the latest video frame
latest_frame = None
frame_lock = threading.Lock()
running = True  # Used to signal threads to stop
cv_request = False  # New flag to trigger one inference on request

# Open CSV files for logging
performance_log = open("performance_log.csv", "w", newline="")
csv_writer = csv.writer(performance_log)
csv_writer.writerow(["Video", "Inference_Count", "Timestamp", "CV_Inference_Time_ms", "Total_Processing_Time_ms", "Avg_FPS"])

arduino_timing_log = open("arduino_timing.csv", "w", newline="")
arduino_csv_writer = csv.writer(arduino_timing_log)
arduino_csv_writer.writerow(["Timestamp", "Timing_Line"])

def handshake_with_output():

    print("[CV][HANDSHAKE] Skipping handshake, continuous stream mode enabled.")
    return True

def read_from_output():

    with open("arduino_log.txt", "a") as log_file:
        while True:
            if arduino.in_waiting:
                line = arduino.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print("[OUTPUT LOG]", line)
                    log_file.write(line + "\n")
                    log_file.flush()

                    if line == "[OM_CV_REQUEST]":
                        global cv_request
                        cv_request = True
                    if "[TIMING]" in line:
                        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                        arduino_csv_writer.writerow([timestamp, line])
                        arduino_timing_log.flush()

            time.sleep(0.1)

def send_to_arduino(largest_boxes):
    # Build a list of class IDs (or -1 for missing detections)
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
            cv2.putText(img, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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

def process_detections(output_data, input_shape, conf_threshold=0.23, iou_threshold=0.5):
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

# --- Live Camera Capture Thread ---


def live_camera_picamera2():
    global latest_frame, running
    # Initialize Picamera2
    picam2 = Picamera2()
    video_config = picam2.create_preview_configuration(main={"size": (720, 720)})
    picam2.configure(video_config)
    picam2.start()
    
    while running:
        # Capture frame from Picamera2
        frame = picam2.capture_array()
        # Optionally convert from RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        with frame_lock:
            latest_frame = frame_bgr.copy()
        cv2.imshow("Live Camera", frame_bgr)
        if cv2.waitKey(1) & 0xFF == 27:  # Exit on 'Esc' key
            running = False
            break
    picam2.stop()


# --- CV Inference Thread ---
def cv_inference_worker(interpreter, input_details, output_details, input_size, csv_writer, stream_id, sim_start_time):
    global latest_frame, running, cv_request
    inference_count = 0
    while running:
        if not cv_request:
            time.sleep(0.01)
            continue
        
        with frame_lock:
            if latest_frame is None:
                continue
            frame = latest_frame.copy()
        inference_count += 1
        
        frame_disp = cv2.resize(frame, (display_width, display_height))
        input_tensor = preprocess_input(frame_disp, input_size)
        
        inference_start = time.time()
        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()
        inference_end = time.time()
        cv_inference_time = (inference_end - inference_start) * 1000  # ms
        
        output_data = interpreter.get_tensor(output_details[0]['index'])
        detections = process_detections(output_data, (input_size, input_size, 3), conf_threshold=0.5, iou_threshold=0.5)
        
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
        
        send_to_arduino(largest_boxes)
        cv_request = False
        
        current_time = time.time()
        elapsed = current_time - sim_start_time
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(sim_start_time + elapsed))
        total_processing_time = (current_time - inference_start) * 1000  # ms
        avg_fps = inference_count / elapsed if elapsed > 0 else 0
        csv_writer.writerow([stream_id, inference_count, timestamp, f"{cv_inference_time:.2f}", f"{total_processing_time:.2f}", f"{avg_fps:.2f}"])
        performance_log.flush()
        print(f"[CV] Total processing: {total_processing_time:.2f} ms, Avg FPS: {avg_fps:.2f}")
        time.sleep(0.001)

def main():
    global running, latest_frame
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
    
    # Use live camera feed instead of a video file
    running = True
    latest_frame = None
    sim_start_time = time.time()
    
    camera_thread = threading.Thread(target=live_camera_picamera2)
    inference_thread = threading.Thread(target=cv_inference_worker, args=(interpreter, input_details, output_details, input_size, csv_writer, "live_camera", sim_start_time))
    
    camera_thread.start()
    inference_thread.start()
    
    camera_thread.join()
    running = False
    inference_thread.join()
    
    cv2.destroyAllWindows()
    performance_log.close()
    arduino_timing_log.close()
    print("[CV] Live camera processing finished. Performance log saved.")


if __name__ == '__main__':
    main()
