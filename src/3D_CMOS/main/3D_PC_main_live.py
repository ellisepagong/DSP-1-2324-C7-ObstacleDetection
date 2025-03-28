import depthai as dai
import cv2
import numpy as np
import time
import tensorflow as tf
import sys
import os
import serial
from pathlib import Path
import csv
import re

# Precompile a regex that looks for the metric name and its value in ms.
# This will match any of MOTOR_ACTIVATION, CV_TO_BT, or OVERALL.
timing_regex = re.compile(r'\[(MOTOR_ACTIVATION|CV_TO_BT|OVERALL)\]\s+(\d+)\s*ms')

# ----------------------------
# Object detection parameters and classes dictionary
# ----------------------------
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

# Define which classes are considered "special"
SPECIAL_CLASSES = {2, 3, 7, 9}

# ----------------------------
# Display configuration and segmentation parameters
# ----------------------------
DISPLAY_WIDTH = 720
DISPLAY_HEIGHT = 720
NUM_SEGMENTS = 7
SEG_SIZE = DISPLAY_WIDTH / NUM_SEGMENTS

# ----------------------------
# Helper Functions for Object Detection
# ----------------------------
def assign_segment(x1, x2):
    x_center = ((x2 - x1) / 2) + x1
    segment_index = int(x_center // SEG_SIZE)
    return max(0, min(segment_index, NUM_SEGMENTS - 1))

def get_depth_at_point(depth_frame, x, y, window=5):
    h, w = depth_frame.shape
    x1 = max(0, int(x) - window)
    x2 = min(w, int(x) + window)
    y1 = max(0, int(y) - window)
    y2 = min(h, int(y) + window)
    window_vals = depth_frame[y1:y2, x1:x2]
    valid = window_vals[window_vals > 0]
    if valid.size > 0:
        return int(np.median(valid) / 10.0)  # convert mm to cm
    else:
        return 0

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

def display_candidates(img, candidate_groupB, candidate_groupA):
    for candidate in [candidate_groupB, candidate_groupA]:
        if candidate is not None:
            box, score, cls, depth_val, seg = candidate['box'], candidate['score'], candidate['class'], candidate['depth'], candidate['segment']
            cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 50, 50), 2)
            text = f"Class: {classes_dict[int(cls)]}, Conf: {score:.2f}, Dist: {depth_val}cm, Seg: {seg}"
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            rect_start = (int(box[0]), int(box[1]) - text_height - 10)
            rect_end = (int(box[0]) + text_width, int(box[1]))
            cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)
            cv2.putText(img, text, (int(box[0]), int(box[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def send_to_console(candidate_groupB, candidate_groupA, inference_time, total_duration):
    if candidate_groupB is not None:
        out_groupB = f"{candidate_groupB['class']}, {candidate_groupB['depth']}, {candidate_groupB['segment']}"
    else:
        out_groupB = "-1, -1, -1"
        
    if candidate_groupA is not None:
        out_groupA = f"{candidate_groupA['class']}, {candidate_groupA['depth']}, {candidate_groupA['segment']}"
    else:
        out_groupA = "-1, -1, -1"
    
    final_output = out_groupB + ", " + out_groupA
    print(f"[CV] Inference time: {inference_time * 1000:.2f} ms")
    print(f"[CV] Total process duration: {total_duration * 1000:.2f} ms")
    print(f"[CV] Output Values: {final_output}")
    sys.stdout.flush()
    return final_output

def create_pipeline():
    pipeline = dai.Pipeline()
    
    # Color camera (CAM_A)
    colorCam = pipeline.createColorCamera()
    colorCam.setPreviewSize(640, 480)
    colorCam.setInterleaved(False)
    colorCam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    #colorCam.setIspScale(2, 3)

    xoutColor = pipeline.createXLinkOut()
    xoutColor.setStreamName("color")
    colorCam.preview.link(xoutColor.input)
    
    # Mono cameras (CAM_B and CAM_C)
    monoLeft = pipeline.createMonoCamera()
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    
    monoRight = pipeline.createMonoCamera()
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    
    # Stereo Depth node
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setSubpixel(True)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)
    stereo.initialConfig.setConfidenceThreshold(150)
    
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    
    xoutDepth = pipeline.createXLinkOut()
    xoutDepth.setStreamName("depth")
    stereo.depth.link(xoutDepth.input)
    
    return pipeline, stereo

# ----------------------------
# Main Function
# ----------------------------
def main():
    pipeline, stereo = create_pipeline()
    
    # Initialize serial connection to Arduino.
    try:
        COM_PORT = 'COM7'  # Adjust as necessary.
        BAUD_RATE = 9600
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        time.sleep(5)  # Wait for Arduino reset.
        print(f"[CV] Connected to Arduino on {COM_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"[CV] Failed to connect to Arduino: {e}")
        ser = None

    # Create a directory for performance metrics if it doesn't exist
    script_dir = Path(__file__).parent
    metrics_dir = script_dir / "performance_metrics"
    metrics_dir.mkdir(parents=True, exist_ok=True)

    # Create and open the CV performance metrics CSV file for writing
    csv_filename = metrics_dir / "CV_performance metrics.csv"
    csv_file = open(csv_filename, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Inference_Time", "Total_Duration"])

    # Create and open the MEGA performance metrics CSV file for writing
    mega_csv_filename = metrics_dir / "MEGA_performance metrics.csv"
    mega_csv_file = open(mega_csv_filename, mode='w', newline='')
    mega_csv_writer = csv.writer(mega_csv_file)
    # Update header to your desired column names:
    mega_csv_writer.writerow(["Motor_Activation", "CV_To_BT", "Overall"])

    pending_metrics = {}
    
    with dai.Device(pipeline) as device:
        queueColor = device.getOutputQueue(name="color", maxSize=1, blocking=True)
        queueDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        
        interpreter = tf.lite.Interpreter(model_path=r"D:\Career\IDS\DSP-1-2324-C7-ObstacleDetection\src\3D_CMOS\tests\model_float16_480x480.tflite")
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        input_size = input_details[0]['shape'][1]
        print(f"[CV] Model input size: {input_size}x{input_size}")
        
        prev_frame_time = time.time()
        
        while True:
            # --- Flush stale frames from the queues ---
            # Flush the color queue
            rgbFrame = None
            while True:
                packet = queueColor.tryGet()
                if packet is None:
                    break
                rgbFrame = packet.getCvFrame()
            if rgbFrame is None:
                rgbFrame = queueColor.get().getCvFrame()
            
            # Flush the depth queue
            depthFrame = None
            while True:
                packet = queueDepth.tryGet()
                if packet is None:
                    break
                depthFrame = packet.getFrame()
            if depthFrame is None:
                depthFrame = queueDepth.get().getFrame()

            # --- Wait for handshake ---
            if ser is not None:
                handshake_received = False
                while ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    # Look for a timing line using regex
                    match = timing_regex.search(line)
                    if match:
                        metric_tag, metric_value = match.groups()
                        pending_metrics[metric_tag] = metric_value
                    elif "[OM_CV_REQUEST]" in line:
                        print("[CV] Handshake received. Running inference.")
                        handshake_received = True
                    else:
                        print(f"[MEGA2560] {line}")
                # Once handshake is received, if we have all metrics, write them out.
                if handshake_received:
                    if all(key in pending_metrics for key in ("MOTOR_ACTIVATION", "CV_TO_BT", "OVERALL")):
                        motor = pending_metrics["MOTOR_ACTIVATION"]
                        cv_bt = pending_metrics["CV_TO_BT"]
                        overall = pending_metrics["OVERALL"]
                        mega_csv_writer.writerow([motor, cv_bt, overall])
                        mega_csv_file.flush()
                        print("Wrote CSV row:", motor, cv_bt, overall)
                        # Clear for the next set of timings
                        pending_metrics.clear()
                    else:
                        print("Incomplete metrics, not writing CSV row yet.")
                else:
                    time.sleep(0.01)
                    continue




            rgbFrame_disp = cv2.resize(rgbFrame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            depthFrame = np.clip(depthFrame, 0, 5100)
            depthFrame = cv2.resize(depthFrame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            
            input_tensor = preprocess_input(rgbFrame_disp, input_size)
            
            current_time = time.time()
            live_fps = 1.0 / (current_time - prev_frame_time) if (current_time - prev_frame_time) > 0 else 0
            prev_frame_time = current_time
            
            start_inference = time.time()
            interpreter.set_tensor(input_details[0]['index'], input_tensor)
            interpreter.invoke()
            inference_time = time.time() - start_inference
            
            output_data = interpreter.get_tensor(output_details[0]['index'])
            detections = process_detections(output_data, (input_size, input_size, 3),
                                            conf_threshold=0.23, iou_threshold=0.5)
            
            scale_x = DISPLAY_WIDTH / input_size
            scale_y = DISPLAY_HEIGHT / input_size
            
            # First pass: get the overall closest detection (candidate_groupB)
            candidate_groupB = None
            for detection in detections:
                class_id, score, x1, y1, x2, y2 = detection
                x1_disp = x1 * scale_x
                y1_disp = y1 * scale_y
                x2_disp = x2 * scale_x
                y2_disp = y2 * scale_y
                x_center = (x1_disp + x2_disp) / 2
                y_center = (y1_disp + y2_disp) / 2
                depth_val = get_depth_at_point(depthFrame, x_center, y_center)
                if depth_val == 0:
                    continue
                candidate = {
                    'class': class_id,
                    'score': score,
                    'depth': depth_val,
                    'segment': int(x_center // SEG_SIZE),
                    'box': (x1_disp, y1_disp, x2_disp, y2_disp)
                }
                if candidate_groupB is None or depth_val < candidate_groupB['depth']:
                    candidate_groupB = candidate

            # Second pass: get the closest detection among SPECIAL_CLASSES (candidate_groupA)
            candidate_groupA = None
            for detection in detections:
                class_id, score, x1, y1, x2, y2 = detection
                x1_disp = x1 * scale_x
                y1_disp = y1 * scale_y
                x2_disp = x2 * scale_x
                y2_disp = y2 * scale_y
                x_center = (x1_disp + x2_disp) / 2
                y_center = (y1_disp + y2_disp) / 2
                depth_val = get_depth_at_point(depthFrame, x_center, y_center)
                if depth_val == 0:
                    continue
                if class_id not in SPECIAL_CLASSES:
                    continue
                candidate = {
                    'class': class_id,
                    'score': score,
                    'depth': depth_val,
                    'segment': int(x_center // SEG_SIZE),
                    'box': (x1_disp, y1_disp, x2_disp, y2_disp)
                }
                if candidate_groupB is not None and candidate_groupB['class'] == candidate['class']:
                    continue
                if candidate_groupA is None or depth_val < candidate_groupA['depth']:
                    candidate_groupA = candidate
            
            if candidate_groupB is not None:
                out_groupB = f"{candidate_groupB['class']}, {candidate_groupB['depth']}, {candidate_groupB['segment']}"
            else:
                out_groupB = "-1, -1, -1"
            if candidate_groupA is not None:
                out_groupA = f"{candidate_groupA['class']}, {candidate_groupA['depth']}, {candidate_groupA['segment']}"
            else:
                out_groupA = "-1, -1, -1"
            final_output = out_groupB + ", " + out_groupA + "\n"
            
            display_candidates(rgbFrame_disp, candidate_groupB, candidate_groupA)
            
            total_duration = time.time() - start_inference
            
            output_str = send_to_console(candidate_groupB, candidate_groupA, inference_time, total_duration)
            
            if ser is not None:
                try:
                    ser.write((output_str + "\n").encode())
                except Exception as e:
                    print(f"[CV] Error sending data to Arduino: {e}")
            
            csv_writer.writerow([f"{inference_time:.3f}", f"{total_duration:.3f}"])
            csv_file.flush()
            
            depth_norm = cv2.normalize(depthFrame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            combined_frame = np.hstack((rgbFrame_disp, depth_colormap))
            # cv2.imshow("RGB Camera Feed", rgbFrame_disp)
            # cv2.imshow("Depth Map", depth_colormap)
            cv2.imshow("Combined Inference and Heatmap", combined_frame)
            
            if cv2.waitKey(1) == ord('q'):
                print("[CV] Exiting main loop.")
                break
        
        csv_file.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()