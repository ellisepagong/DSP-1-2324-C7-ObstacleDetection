import depthai as dai
import cv2
import numpy as np
import time
import tensorflow as tf
import sys
import os

# ----------------------------
# Object detection parameters and classes dictionary (unchanged)
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

# ----------------------------
# Display configuration and segmentation parameters
# ----------------------------
DISPLAY_WIDTH = 720
DISPLAY_HEIGHT = 720
NUM_SEGMENTS = 7
SEG_SIZE = DISPLAY_WIDTH / NUM_SEGMENTS  # Each segment width

# ----------------------------
# Helper Functions for Object Detection
# ----------------------------

def assign_segment(x1, x2):
    """
    Determine which column (0 to 6) a detection belongs to based on its horizontal center.
    """
    x_center = ((x2 - x1) / 2) + x1
    segment_index = int(x_center // SEG_SIZE)
    return max(0, min(segment_index, NUM_SEGMENTS - 1))

def compute_depth_per_column(depth_frame):
    """
    Splits the depth frame (heat map) into 7 vertical segments and computes a representative
    depth value (here, using the median over each column) for each column.
    """
    h, w = depth_frame.shape
    col_width = w / NUM_SEGMENTS
    depths = []
    for i in range(NUM_SEGMENTS):
        start_col = int(i * col_width)
        end_col = int((i + 1) * col_width)
        segment = depth_frame[:, start_col:end_col]
        valid_vals = segment[segment > 0]
        if valid_vals.size > 0:
            # Using median for stability; convert mm to cm.
            median_depth = int(np.median(valid_vals) / 10.0)
        else:
            median_depth = 0
        depths.append(median_depth)
    return depths

def get_depth_at_point(depth_frame, x, y, window=5):
    """
    Returns the median depth (in cm) in a small window around the point (x, y) from the depth heat map.
    """
    h, w = depth_frame.shape
    x1 = max(0, int(x) - window)
    x2 = min(w, int(x) + window)
    y1 = max(0, int(y) - window)
    y2 = min(h, int(y) + window)
    window_vals = depth_frame[y1:y2, x1:x2]
    valid = window_vals[window_vals > 0]
    if valid.size > 0:
        return int(np.median(valid) / 10.0)
    else:
        return 0

def display_pred(img, largest_boxes):
    """
    Draws bounding boxes and labels for each column detection.
    """
    for data in largest_boxes.values():
        if data is not None:
            box, conf, cls = data
            x1, y1, x2, y2 = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 50, 50), 2)
            text = f'Class: {classes_dict[int(cls)]}, Conf: {conf:.2f}'
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            rect_start = (int(x1), int(y1) - text_height - 10)
            rect_end = (int(x1) + text_width, int(y1))
            cv2.rectangle(img, rect_start, rect_end, (255, 50, 50), -1)
            cv2.putText(img, text, (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def send_to_console(largest_boxes, final_depths, inference_time):
    """
    Prints the output message in the following format:
      < c0 c1 c2 c3 c4 c5 c6 d0 d1 d2 d3 d4 d5 d6 >
    where c0..c6 are class IDs for each column (or -1 if no detection)
    and d0..d6 are the corresponding depth values (in cm).
    Also prints the inference time.
    """
    os.system('cls' if os.name == 'nt' else 'clear')
    classes_message = [int(largest_boxes[i][2]) if largest_boxes[i] is not None else -1 for i in range(NUM_SEGMENTS)]
    message = "< " + " ".join(map(str, classes_message + final_depths)) + " >"
    print("================ LIVE DATA ================")
    print(f"Inference time: {inference_time:.3f} seconds")
    print(f"Output Values (CV): {message}")
    for i in range(NUM_SEGMENTS):
        print(f"Segment {i} Depth: {final_depths[i]} cm")
    sys.stdout.flush()

def preprocess_input(image, input_size):
    """
    Resizes and normalizes the image for the object detection model.
    """
    resized_img = cv2.resize(image, (input_size, input_size))
    normalized_img = resized_img / 255.0
    input_tensor = np.expand_dims(normalized_img, axis=0).astype(np.float32)
    return input_tensor

def process_detections(output_data, input_shape, conf_threshold=0.60, iou_threshold=0.5):
    """
    Processes raw NN output into a list of detections [class_id, score, x1, y1, x2, y2].
    Coordinates are scaled to the input image size.
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

def create_pipeline():
    """
    Sets up a DepthAI pipeline with a color camera and a stereo depth node.
    The stereo depth simulates two depth cameras.
    """
    
    pipeline = dai.Pipeline()
    
    # Color camera (CAM_A)
    colorCam = pipeline.createColorCamera()
    colorCam.setPreviewSize(640, 480)
    colorCam.setInterleaved(False)
    colorCam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
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
    
    # Stereo Depth node with improved configuration
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setSubpixel(True)
    stereo.setLeftRightCheck(True)
    # ----- Added modifications for near-range accuracy -----
    stereo.setExtendedDisparity(True)
    stereo.initialConfig.setDisparityShift(30)
    stereo.initialConfig.setConfidenceThreshold(150)
    # -------------------------------------------------------
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    
    # Depth output stream (named "depth")
    xoutDepth = pipeline.createXLinkOut()
    xoutDepth.setStreamName("depth")
    stereo.depth.link(xoutDepth.input)
    
    return pipeline, stereo

# ----------------------------
# Main Function
# ----------------------------
def main():
    pipeline, stereo = create_pipeline()
    with dai.Device(pipeline) as device:
        queueColor = device.getOutputQueue(name="color", maxSize=1, blocking=True)
        queueDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        
        # For visualization: get multiplier if needed from max disparity.
        max_disp = stereo.initialConfig.getMaxDisparity()
        multiplier = 255.0 / max_disp
        
        # TensorFlow Lite Interpreter for object detection
        interpreter = tf.lite.Interpreter(model_path=r"D:\Career\IDS\DSP-1-2324-C7-ObstacleDetection\src\3D_CMOS\tests\model_float16_480x480.tflite")
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        input_size = input_details[0]['shape'][1]
        print(f"[CV] Model input size: {input_size}x{input_size}")
        
        prev_frame_time = time.time()
        
        while True:
            # Get the color frame and resize for display.
            rgbFrame = queueColor.get().getCvFrame()
            rgbFrame_disp = cv2.resize(rgbFrame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            
            # Get the depth frame, clip its values, and then resize it to match the color frame.
            inDepth = queueDepth.get()
            depthFrame = inDepth.getFrame()
            depthFrame = np.clip(depthFrame, 0, 5100)
            depthFrame = cv2.resize(depthFrame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            
            # Compute per-column depth from the depth heat map.
            column_depths = compute_depth_per_column(depthFrame)
            
            # Preprocess the display frame for the object detection model.
            input_tensor = preprocess_input(rgbFrame_disp, input_size)
            
            current_time = time.time()
            live_fps = 1.0 / (current_time - prev_frame_time) if (current_time - prev_frame_time) > 0 else 0
            prev_frame_time = current_time
            
            # Run inference.
            start_inference = time.time()
            interpreter.set_tensor(input_details[0]['index'], input_tensor)
            interpreter.invoke()
            inference_time = time.time() - start_inference
            
            output_data = interpreter.get_tensor(output_details[0]['index'])
            detections = process_detections(output_data, (input_size, input_size, 3),
                                            conf_threshold=0.23, iou_threshold=0.5)
            
            scale_x = DISPLAY_WIDTH / input_size
            scale_y = DISPLAY_HEIGHT / input_size
            
            # Initialize dictionaries to store the largest detection (by area) per column.
            largest_boxes = {i: None for i in range(NUM_SEGMENTS)}
            largest_areas = {i: 0 for i in range(NUM_SEGMENTS)}
            depth_at_centers = [0] * NUM_SEGMENTS
            
            # Process each detection.
            for detection in detections:
                class_id, score, x1, y1, x2, y2 = detection
                x1_disp = x1 * scale_x
                y1_disp = y1 * scale_y
                x2_disp = x2 * scale_x
                y2_disp = y2 * scale_y
                area = (x2_disp - x1_disp) * (y2_disp - y1_disp)
                seg = assign_segment(x1_disp, x2_disp)
                if area > largest_areas[seg]:
                    largest_areas[seg] = area
                    largest_boxes[seg] = ((x1_disp, y1_disp, x2_disp, y2_disp), score, class_id)
                    # Estimate depth using the heat map by taking the median inside the bounding box.
                    x_center = (x1_disp + x2_disp) / 2
                    y_center = (y1_disp + y2_disp) / 2
                    depth_at_centers[seg] = get_depth_at_point(depthFrame, x_center, y_center)
            
            # For each segment, choose the detection depth if available; otherwise, use the column depth.
            final_depths = []
            for i in range(NUM_SEGMENTS):
                if largest_boxes[i] is not None and depth_at_centers[i] > 0:
                    final_depths.append(depth_at_centers[i])
                else:
                    final_depths.append(column_depths[i])
            
            # Draw detection bounding boxes.
            display_pred(rgbFrame_disp, largest_boxes)
            # Print the output message and additional info to the console.
            send_to_console(largest_boxes, final_depths, inference_time)
            
            fps_text = f"Live FPS: {live_fps:.2f} | Inference FPS: {1.0/inference_time:.2f}"
            cv2.putText(rgbFrame_disp, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            
            # Visualize the depth heat map.
            depth_norm = cv2.normalize(depthFrame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            
            cv2.imshow("RGB Camera Feed", rgbFrame_disp)
            cv2.imshow("Depth Map", depth_colormap)
            
            if cv2.waitKey(1) == ord('q'):
                print("[CV] Exiting main loop.")
                break
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
