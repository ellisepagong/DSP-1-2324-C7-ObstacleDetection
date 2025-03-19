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
DISPLAY_WIDTH = 720   # used for both display and segmentation
DISPLAY_HEIGHT = 720
SEG_SIZE = DISPLAY_WIDTH / 7

# ----------------------------
# Helper Functions for Object Detection
# ----------------------------
def assign_segment(x1, x2):
    x_center = ((x2 - x1) / 2) + x1
    segment_index = int(x_center // SEG_SIZE)
    return max(0, min(segment_index, 4))

def send_to_console(largest_boxes, distances, inference_time):
    # Clear the console for updated output
    os.system('cls' if os.name == 'nt' else 'clear')  # Clears the console (Windows: 'cls', Unix-based: 'clear')

    # Prepare a message with five class IDs (or -1 if no detection)
    # followed by three distance values (left, center, right) in centimeters.
    classes_message = [
        int(data[2]) if data is not None else -1
        for data in largest_boxes.values()
    ]
    distance_message = [distances['left'], distances['center'], distances['right']]
    message = "< " + " ".join(map(str, classes_message + distance_message)) + " >"

    # Display live updating console output
    print("================ LIVE DATA ================")
    print(f"Inference time: {inference_time:.3f} seconds")
    print(f"Output Values (CV): {message}")
    print(f"Specific Distance Values (Left)    : {distances['left']} cm")
    print(f"Specific Distance Values (Center)  : {distances['center']} cm")
    print(f"Specific Distance Values (Right)   : {distances['right']} cm")
    
    # Flush output to ensure real-time updating
    sys.stdout.flush()

def preprocess_input(image, input_size):
    resized_img = cv2.resize(image, (input_size, input_size))
    normalized_img = resized_img / 255.0
    input_tensor = np.expand_dims(normalized_img, axis=0).astype(np.float32)
    return input_tensor

def process_detections(output_data, input_shape, conf_threshold=0.60, iou_threshold=0.5):
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

def display_pred(img, largest_boxes):
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

# ----------------------------
# Updated Depth Perception Functions
# ----------------------------
def compute_region_distances_depth(depth_frame):
    """
    Computes the minimum depth (i.e. closest object) in three regions:
    left (segments 0-1), center (segment 2), and right (segments 3-4).
    The depth_frame is in millimeters.
    Returns distances in centimeters as integers.
    """
    h, w = depth_frame.shape
    col_width = w / 5
    regions = {'left': [], 'center': [], 'right': []}
    for col in range(w):
        seg = int(col // col_width)
        seg = max(0, min(seg, 4))
        if seg in [0, 1]:
            region = 'left'
        elif seg == 2:
            region = 'center'
        else:
            region = 'right'
        valid_vals = depth_frame[:, col][depth_frame[:, col] > 0]
        regions[region].extend(valid_vals.tolist())
    
    distances = {}
    for region, vals in regions.items():
        if vals:
            distances[region] = int(min(vals) / 10.0)  # Convert mm to cm
        else:
            distances[region] = 0
    return distances

# ----------------------------
# Updated Pipeline Setup for Improved Depth Perception
# ----------------------------
def create_pipeline():
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
    stereo.setSubpixelFractionalBits(5)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.initialConfig.setDisparityShift(1)
    stereo.setOutputSize(640, 400)  # Set output size as in your depth perception code
    
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
        
        # For visualization, compute a multiplier from max disparity
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
            # Get color frame
            rgbFrame = queueColor.get().getCvFrame()
            rgbFrame_disp = cv2.resize(rgbFrame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            
            # Get depth frame (in mm)
            inDepth = queueDepth.get()
            depthFrame = inDepth.getFrame()
            depthFrame = np.clip(depthFrame, 0, 5100)  # Clip max depth to 5100 mm
            
            # Compute region distances (in cm, as integers)
            distances = compute_region_distances_depth(depthFrame)
            
            # Preprocess color image for detection
            input_tensor = preprocess_input(rgbFrame_disp, input_size)
            
            current_time = time.time()
            live_fps = 1.0 / (current_time - prev_frame_time) if (current_time - prev_frame_time) > 0 else 0
            prev_frame_time = current_time
            
            start_inference = time.time()
            interpreter.set_tensor(input_details[0]['index'], input_tensor)
            interpreter.invoke()
            inference_time = time.time() - start_inference
            inference_fps = 1.0 / inference_time if inference_time > 0 else 0
            print("[CV] Inference time: {:.3f} seconds".format(inference_time))
            
            output_data = interpreter.get_tensor(output_details[0]['index'])
            detections = process_detections(output_data, (input_size, input_size, 3),
                                            conf_threshold=0.23, iou_threshold=0.5)
            
            scale_x = DISPLAY_WIDTH / input_size
            scale_y = DISPLAY_HEIGHT / input_size
            
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
            
            display_pred(rgbFrame_disp, largest_boxes)
            send_to_console(largest_boxes, distances, inference_time)
            
            fps_text = f"Live FPS: {live_fps:.2f} | Inference FPS: {inference_fps:.2f}"
            cv2.putText(rgbFrame_disp, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            
            # Normalize and color-map depth frame for visualization
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