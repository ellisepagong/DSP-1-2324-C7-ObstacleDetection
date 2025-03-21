import depthai as dai
import cv2
import numpy as np
import time
import tensorflow as tf
import sys
import os

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
SPECIAL_CLASSES = {2, 3, 7, 9}  # bike, crosswalk, stairs, vehicle

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
    Determine which column (0 to NUM_SEGMENTS-1) a detection belongs to based on its horizontal center.
    """
    x_center = ((x2 - x1) / 2) + x1
    segment_index = int(x_center // SEG_SIZE)
    return max(0, min(segment_index, NUM_SEGMENTS - 1))

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
        return int(np.median(valid) / 10.0)  # convert mm to cm
    else:
        return 0

def preprocess_input(image, input_size):
    """
    Resizes and normalizes the image for the object detection model.
    """
    resized_img = cv2.resize(image, (input_size, input_size))
    normalized_img = resized_img / 255.0
    input_tensor = np.expand_dims(normalized_img, axis=0).astype(np.float32)
    return input_tensor

def process_detections(output_data, input_shape, conf_threshold=0.23, iou_threshold=0.5):
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

def display_candidates(img, candidate_groupB, candidate_groupA):
    """
    Draws bounding boxes and labels for the two candidate detections.
    For the non-special (group B) candidate, the label is shown as:
      "Class: {class}, Conf: {score:.2f}, Dist: {depth}cm, Seg: {segment}"
    For the special (group A) candidate, the label is:
      "Class: {class}, Conf: {score:.2f}, Dist: {depth}cm, Seg: {segment}"
    """
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

def send_to_console(candidate_groupB, candidate_groupA, inference_time):
    """
    Clears the console and prints the final output in the required format.
    The non-special (group B) candidate output is formatted as: "segment depth class"
    The special (group A) candidate output is formatted as: "class depth segment"
    If a candidate is missing, its triplet is "-1 -1 -1".
    Then the final output is a concatenation:
      <groupB_triplet> <groupA_triplet>
    """
    os.system('cls' if os.name == 'nt' else 'clear')
    
    if candidate_groupB is not None:
        out_groupB = f"{candidate_groupB['segment']} {candidate_groupB['depth']} {candidate_groupB['class']}"
    else:
        out_groupB = "-1 -1 -1"
        
    if candidate_groupA is not None:
        out_groupA = f"{candidate_groupA['class']} {candidate_groupA['depth']} {candidate_groupA['segment']}"
    else:
        out_groupA = "-1 -1 -1"
    
    final_output = out_groupB + " " + out_groupA
    print("================ LIVE DATA ================")
    print(f"Inference time: {inference_time:.3f} seconds")
    print(f"Output Values (CV): {final_output}")
    sys.stdout.flush()

def create_pipeline():
    """
    Sets up a DepthAI pipeline with a color camera and a stereo depth node.
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
    stereo.setExtendedDisparity(True)
    stereo.initialConfig.setConfidenceThreshold(100)
    
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    
    # Depth output stream (named "depth")
    xoutDepth = pipeline.createXLinkOut()
    xoutDepth.setStreamName("depth")
    stereo.depth.link(xoutDepth.input)
    
    return pipeline, stereo

def visualize_segments(img):
    """
    Draws seven vertical segments on the RGB camera feed.
    """
    for i in range(1, NUM_SEGMENTS):  # Draw six vertical lines
        x = int(i * SEG_SIZE)
        cv2.line(img, (x, 0), (x, DISPLAY_HEIGHT), (0, 255, 0), 2)  # Green lines
        cv2.putText(img, str(i), (x - 15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)  # Label segments

# ----------------------------
# Main Function
# ----------------------------
def main():
    pipeline, stereo = create_pipeline()
    with dai.Device(pipeline) as device:
        queueColor = device.getOutputQueue(name="color", maxSize=1, blocking=True)
        queueDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        interpreter = tf.lite.Interpreter(model_path=r"D:\Career\IDS\DSP-1-2324-C7-ObstacleDetection\src\3D_CMOS\tests\model_float16_480x480.tflite")
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        input_size = input_details[0]['shape'][1]

        prev_frame_time = time.time()

        while True:
            rgbFrame = queueColor.get().getCvFrame()
            rgbFrame_disp = cv2.resize(rgbFrame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))

            # Draw segment lines
            visualize_segments(rgbFrame_disp)

            inDepth = queueDepth.get()
            depthFrame = inDepth.getFrame()
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

            candidate_groupA = None  
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

                segment = int(x_center // SEG_SIZE)
                candidate = {
                    'class': class_id,
                    'score': score,
                    'depth': depth_val,
                    'segment': segment,
                    'box': (x1_disp, y1_disp, x2_disp, y2_disp)
                }

                if class_id in SPECIAL_CLASSES:
                    if candidate_groupA is None or depth_val < candidate_groupA['depth']:
                        candidate_groupA = candidate
                else:
                    if candidate_groupB is None or depth_val < candidate_groupB['depth']:
                        candidate_groupB = candidate

            if candidate_groupB is not None:
                out_groupB = f"{candidate_groupB['segment']} {candidate_groupB['depth']} {candidate_groupB['class']}"
            else:
                out_groupB = "-1 -1 -1"
            if candidate_groupA is not None:
                out_groupA = f"{candidate_groupA['class']} {candidate_groupA['depth']} {candidate_groupA['segment']}"
            else:
                out_groupA = "-1 -1 -1"
            final_output = out_groupB + " " + out_groupA

            display_candidates(rgbFrame_disp, candidate_groupB, candidate_groupA)
            send_to_console(candidate_groupB, candidate_groupA, inference_time)

            fps_text = f"Live FPS: {live_fps:.2f} | Inference FPS: {1.0/inference_time:.2f}"
            cv2.putText(rgbFrame_disp, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

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
