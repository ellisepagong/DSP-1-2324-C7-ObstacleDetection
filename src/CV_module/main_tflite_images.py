import numpy as np
import time 
from tflite_runtime.interpreter import Interpreter
import cv2
import os

def preprocess_input(image, input_size):
    print("Preprocessing input image...")
    resized_img = cv2.resize(image, (input_size, input_size))
    normalized_img = resized_img / 255.0  # Normalize if required by your model
    input_tensor = np.expand_dims(normalized_img, axis=0).astype(np.float32)
    print(f"Input tensor shape: {input_tensor.shape}")
    return input_tensor

def non_max_suppression(detections, iou_threshold):
    print("Applying Non-Maximum Suppression (NMS)...")
    if len(detections) == 0:
        print("No detections to process for NMS.")
        return []
    
    detections = np.array(detections)
    x1 = detections[:, 2]
    y1 = detections[:, 3]
    x2 = detections[:, 4]
    y2 = detections[:, 5]
    scores = detections[:, 1]
    class_ids = detections[:, 0]
    
    indices = cv2.dnn.NMSBoxes(
        bboxes=list(zip(x1, y1, x2 - x1, y2 - y1)),
        scores=scores.tolist(),
        score_threshold=0.5,
        nms_threshold=iou_threshold
    )
    
    indices = indices.flatten() if len(indices) > 0 else []
    print(f"NMS kept {len(indices)} out of {len(detections)} detections.")
    return [detections[i] for i in indices]

def process_detections(output_data, img_shape, conf_threshold=0.5, iou_threshold=0.5):
    print("Processing detections...")
    # Reshape and transpose output data
    output_data = np.squeeze(output_data)  # Shape: [14, 8400]
    output_data = np.transpose(output_data)  # Shape: [8400, 14]
    
    detections = []
    img_height, img_width = img_shape[:2]
    
    for idx, detection in enumerate(output_data):
        # Extract bounding box and class probabilities
        x_center, y_center, width, height = detection[0:4]
        class_scores = detection[4:]
        
        # Get the class with the highest score
        class_id = np.argmax(class_scores)
        score = class_scores[class_id]
        
        if score > conf_threshold:
            # Convert normalized coordinates to image coordinates
            x_center *= img_width
            y_center *= img_height
            width *= img_width
            height *= img_height
            
            x1 = x_center - width / 2
            y1 = y_center - height / 2
            x2 = x_center + width / 2
            y2 = y_center + height / 2
            
            # Append detection
            detections.append([class_id, score, x1, y1, x2, y2])
            
            print(f"Detection {len(detections)}: Class {class_id}, Score {score:.2f}")
    
    # Apply Non-Maximum Suppression (NMS)
    detections = non_max_suppression(detections, iou_threshold)
    
    return detections

def main():
    print("Starting the object detection script...")
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
    
    # Desired input resolution
    desired_resolution = 640
    
    # Load TFLite model
    print("Loading TFLite model...")
    interpreter = Interpreter(model_path="model_float16.tflite")
    interpreter.allocate_tensors()
    print("Model loaded successfully.")
    
    # Get input and output details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    print(f"Input details: {input_details}")
    print(f"Output details: {output_details}")
    
    # Set input size to desired resolution
    input_size = desired_resolution
    print(f"Input size set to: {input_size}x{input_size}")
    
    # Paths for input and output images
    input_dir = "testImages/inputImages"
    output_dir = "testImages/outputImages"
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get list of image files in the input directory
    image_files = [f for f in os.listdir(input_dir) if f.lower().endswith('.jpg')]
    
    print(f"Found {len(image_files)} images in {input_dir}")
    
    # Start timing the entire script
    script_start_time = time.time()
    
    for idx, image_file in enumerate(image_files):
        print(f"\nProcessing image {idx+1}/{len(image_files)}: {image_file}")
        image_path = os.path.join(input_dir, image_file)
        output_path = os.path.join(output_dir, image_file)
        
        # Read the image
        img = cv2.imread(image_path)
        if img is None:
            print(f"Failed to read image {image_path}. Skipping.")
            continue
        
        # Start timing for the current image
        image_start_time = time.time()
        
        # Preprocess the image
        input_tensor = preprocess_input(img, input_size)
        
        # Run inference
        print("Running inference...")
        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()
        
        # Get outputs
        output_data = interpreter.get_tensor(output_details[0]['index'])  # Shape: [1, 14, 8400]
        
        # Process detections
        detections = process_detections(output_data, img.shape, conf_threshold=0.5, iou_threshold=0.5)
        print(f"Total detections after NMS: {len(detections)}")
        
        # Draw detections
        for detection in detections:
            class_id, score, x1, y1, x2, y2 = detection
            x1 = int(max(0, x1))
            y1 = int(max(0, y1))
            x2 = int(min(img.shape[1], x2))
            y2 = int(min(img.shape[0], y2))
            class_name = classes_dict.get(int(class_id), 'Unknown')
            
            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label background
            label = f'{class_name}: {score:.2f}'
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            label_ymin = max(y1, label_size[1] + 10)
            cv2.rectangle(img, (x1, label_ymin - label_size[1] - 10), (x1 + label_size[0], label_ymin + 5), (0, 255, 0), cv2.FILLED)
            
            # Put label text
            cv2.putText(img, label, (x1, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Save the output image
        cv2.imwrite(output_path, img)
        print(f"Saved output image to {output_path}")
        
        # End timing for the current image
        image_end_time = time.time()
        print(f"Time taken for image {image_file}: {image_end_time - image_start_time:.2f} seconds")
    
    # End timing the entire script
    script_end_time = time.time()
    print(f"Total time taken to process all images: {script_end_time - script_start_time:.2f} seconds")

if __name__ == '__main__':
    main()
