import depthai as dai
import cv2
import numpy as np

# Function to get a frame from the output stream
def getFrame(queue):
    frame = queue.get()
    return frame.getCvFrame()

# Function to setup mono cameras
def getMonoCamera(pipeline, isLeft):
    mono = pipeline.createMonoCamera()
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    
    if isLeft:
        mono.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Fix for LEFT
		# DEPRECATED: mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    else:
        mono.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Fix for RIGHT
        # DEPRECATED: mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    return mono

pipeline = dai.Pipeline()

# setup left and right cams
monoLeft = getMonoCamera(pipeline, True)
monoRight = getMonoCamera(pipeline, False)

def normalize_bbox(bbox, frame_shape):
    h, w = frame_shape[:2]
    x_min = int(bbox[0] * w)
    y_min = int(bbox[1] * h)
    x_max = int(bbox[2] * w)
    y_max = int(bbox[3] * h)
    return x_min, y_min, x_max, y_max

# neural network node
nn = pipeline.create(dai.node.NeuralNetwork)
nn.setBlobPath(model_path=r"D:\Career\IDS\DSP-1-2324-C7-ObstacleDetection\src\3D_CMOS\tests\best.pt")

# RGB camera node
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(450, 450) # model input image size
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Depth node
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

# link camera feed to model input layer
cam_rgb.preview.link(nn.input)

# Define output streams
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

nn_out = pipeline.create(dai.node.XLinkOut)
nn_out.setStreamName("nn")
nn.out.link(nn_out.input)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

with dai.Device(pipeline) as device:
    queueRGB = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    queueNN = device.getOutputQueue(name="nn", maxSize=1, blocking=False)
    queueDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

    while True:

        out_RGB = queueRGB.get()
        out_nn = queueNN.get()
        out_dep = queueDepth.get()

        frame = out_RGB.getCvFrame()
        depth = in_depth.getFrame()

        results = np.array(out_nn.getFirstLayerFp16())

        for i in range(0, len(results), 6):
            x_min, y_min, x_max, y_max, confidence, class_id = results[i:i+6]

            if confidence > 0.5:  # Confidence threshold
                x1, y1, x2, y2 = denormalize_bbox([x_min, y_min, x_max, y_max], frame.shape)

                # Estimate depth by taking the median value inside the bounding box
                object_depth = np.median(depth[y1:y2, x1:x2])

                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"Class {int(class_id)}: {object_depth:.2f}mm"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Object Detection", frame)

        if cv2.waitKey(1) == ord('q'):
            break