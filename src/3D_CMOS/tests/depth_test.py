import cv2
import depthai as dai
import numpy as np
import time

pipeline = dai.Pipeline()

left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setSubpixel(True)
stereo.setSubpixelFractionalBits(5)
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(False)
stereo.initialConfig.setDisparityShift(1)
stereo.setOutputSize(640, 400)

left.out.link(stereo.left)
right.out.link(stereo.right)

depth_out = pipeline.create(dai.node.XLinkOut)
depth_out.setStreamName("depth")
stereo.depth.link(depth_out.input)

with dai.Device(pipeline) as device:
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    while True:
        inDepth = depthQueue.get()
        depthFrame = inDepth.getFrame()  # Depth values in mm
        timestamp = time.time()

        # Clip depth values to a maximum of 5100 mm (5.1 m)
        depthFrame = np.clip(depthFrame, 0, 5100)

        min_depth_cm = np.min(depthFrame) / 10.0
        max_depth_cm = np.max(depthFrame) / 10.0
        center_depth_cm = depthFrame[depthFrame.shape[0] // 2, depthFrame.shape[1] // 2] / 10.0

        print(f"[{timestamp:.2f}] Min Depth: {min_depth_cm:.2f} cm, Max Depth: {max_depth_cm:.2f} cm, Center Depth: {center_depth_cm:.2f} cm")

        depth_norm = cv2.normalize(depthFrame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        cv2.imshow("Depth Map", depth_colormap)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
