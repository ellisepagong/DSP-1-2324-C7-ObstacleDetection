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

# Function to create stereo depth pair
def getStereoPair(pipeline, monoLeft, monoRight):
    stereo = pipeline.createStereoDepth()
    stereo.setLeftRightCheck(True)  # Marks occluded pixels as invalid
    stereo.setSubpixel(True)  # Increases depth precision
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    return stereo

if __name__ == "__main__":
    # Initialize pipeline
    pipeline = dai.Pipeline()

    # Setup mono cameras
    monoLeft = getMonoCamera(pipeline, isLeft=True)
    monoRight = getMonoCamera(pipeline, isLeft=False)

    # Setup output streams
    xoutLeft = pipeline.createXLinkOut()
    xoutLeft.setStreamName("left")
    xoutRight = pipeline.createXLinkOut()
    xoutRight.setStreamName("right")

    monoLeft.out.link(xoutLeft.input)
    monoRight.out.link(xoutRight.input)

    # Setup stereo depth
    stereo = getStereoPair(pipeline, monoLeft, monoRight)
    
    xoutDisp = pipeline.createXLinkOut()
    xoutDisp.setStreamName("disparity")
    stereo.disparity.link(xoutDisp.input)

    # Start pipeline
    with dai.Device(pipeline) as device:
        queueLeft = device.getOutputQueue(name="left", maxSize=1)
        queueRight = device.getOutputQueue(name="right", maxSize=1)
        queueDisp = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)

        multiplier = 255 / stereo.initialConfig.getMaxDisparity()  # Fix for deprecated method

        while True:
            leftFrame = getFrame(queueLeft)
            rightFrame = getFrame(queueRight)
            disparityFrame = queueDisp.get().getFrame()

            # Normalize disparity for better visualization
            disparityFrame = (disparityFrame * multiplier).astype(np.uint8)
            disparityFrame = cv2.applyColorMap(disparityFrame, cv2.COLORMAP_JET)

            # Show outputs
            out = np.uint8(leftFrame/2 + rightFrame/2)
            cv2.imshow("Camera Feed", out)
            cv2.imshow("Disparity Map", disparityFrame)

            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
