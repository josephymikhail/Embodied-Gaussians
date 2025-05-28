import pyrealsense2 as rs
import numpy as np
import cv2
import os

SAVE_DIR = "/home/lab/embodied_gaussians/camera_pose/get_intrinsics"


pipeline = rs.pipeline()
config = rs.config()
config.enable_device('007522062003')
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("Press 's' to save image, 'q' to quit.")

try:
    i = 0
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        cv2.imshow("RealSense Color", color_image)
        key = cv2.waitKey(1)
        if key == ord('s'):
            filename = f"{SAVE_DIR}/image_{i:02d}.png"
            cv2.imwrite(filename, color_image)
            print(f"Saved {filename}")
            i += 1
        elif key == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
