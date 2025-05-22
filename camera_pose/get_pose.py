import cv2
import numpy as np
import pyrealsense2 as rs

# === Checkerboard configuration ===
CHECKERBOARD = (5, 8)  # Internal corners (rows, cols)
SQUARE_SIZE = 0.03556  # meters

# === Camera intrinsics (from RealSense D435) ===
fx = 615.6595458984375
fy = 615.6107788085938
cx = 321.5747375488281
cy = 236.33041381835938
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]])
dist_coeffs = np.zeros((4, 1))  # Assuming no distortion

# === 3D object points in the checkerboard frame ===
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# === Set up RealSense pipeline with the given serial number ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_device('827112070893')
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

cv2.namedWindow("RealSense Checkerboard", cv2.WINDOW_AUTOSIZE)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        cv2.drawChessboardCorners(color_image, CHECKERBOARD, corners, ret)

        if ret:
            # Refine the corner locations
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

            # Solve PnP to get camera pose
            success, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)

            if success:
                # Convert to a 4x4 homogeneous transformation matrix
                R, _ = cv2.Rodrigues(rvec)
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.ravel()

                print("\nCamera pose (4x4) w.r.t. checkerboard (table):")
                print(T)

                # Draw checkerboard corners on the image
                cv2.drawChessboardCorners(color_image, CHECKERBOARD, corners2, ret)


        if not ret:
            print('checkerboard not found')
            
        cv2.imshow("RealSense Checkerboard", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
