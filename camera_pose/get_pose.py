import cv2
import numpy as np
import pyrealsense2 as rs
import json
import os
from scipy.spatial.transform import Rotation as R

# === USER INPUTS ===
CHECKERBOARD = (8, 6)  # (columns, rows) of inner corners
square_size = 0.036  # size of one square in meters

# manual calibration values for 827112070893
# fx = 637.2325
# fy = 640.068
# cx = 328.16088
# cy = 221.4209
# dist_coeffs = np.array([-5.52e-02, 2.0616, -6.32195e-03, -1.488937e-03, -9.4295e0])

# manual calibration values for 327122076541
# fx = 906.2524
# fy = 910.88
# cx = 670.619
# cy = 397.239
# dist_coeffs = np.array([0.00249979, 0.69496, 0.0077, 0.004496, -2.26346])
intrinsics1 = np.array(
    [
        906.2524,
        910.88,
        670.619,
        397.239,
        0.00249979,
        0.69496,
        0.0077,
        0.004496,
        -2.26346,
    ]
)


# new manual values for 007522062003
# fx = 898.489
# fy = 902.242
# cx = 644.933
# cy = 361.7777
# dist_coeffs = np.array([0.009, 0.83815, 0.0049, 0.005, -2.8492])
intrinsics2 = np.array(
    [898.489, 902.242, 644.933, 361.7777, 0.009, 0.83815, 0.0049, 0.005, -2.8492]
)

# manual values for 313522070442
# fx = 889.12309
# fy = 892.842
# cx = 649.01486
# cy = 383.25678
# dist_coeffs = np.array([-6.00037e-02, 1.9879e0, 6.765e-03, 2.3826e-03, -8.0174e0])
intrinsics3 = np.array(
    [
        889.12309,
        892.842,
        649.01486,
        383.25678,
        -6.00037e-02,
        1.9879e0,
        6.765e-03,
        2.3826e-03,
        -8.0174e0,
    ]
)


serial_map = {
    "1": ("327122076541", intrinsics1),
    "2": ("007522062003", intrinsics2),
    "3": ("313522070442", intrinsics3),
}

# Display options
print("Select a camera by number:")
print("1. Serial: 327122076541")
print("2. Serial: 007522062003")
print("3. Serial: 313522070442")

# Get user input
choice = input("Enter the number (1, 2, or 3): ").strip()

# Validate and assign
if choice in serial_map:
    serial_number, selected_intrinsics = serial_map[choice]
    fx, fy, cx, cy = selected_intrinsics[:4]
    dist_coeffs = selected_intrinsics[4:]
    print(f"Selected serial number: {serial_number}")
    print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}")
    print(f"Distortion coefficients: {dist_coeffs}")
else:
    print("Invalid choice. Please run the script again and select 1, 2, or 3.")


camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
# dist_coeffs = np.array([0,0,0,0])

# === PREPARE OBJECT POINTS ===
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

pipeline = rs.pipeline()
config = rs.config()
config.enable_device(serial_number)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

cv2.namedWindow("RealSense Checkerboard", cv2.WINDOW_AUTOSIZE)
output_freq = 0
T = np.eye(4)

try:
    while True:
        output_freq += 1
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:

            # Refine the corner locations
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria=(
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    50,
                    0.0001,
                ),
            )

            # Solve PnP to get camera pose
            success, rvec, tvec = cv2.solvePnP(
                objp, corners2, camera_matrix, dist_coeffs
            )

            if success:
                # Convert to a 4x4 homogeneous transformation matrix
                rot, _ = cv2.Rodrigues(rvec)
                # T = np.eye(4)
                T[:3, :3] = rot
                # Convert to inches for better intuition
                # T[:3, 3] = tvec.ravel() * 39.3701
                T[:3, 3] = tvec.ravel()
                opencv_to_blender = np.array(
                    [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
                )
                # invert pose to get camera in world frame (checkerboard frame)
                # then multiply to rotate from opencv to blender frame
                # both of these transformations are crucical to get later scripts working
                T = np.linalg.inv(T)
                T = T @ opencv_to_blender

                if output_freq % 150 == 0:
                    rot = T[:3, :3]
                    euler = R.from_matrix(rot).as_euler("zyx")
                    euler = euler * 180 / np.pi  # Convert radians to degrees
                    print("euler = " + str(euler))
                    print("translation vector = " + str(T[:3, 3]))

                # Draw checkerboard corners on the image
                cv2.drawChessboardCorners(color_image, CHECKERBOARD, corners2, ret)

                # === REPROJECTION ERROR ===
                imgpoints2, _ = cv2.projectPoints(
                    objp, rvec, tvec, camera_matrix, dist_coeffs
                )
                reprojection_error = cv2.norm(corners2, imgpoints2, cv2.NORM_L2) / len(
                    imgpoints2
                )

        cv2.imshow("RealSense Checkerboard", color_image)
        # press c to capture and save pose
        if cv2.waitKey(1) & 0xFF == ord("c"):
            image_number = input("Enter image number: ")
            pose = T.tolist()
            format = {"X_WT": pose}
            format2 = {image_number: format}
            # this can be changed
            # extrinsics_path = "/home/lab/embodied_gaussians/scripts/extrinsics.json"
            extrinsics_path = (
                "/home/lab/embodied_gaussians/scripts/extrinsics_offline.json"
            )
            # make sure extrinsics.json has these brackets { } even if
            # there is no data yet
            if os.path.exists(extrinsics_path):
                with open(extrinsics_path, "r") as f:
                    # automatically parses json file as python dictionary
                    data = json.load(f)
            else:
                data = {}

            # Step 2: Update or insert the new pose
            data[image_number] = {"X_WT": pose}

            # Step 3: Write back the updated dictionary to the JSON file
            with open(extrinsics_path, "w") as f:
                json.dump(data, f, indent=4)
            print("Pose updated")
        # press q to quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Quitting")
            break


finally:
    pipeline.stop()
    cv2.destroyAllWindows()

# standing behind the camera: pos x axis is to the right, pos y axis is down, pos z axis is forward
