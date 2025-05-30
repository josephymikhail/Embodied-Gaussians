import cv2
import numpy as np
import pyrealsense2 as rs
import json
import os

# === USER INPUTS ===
CHECKERBOARD = (8, 6)  # (columns, rows) of inner corners
square_size = 0.036  # size of one square in meters


#from real sense for 007522062003
#fx = 607.841552734375
#fy = 606.5180053710938
#cx = 319.40252685546875
#cy = 237.56298828125

#realsense values for 827112070893
#fx = 615.6595458984375
#fy = 615.6107788085938
#cx = 321.5747375488281
#cy = 236.33041381835938

#realsense values for 327122076541
#fx = 604.9152221679688
#fy = 604.6057739257812
#cx = 328.6421203613281
#cy = 255.98118591308594


#manual calibration values for 827112070893
fx = 554.637
fy = 594.6277
cx = 354.3337
cy = 329.573
dist_coeffs = np.array([4.406e-02, 3.3612e-01, 4.5784e-04, 3.164e-02, -5.6377e-01])

#old manual calibration values for 007522062003
#fx = 538.3586
#fy = 546.76
#cx = 281.4157
#cy = 277.938
#dist_coeffs = np.array([0.2514, -0.78757, 0.01096, -0.02877, 0.43484])

#new manual values for 007522062003
#fx = 614.559
#fy = 617.694
#cx = 319.9279
#cy = 218.358
#dist_coeffs = np.array([3.715e-02, 9.0511e-01, -1.247e-02, 1.9958e-03, -4.083e0])

camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]])
#dist_coeffs = np.array([0,0,0,0])

# === PREPARE OBJECT POINTS ===
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

pipeline = rs.pipeline()
config = rs.config()
#breakpoint()
#007522062003
#827112070893
#327122076541
serial_number = '827112070893'
config.enable_device(serial_number)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
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
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001)
            )

            # Solve PnP to get camera pose
            success, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)

            if success:
                # Convert to a 4x4 homogeneous transformation matrix
                R, _ = cv2.Rodrigues(rvec)
                #T = np.eye(4)
                T[:3, :3] = R
                # Convert to inches for better intuition
                #T[:3, 3] = tvec.ravel() * 39.3701
                T[:3, 3] = tvec.ravel()
                #inv_T = np.linalg.inv(T)
                #T = np.linalg.inv(T)

                #look more into this
                opencv_to_blender = np.array([
                    [1,  0,  0,  0],
                    [0,  1,  0,  0],
                    [0,  0,  1,  0],
                    [0,  0,  0,  1]
                ])
                T = np.linalg.inv(T)
                T = T @ opencv_to_blender

                

                # Draw checkerboard corners on the image
                cv2.drawChessboardCorners(color_image, CHECKERBOARD, corners2, ret)

                # === REPROJECTION ERROR ===
                imgpoints2, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
                reprojection_error = cv2.norm(corners2, imgpoints2, cv2.NORM_L2) / len(imgpoints2)

                if output_freq % 150 == 0:
                        print(T)
                        print(reprojection_error)



        cv2.imshow("RealSense Checkerboard", color_image)
        #press q to exit loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pose = T.tolist()
            format = {"X_WT": pose}
            format2 = {serial_number: format}
            #this can be changed 
            extrinsics_path = "/home/lab/embodied_gaussians/scripts/extrinsics.json"

            #make sure extrinsics.json has these brackets { } even if 
            #there is no data yet
            if os.path.exists(extrinsics_path):
                with open(extrinsics_path, 'r') as f:
                    #automatically parses json file as python dictionary
                    data = json.load(f)
            else:
                data = {}

            # Step 2: Update or insert the new pose
            data[serial_number] = {"X_WT": pose}

            # Step 3: Write back the updated dictionary to the JSON file
            with open(extrinsics_path, 'w') as f:
                json.dump(data, f, indent=4)
    
            break


finally:
    pipeline.stop()
    cv2.destroyAllWindows()

#standing behind the camera: pos x axis is to the right, pos y axis is down, pos z axis is forward



