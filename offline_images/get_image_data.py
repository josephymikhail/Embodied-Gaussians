# The idea of this script is that you have a realsense camera that you are moving around and everytime you press a certain key (say the space bar) it will take a snapshot and output the color image, depth image, and depth scale(?)
# One thing I need to look into: how will I get the extrinsics?
# For now I will write this script assuming we are using one camera and the intrinsics are fixed.


from realsense import MultiRealsense
import time
import pickle
import numpy as np

all_camera_data, all_intrinsics, all_depth_scale = {}, {}, {}
# breakpoint()

print("Storing image data.")
# 327122076541
# 313522070442
serial_numbers = ["327122076541"]
with MultiRealsense(serial_numbers=serial_numbers, enable_depth=True) as realsenses:
    realsenses.set_exposure(177, 70)
    realsenses.set_white_balance(4600)
    time.sleep(1)  # Give some time for color to adjust
    all_camera_data = realsenses.get()
    all_intrinsics = realsenses.get_intrinsics()
    all_depth_scale = realsenses.get_depth_scale()
    # breakpoint()


# breakpoint()
image_number = input("Enter image number: ")

path_file = f"/home/lab/embodied_gaussians/offline_images/image_{image_number}_data.pkl"


with open(path_file, "wb") as f:
    pickle.dump(
        {
            "all_camera_data": all_camera_data,
            "all_intrinsics": all_intrinsics,
            "all_depth_scale": all_depth_scale,
        },
        f,
    )


# K is intrinsics matrix which is type numpy.ndarray
# X_WC is also numpy.ndarray which is the classic pose matrix

# all_camera_data is a dict of size 1
# all_intrinsics is a dict of size 1
# all_depth_scale is a dict of size 1


"""
format of extrinsics input to function:
{'327122076541': ExtrinsicsData(X_WC=array([[ 0.14973847,  0.68857565, -0.70953645, -0.72297208],
       [ 0.98788852, -0.13371788,  0.07871341,  0.38877831],
       [-0.04067757, -0.71272933, -0.70025869, -0.59911663],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])), '313522070442': ExtrinsicsData(X_WC=array([[ 0.31522863, -0.62380601,  0.71519017,  1.07536368],
       [-0.94867951, -0.18707287,  0.2549724 ,  0.22413078],
       [-0.02526064, -0.75886086, -0.65076271, -0.55055822],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]))}

all_camera_data, all_intrinsics, all_depth_scale are all dicts of length 2
depth scale = 0.0010000000474974513
len(color) = 720
len(mask) = 720

(Pdb) type(datapoints[0])
<class 'embodied_gaussians.scene_builders.domain.MaskedPosedImageAndDepth'>
(Pdb) type(datapoints)
<class 'list'>

"""
