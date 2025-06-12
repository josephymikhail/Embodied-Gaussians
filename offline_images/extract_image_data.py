import numpy as np

image_number = input("Enter image number: ")

path_file = f"/home/lab/embodied_gaussians/offline_images/image_{image_number}_data.npz"
image_data = np.load(path_file)

info = image_data.files
camera_data = info[0]
intrinsics = info[1]
