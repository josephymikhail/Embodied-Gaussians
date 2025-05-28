import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_device('827112070893')
#827112070893
#007522062003
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

color_stream = profile.get_stream(rs.stream.color)
intr = color_stream.as_video_stream_profile().get_intrinsics()

print("fx:", intr.fx)
print("fy:", intr.fy)
print("cx:", intr.ppx)
print("cy:", intr.ppy)

dist_coeffs = np.array(intr.coeffs[:4])
print(dist_coeffs)


pipeline.stop()
