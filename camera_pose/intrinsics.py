import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

color_stream = profile.get_stream(rs.stream.color)
intr = color_stream.as_video_stream_profile().get_intrinsics()

print("fx:", intr.fx)
print("fy:", intr.fy)
print("cx:", intr.ppx)
print("cy:", intr.ppy)

pipeline.stop()
