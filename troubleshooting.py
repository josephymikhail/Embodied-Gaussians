import pyrealsense2 as rs
ctx = rs.context()
for dev in ctx.query_devices():
    print("Device:", dev.get_info(rs.camera_info.name))
    for s in dev.query_sensors():
        print("  Sensor:", s.get_info(rs.camera_info.name))
