import pyrealsense2 as rs
import numpy as np
class RealSenseDriver:
def __init__(self, width=1280, height=720, fps=30):
self.pipeline = rs.pipeline()
self.config = rs.config()
self.config.enable_stream(rs.stream.depth,
width, height,
rs.format.z16, fps)
self.config.enable_stream(rs.stream.color,
width, height,
rs.format.bgr8, fps)
def start(self):
self.pipeline.start(self.config)
self.align = rs.align(rs.stream.color)
def get_frames(self):
frames = self.pipeline.wait_for_frames()
aligned = self.align.process(frames)
depth = np.asanyarray(
aligned.get_depth_frame().get_data())
color = np.asanyarray(
aligned.get_color_frame().get_data())
return color, depth
def get_pointcloud(self):
# Generate point cloud from depth
pc = rs.pointcloud()
frames = self.pipeline.wait_for_frames()
depth = frames.get_depth_frame()
pc.map_to(frames.get_color_frame())
points = pc.calculate(depth)
return np.asanyarray(points.get_vertices())
def stop(self):
self.pipeline.stop()
