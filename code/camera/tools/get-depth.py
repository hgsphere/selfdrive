import pyrealsense2 as rs
from numpy import asanyarray, zeros, float32
from cv2 import (resize as cv_resize,
				 GaussianBlur as cv_GaussianBlur,
				 INTER_LINEAR as cv_INTER_LINEAR,
				 imwrite as cv_imwrite)
import numpy as np
import os


class testPipe():
	def __init__(self):
		self.config = rs.config()
		shape_rgb = (424, 240)
		self.shape = (480, 270)
		self.frame_rate = 30
		self.config.enable_stream(rs.stream.depth,
								  self.shape[0], self.shape[1],
								  rs.format.z16, self.frame_rate)
		self.config.enable_stream(rs.stream.color,
								  shape_rgb[0], shape_rgb[1],
								  rs.format.bgr8, self.frame_rate)
		self.pipeline = rs.pipeline()

		self.profile = None
		self.depth_sensor = None
		self.depth_scale = None
		self.clipping_distance = None

	def start(self):
		self.profile = self.pipeline.start(self.config)
		self.depth_sensor = self.profile.get_device().first_depth_sensor()
		self.depth_scale = self.depth_sensor.get_depth_scale()

		clipping_distance_in_meters = 0.3
		self.clipping_distance = clipping_distance_in_meters / self.depth_scale

	# def __del__(self):
	# 	self.pipeline.stop()

	def getFrame(self):
		frames = self.pipeline.wait_for_frames()
		depth = frames.get_depth_frame()
		depthData = asanyarray(depth.get_data())
		depthFloat = zeros(depthData.shape, dtype=float32)

		grey_color = 153
		# bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

		for i in range(depth.width):
			for j in range(depth.height):
				distance = depthData[j, i]
				if distance > self.clipping_distance:
					depthFloat[j, i] = grey_color
				else:
					depthFloat[j, i] = distance

		# depthFrame = cv_resize(depthData,
		# 					   (self.shape[0]//2, self.shape[1]//2),
		# 					   interpolation=cv_INTER_LINEAR)
		gauss = cv_GaussianBlur(depthFloat, (5, 5), -1)
		return gauss


def main():
	"""Example of how to get and parse depth frames"""
	print("tests beginning...")
	tp = testPipe()
	tp.start()
	outPath = os.path.abspath("./depthFrames")

	for i in range(10):
		frame = tp.getFrame()
		if i < 5:
			continue
		input("waiting...")
		cv_imwrite(os.path.join(outPath, "frame{}.jpeg".format(i)), frame)
		# print(depthData)


if __name__ == '__main__':
	main()
