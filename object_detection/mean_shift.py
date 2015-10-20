import math
import numpy as np
import cv
import cv2
from sklearn import preprocessing

class ObjectFrame(object):
	def __init__(self, window_x, window_y, rad, dx, dy, frame, url):
		"""Initialize object frame. The following almost function like globals:
			window_x -> size of the frame in the x direction
			window_y -> size of the frame in the y direction
			rad -> size of the "radius" (half the height) of each bin (rectangle)
			dx -> size of width we iterate over for comparing histograms
			dy -> size of height we iterate over for comparing histograms
		The frame is the name of the openCV frame object, and url is the link
		to parse the object from.
		"""
		self.window_x = window_x
		self.window_y = window_y
		self.rad = rad
		self.dx = dx
		self.dy = dy
		self.frame = frame
		self.url = url
	def set_coordinates(self, x, y):
		"""Sets the x and y coordinates of the center of the object to
		track within the image"""
		self.x = x
		self.y = y
	def create_image(self):
		"""Creates an openCV image and resizes it based on the image url. The image
		is a numpy array"""
		img = cv2.imread(self.url)
		self.img = cv2.resize(img, (self.window_x, self.window_y))
	def create_fixed_object(self):
		"""Given knowledge of the x and y coordinates of the object to track, parses
		that from the image numpy array"""
		self.obj = self.img[self.y-self.rad:self.y+self.rad,
			self.x-self.rad:self.x+self.rad]
	def create_fixed_hist(self):
		"""Saves a histogram from the object, and flattens it. Specifically, 
		extracts a 3D RGB color histogram from the image, using 8 bins per 
		channel"""
		hist = cv2.calcHist([self.obj], [0, 1, 2], None, [8, 8, 8], 
			[0, 256, 0, 256, 0, 256])
		self.hist = cv2.normalize(hist).flatten()
	def create_general_object(self, x, y):
		"""Given any x and y, computes an object around it from the image 
		object based on the 'radius'"""
		return self.img[y-self.rad:y+self.rad, x-self.rad:x+self.rad]
	def create_general_hist(self, obj):
		"""Given an object, computes a histogram from it and returns the
		flattened version"""
		hist = cv2.calcHist([obj], [0, 1, 2], None, [8, 8, 8], 
			[0, 256, 0, 256, 0, 256])
		return cv2.normalize(hist).flatten()
	@staticmethod
	def find_hypotenuse(x, y):
		"""Static method to find and return the hypotenuse given x and y"""
		return math.sqrt(x**2 + y**2)
	def calculate_weights(self):
		"""Iterates over x and y starting at the frame radius by dx and dy
		until the window size. At each 'coordinate', computes an object around
		it, computes a histogram, and calculates a weight by comparing that
		histogram to the original frame. Subtracts the normalized distance 
		from the object of the original frame, and creates a list of all of
		these new weights, saved as a numpy array.
		"""
		weights = []
		for x in range(frame.rad, frame.window_x, frame.dx):
			for y in range(frame.rad, frame.window_y, frame.dy):
				obj = new_frame.create_general_object(x,y)
				hist = new_frame.create_general_hist(obj)
				# compare histograms to find weight
				weight = cv2.compareHist(frame.hist, hist, method=cv2.cv.CV_COMP_CORREL)
				# find distance away from old point, and normalize by max distance
				max_distance = float(self.find_hypotenuse(frame.window_x, frame.window_y))
				distance = self.find_hypotenuse(x-frame.x, y-frame.y) / max_distance
				# subtract normalized distance from weight
				weight = weight - distance
				# make sure no weights are negative
				if weight < 0:
					weight = 0
				# append weights to array
				weights.append(weight)
		self.weights = np.array(weights)
	def normalize_weights(self):
		"""Normalizes the weights based on their sum"""
		total_weight = sum(self.weights)
		self.norm_weights = self.weights / float(total_weight)
	def find_new_coordinates(self):
		"""Finds the index with the max weight, and finds the x and y coordinate
		from that based on dx, dy, the radius, and the size of the frame. Then
		saves these coordinates"""
		max_weight = max(self.norm_weights)
		max_index = list(self.norm_weights).index(max_weight)
		new_x = int(self.rad + (max_index / (self.window_y / self.dy))*self.dx)
		new_y = int(self.rad + (max_index % (self.window_y / self.dy))*self.dy)
		self.set_coordinates(new_x, new_y)

if __name__ == '__main__':	
	# Create first frame
	frame = ObjectFrame(640, 480, 20, 40, 40, 'Frame', 'chess1.jpg')
	frame.create_image()
	cv2.namedWindow(frame.frame)

	# Set coordinates of object
	frame.set_coordinates(435, 215)

	# Create and display rectangle based on location of object to track
	cv2.rectangle(frame.img, (frame.x-frame.rad, frame.y-frame.rad), 
		(frame.x+frame.rad, frame.y+frame.rad), 255)
	cv2.imshow(frame.frame, frame.img)
	cv2.waitKey()

	# Create object and histogram of object
	frame.create_fixed_object()
	frame.create_fixed_hist()

	# Set all frames to iterate over (after first frame)
	frame_list = [
		['Frame2', 'chess2.jpg'],
		['Frame3', 'chess3.jpg'],
		['Frame4', 'chess4.jpg'],
		['Frame5', 'chess5.jpg']
	]

	# Iterate over frames and urls in frame_list
	for f in frame_list:	
		# create new frame to compare to original frame
		new_frame = ObjectFrame(640, 480, 20, 40, 40, f[0], f[1])
		new_frame.create_image()

		# compare new frame to original frame
		new_frame.calculate_weights()
		new_frame.normalize_weights()
		new_frame.find_new_coordinates()

		# display new coordinate on screen
		cv2.rectangle(new_frame.img, (new_frame.x-new_frame.rad, new_frame.y-new_frame.rad), 
			(new_frame.x+new_frame.rad, new_frame.y+new_frame.rad), 255)
		cv2.imshow(new_frame.frame, new_frame.img)
		cv2.waitKey()

		# set old frame to new frame, and create new histogram based on center of obj
		frame = new_frame
		frame.create_fixed_object()
		frame.create_fixed_hist()

