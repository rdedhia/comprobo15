import math
import numpy as np
import cv
import cv2
from sklearn import preprocessing

window_x = 640
window_y = 480

cv2.namedWindow('frame')
img = cv2.imread('chess1.jpg')
img = cv2.resize(img, (window_x, window_y))

init_x = 435
init_y = 215
rad = 20
dx = 40
dy = 40

cv2.rectangle(img, (init_x-rad, init_y-rad), (init_x+rad, init_y+rad), 255)
cv2.imshow('frame', img)
cv2.waitKey()

obj = img[init_y-rad:init_y+rad, init_x-rad:init_x+rad]

# extract a 3D RGB color histogram from the image, using 8 bins per channel, 
# normalize, and update the index
init_hist = cv2.calcHist([obj], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
init_hist = cv2.normalize(init_hist).flatten()


img2 = cv2.imread('chess2.jpg')
img2 = cv2.resize(img2, (window_x, window_y))

weights = []
n = (window_x / 20) * (window_y / 20)

for x in range(20, window_x-20, dx):
	for y in range(20, window_y-20, dy):
		obj = img2[y-rad:y+rad, x-rad:x+rad]
		hist = cv2.calcHist([obj], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
		hist = cv2.normalize(hist).flatten()
		# Compare histograms
		match = cv2.compareHist(init_hist, hist, method=cv2.cv.CV_COMP_CORREL)
		# find distance away from old point
		z = math.sqrt((x - 435)**2 + (y - 215)**2)
		# dealing w/ weird negative values
		if match < 0:
			match = 0
		weight = match / float(z)
		weights.append(weight)

weights = np.array(weights)
total_weight = sum(weights)
norm_weights = weights / float(total_weight)

print len(weights)

new_x = 0
new_y = 0

for i, weight in enumerate(norm_weights):
	new_x += (20 + (i % 20)*40) * weight
	new_y += (20 + (i / 20)*40) * weight

new_x = int(new_x)
new_y = int(new_y)
print new_x
print new_y

cv2.rectangle(img2, (new_x-rad, new_y-rad), (new_x+rad, new_y+rad), 255)
cv2.imshow('frame', img2)
cv2.waitKey()

