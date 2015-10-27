"""This script is openCV's implementation of cam shift from their documentation.
We have made some minor modifications to deal with errors, and to initialize the
starting positions of the objects to be tracked in our videos."""


import numpy as np
import cv2
import cv

# Set video name
video_name = 'object.mp4'

if video_name == 'ballislife.mp4':
    r,h,c,w = 130,50,355,50
elif video_name == 'object.mp4':
    r,h,c,w = 113,8,274,8

cap = cv2.VideoCapture(video_name)
 
# take first frame of the video
ret,frame = cap.read()
 
# setup initial location of window
track_window = (c,r,w,h)

# set up the ROI for tracking
roi = frame[r:r+h, c:c+w]
hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

# Setup the termination criteria, either 10 iteration or move by atleast 1 pt
term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

while(1):
    ret ,frame = cap.read()

    if ret == True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

        # apply meanshift to get the new location
        ret, track_window = cv2.CamShift(dst, track_window, term_crit)

        # Revert to values from last iteration of loop if cam shift loses object
        if track_window == (0, 0, 0, 0):
            ret = last_ret
            track_window = last_window

        # Draw it on image
        x,y,w,h = track_window
        cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
        cv2.imshow('frame', frame)

        k = cv2.waitKey(60) & 0xff
        if k == 27:
            break
        else:
            cv2.imwrite(chr(k)+".jpg", frame)

        # save previous value of ball in case cam shift loses object
        if track_window != (0, 0, 0, 0):
            last_ret = ret
            last_window = track_window

    else:
        break

cv2.destroyAllWindows()
cap.release()
