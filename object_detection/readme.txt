There are four scripts in this folder

mean_shift.py
- Our implementation of object tracking, loosely based on mean shift, tracks a green marker cap through 5 "frames". It utilizes color histogram comparisons to function

video_mean_shift.py
- This is our implementation of object tracking on two videos. There are two variables that can be set to customize the behavior of the script. The algorithm can use either rgb or hsv values by setting `rgb` to True or False in the main loop of the script. In addition, the script is written to work for two videos. `object.mp4` tracks a bottle cap moving across a table, and `ballislife.mp4` tracks a green ball rolling around the floor. These can be switched by changing the `video_name` variable in the main loop.

opencv_mean_shift.py
- This script uses openCV's mean shift algorithm on our two videos. Again, the `video_name` variable can be changed to `object.mp4` or `ballislife.mp4`.

opencv_cam_shift.py
- This script does the same as opencv_mean_shift.py, except using cam shift instead. We had to modify openCV's implementation slightly to deal with error handling.