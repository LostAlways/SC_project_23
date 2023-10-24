READ ME FILE


Calculating the Intrinsic Parameters
    - 9 pictures taken around the Checkerboard using the Realsense RGBD camera
    - Use week 2 content to work out the intrinsic parameters from the RGBD camera
    - The camera is mounted onto the end effector with an offset



Working out the Checkerboard coordinates in the Global world coordinate using intrinsic parameters
    - Code to work it out
    - Get Z value from the camera to the checkerboard by using the depth function of the camera


Eye-in-hand calibration
    - Do some mathematical transforming to work out the transform from the end effector to the camera
    - Use the ax = bx formula. Use Dominik's function to work it out in matlab.


Visual Servoing
    - Use the maths taught to work out the "camera velocity" vector
    - That vectors tells you the transform from desired to current pose of the checkerboard
    - Do some multiplication to figure out the transform and put this process in an end less while loop (I believe)