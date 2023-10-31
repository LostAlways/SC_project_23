READ ME FILE

Moving the dobot:
    Dobot drivers installed from online sources are used to move the Dobot to the locations the team specifies


Calculating the Intrinsic Parameters
    - 10 pictures taken around the Checkerboard using the Realsense RGBD camera using the code file:
    - Use week 2 content to work out the intrinsic parameters from the RGBD camera:
    - Code used to work out the transform from the base of the Dobot to the camera and from the end effector to the checkerboard


Eye-in-hand calibration
    Zachery Taylor's Matlab toolbox was used in the calibration process ("Camera to Robotic Arm Calibration" toolbox ). 10 pictures were taken
of the checkerbaord in different poses and the arm base to the robotic end effector position was recorded. External calibration code of "CalCamArm.m" was
used to get the end effector to the checkerboard transform and the robot base to the camera transform.  The folder "RequiredFuncCalCam" has the necessary 
coding files required to run the CalCamArm.m code.



Visual Servoing
    - A code file is created to work out camera velocity vector for each iteration that the camera is moved 
    - That vectors tells you the transform from desired to current pose of the checkerboard. The vector is implemented
        into the transform of the robot's end effector to move follow the checkerboard accordingly
    - The visual servoing code is placed in a while loop in order to constantly read the image from the sensors anc 
       control the dobot according to it. The code file that uses this methodology to perform visual servoing is "DemoVisServo.m"
    - The file gets the the image from the camera and constantly works out the camera velocity and moves the joints according to the camera velocity
        vector



