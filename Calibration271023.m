% Auto-generated by cameraCalibrator app on 27-Oct-2023
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/1.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/10.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/2.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/3.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/4.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/5.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/6.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/7.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/8.jpg',...
    '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23/9.jpg',...
    };
% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 15;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
