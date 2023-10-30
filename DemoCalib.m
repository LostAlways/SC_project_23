%% Using a derivatio of Zachary Taylors Method to complete the calibration. Uses CalCamEIH. to complete and
 % and outputs of images and Transforms attained from
 % CameraCheckerboardCalibration.m script.


%% Using the CameraCheckerboardCalibration.m Method to attain the transforms and images
%  to complete the method below
T_EE = cell(1,10); %Assume we only have 10 matrices

T_EE{1} = [0.9551 0.2963 0 0.1952; -0.2963 0.9551 0 -0.0605; 0 0 1.000 0.1107; 0 0 0 1.000;];
T_EE{2} = [0.9804 0.1969 0 0.2007; -0.1969 0.9804 0 -0.0403; 0 0 1.000 0.1102; 0 0 0 1.000;];
T_EE{3} = [0.9743 0.2253 0 0.1838; -0.2253 0.9743 0 -0.0425; 0 0 1.000 0.0827; 0 0 0 1.000;];
T_EE{4} = [0.9992 -0.0409 0 0.2025; 0.0409 0.9992 0 0.0083; 0 0 1.000 0.1132; 0 0 0 1.000;];
T_EE{5} = [0.9993 -0.0386 0 0.1982; 0.0386 0.9993 0 0.0077; 0 0 1.000 0.0825; 0 0 0 1.000;];
T_EE{6} = [0.9992 -0.0388 0 0.2108; 0.0388 0.9992 0 0.0082; 0 0 1.000 0.0744; 0 0 0 1.000;];
T_EE{7} = [0.9868 0.1619 0 0.2061; -0.1619 0.9868 0 -0.0338; 0 0 1.000 0.0712; 0 0 0 1.000;];
T_EE{8} = [0.9959 0.0901 0 0.1806; -0.0901 0.9959 0 -0.0163; 0 0 1.000 0.0314; 0 0 0 1.000;];
T_EE{9} = [0.9646 0.2639 0 0.1921; -0.2639 0.9646 0 -0.0525; 0 0 1.000 0.0712; 0 0 0 1.000;];
T_EE{10} = [0.9976 0.0697 0 0.2145; -0.0697 0.9976 0 -0.0150; 0 0 1.000 0.0904; 0 0 0 1.000;];

%% Add the images attained from performing CameraCheckerboardCalibration.m

imageFolder = '/home/david/Desktop/S&C/Assessment/DobotImg27-10-23'; %Images of s=checkerboard taken     
squareSize = 15;     %% Square size in mm


% Convert T_EE to a numeric array
numImages = numel(T_EE);
armMat = zeros(4, 4, numImages);
for i = 1:numImages
    armMat(:, :, i) = cell2mat(T_EE(i));
end

%% Now you can call the function CalCamArmEIH with armMat, a derivation of Zachary Taylors method that implements an Eye In Hand Calibration
[TBase, TEnd, cameraParams, TBaseStd, TEndStd, pixelErr] = CalCamArmEIH(imageFolder, armMat, squareSize, 'maxBaseOffset', 1);



%print results from output of CalCamArmEIH
fprintf('\nFinal camera to arm base transform is\n')
disp(TBase);

fprintf('Final end effector to checkerboard transform is\n')
disp(TEnd);

fprintf('Final camera matrix is\n')
disp(cameraParams);

fprintf('Final camera radial distortion parameters are\n')
disp(cameraParams.RadialDistortion);

 fprintf('Final camera tangential distortion parameters are\n')
 disp(cameraParams.TangentialDistortion);


trplot(eye(4),'frame','B','color','b','length',1);
hold on;
trplot(inv(TBase),'frame','K','color','r','length',1);
hold off;

m=inv(TBase);
R=tr2rt(m);
Q=Quaternion(R);
S=transl(m);