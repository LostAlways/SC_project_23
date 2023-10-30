%%Feeature Detection Method

%% Calculate the camera velocity vectors for each case such that the squares in
 


%% detect Features, Setup Parmaters, Loop through

%Subscribe to topics
% Create a ROS subscriber for the camera image
rgbSub = rossubscriber('/camera/color/image_raw/');
[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

% Create a ROS publisher for target joint states
[targetJointTrajPub, targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");


jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
pause(2);

%% Set Paramters and Target Pos of images
% Calculate corner points using Harris Feature detector

%img = imread("04.jpg");
img = readImage(rgbSub.LatestMessage);
pause(1);

%targetStart = [0.0767, 0.1775, 0.8846, 0];
  
 Target = [   656.1375  499.1717
              665.9321  213.6515
              928.1994  507.5360
              938.0593  223.7113      % This is the s*, target frame  
                    ];

 % Target = [  326.1736  401.9680;
 %  327.3538  309.4665;
 %  415.7202  402.0635;
 %  415.9699  225.7541;
 %  415.7588  310.8948;
 %  416.2524  491.0537;
 %  417.9471  222.4309;
 %  417.5956  313.5609;
 %  417.5275  399.5548;
 %  503.4552  400.1349;
 %  503.9167  222.9924;
 %  504.4050  313.9383;
 %  504.8839  137.6882;
 %  506.2128  225.8483;
 %  506.2247  311.7736;
 %  505.8927  403.0874;
 %  591.3284  488.3322;
 %  592.3251  312.8407;
 %  593.3806  225.7646;
 %  593.4973  227.7616;
 %  594.5222  315.6428;
 %  595.3488  138.4848;
 %  594.5160  225.9192;
 %  594.7791  400.4590;
 %  679.3782  316.7582;
 %  679.7440  227.4373;
 %  679.9025  400.8095;
 %  679.8978  577.1102;
 %  681.3521  141.1221;
 %  682.4328  230.1317;
 %  681.8477  314.0671;
 %  682.8481  403.4618;
 %  682.4404  488.3966;
 %  765.1379  231.8404;
 %  764.9817  314.9965;
 %  766.3909  401.4201;
 %  766.5410  230.5992;
 %  767.1013  317.4032;
 %  767.0000  404.0000;
 %  769.3396  143.6782;
 %  768.4678  401.4052;
 %  768.6027  403.6829;
 %  768.6273  488.6741;
 %  769.4976  491.0531;
 %  769.9481  576.5766;
 %  849.3744  231.0404;
 %  849.7740  318.4927;
 %  851.9433  146.2659;
 %  852.2142  401.5832;
 %  853.3617  491.3141];

f = 400;                    %focal Point
p = length(img)/2;          %Principle point, midway
Z = 0.15;                   %distance of camera from Picture
l = 0.1; %lambda           % rate

% Current observation of Checkerboard 'S'

while true

  img =  readImage(rgbSub.LatestMessage);
  pause (0.2);
  
  img = rgb2gray(img);
  pause(0.2);
  cornerPoints = detectHarrisFeatures(img,"MinQuality",1);              
  cornerPoints.Location;                     %output of location of cornerpoints

  Obs = [];                    %Empty matrix to store observed values of cornerpoints           
  Obs = cornerPoints.Location; %Populate the Matrix with Obs values
                
  xy = (Target-p)/f;          %(target - principle point) focal point
  Obsxy = (Obs-p)/f;          %( Current observed corners - principle point)/ focal


n = length(Target(:,1));

Lx = [];
for i=1:n
    Lxi = FuncLx(xy(i,1),xy(i,2),Z);
    Lx = [Lx;Lxi];
end


%% Calulate the Vc for current image
e2 = Obsxy-xy;              %Error between observed and target
e = reshape(e2',[],1);      %Reshape into a column vector 
de = -e*l;

%
Lx2 = inv(Lx'*Lx)*Lx';
Vc = -l*Lx2*e           

pause(0.2);


currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message
   opt_Vc = Vc*0.2;

   X = currentJointState(1)-opt_Vc(1);
   Y = currentJointState(2)-opt_Vc(3);
   Z = currentJointState(3)+opt_Vc(2);
   % 
   % X = 0.0767 - opt_Vc(1)
   % Y = 0.1775 - opt_Vc(2)
   % Z = 0.8846 - opt_Vc(3)

                jointTarget = [X,Y,Z,0]; % Remember that the Dobot has 4 joints by default.

%[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');

trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;
send(targetJointTrajPub,targetJointTrajMsg);


%setEndEffectorPose(Vc);
end








%% Lx Function
% By entering x y z values, populate a 2 x 4 matrix inputting the follwoing
% vaues. Output = Lx 2 x 4 matrix

function [Lx] = FuncLx(x,y,Z)

Lx = zeros(2,6);

Lx(1,1) = -1/Z;
Lx(1,2) = 0;
Lx(1,3) = x/Z;
Lx(1,4) = x*y;
Lx(1,5) = -(1+x^2);
Lx(1,6) = y;

Lx(2,1) = 0;
Lx(2,2) = -1/Z;
Lx(2,3) = y/Z;
Lx(2,4) = 1+y^2;
Lx(2,5) = -x*y;
Lx(2,6) = -x;
end



%% Set End Effector function
function setEndEffectorPose(Vc)
    % Define the end effector position
    endEffectorPosition = Vc(1:3); % Extract XYZ from Vc
    endEffectorRotation = Vc(4:6); % Extract RPY from Vc

    % Create a ROS publisher for the target end effector pose
    %[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

    % Set the end effector position
    targetEndEffectorMsg.Position.X = endEffectorPosition(1);
    targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
    targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

    % Convert RPY to quaternion and set the end effector orientation
    % qua = eul2quat(endEffectorRotation);
    % %targetEndEffectorMsg.Orientation.W = qua(1);
    % targetEndEffectorMsg.Orientation.X = qua(1);
    % targetEndEffectorMsg.Orientation.Y = qua(2);
    % targetEndEffectorMsg.Orientation.Z = qua(3);

    % Send the target end effector pose
    send(targetEndEffectorPub, targetEndEffectorMsg);
end

%%
%Notes

%Hold on
%imshow(img)
%plot(cornerpoints)



