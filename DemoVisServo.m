%%Feeature Detection Method

%% Calculate the camera velocity vectors for each case such that the squares in
 


%% detect Features, Setup Parmaters, Loop through

%% Subscribe to required topics to capture images and move dobot

% Create a ROS subscriber for the camera image
rgbSub = rossubscriber('/camera/color/image_raw/');

% Create a ROS publisher for target joint states
[targetJointTrajPub, targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");

% Create a ROS Subscriber to the topic joint_states
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); 
pause(2);

%% Set Paramters and Target Position of images
% Target values are determined upon setup of environment(Dobot position,
% Pattern Position and required frame of picture from camera

% Calculate corner points using Harris Feature detector

img = readImage(rgbSub.LatestMessage);      %Gets latest image
pause(1);

% Set Target point, the required view of pattern within frame of camera 
% This would be the s* value
Target = [   656.1375  499.1717    %Top Left Corner(x,Y)
             665.9321  213.6515    %Bottom left corner
             928.1994  507.5360    %Top Right corner
              938.0593  223.7113   %Bottom right Corner
                    ];

f = 400;                    %focal Point
p = length(img)/2;          %Principle point, midway
Z = 0.15;                   %distance of camera from Picture (measured upon setup)
l = 0.1; %lambda            %rate



%% Create a loop that:
 % Read Current image
 % Converts to Gray, Then detects Corners using detectharrisFeatures
 % Stores Observed points
 % Determine the Lx value and then the error
 % Using Lx and Error, determine the Camera Velocity
 % Use Camera Velocity and current joint state to move dobot to required
 % area

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








