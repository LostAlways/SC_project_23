%% joint x 9

jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
pause(2); % Allow some time for a message to appear
currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message

% %      0
%     0.7850
%     0.7846
%          0

%pos1
% -0.7027
%     0.5638
%     0.1894
%          0

%Pos2
% -0.6029
%     0.5427
%     0.1103
%          0


%Pos2
% -0.7027
%     0.5638
%     0.1894
%          0

% %Pos2
% 0.5022
%     0.5129
%    -0.0055
%          0
% 
% 
% %Pos2
% -0.2749
%     0.5038
%    -0.0393
%          0
% 
% 
% 
% %Pos2
%   0.0762
%     0.5038
%    -0.0415
%          0
% 
% %Pos2
%   0.2934
%     0.3789
%     0.0703
%          0
% 
% %Pos2
%   0.4833
%     0.3759
%     0.3300
%          0


%Pos2
   % 0.5331
   %  0.4166
   %  0.5932
   %       0

%Pos2
 % 0.4204
 %    0.5574
 %    0.8692
 %         0


%Pos2
  % -0.0111
  %   0.5320
  %   0.9450
  %        0


