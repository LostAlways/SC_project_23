
%% Creating a loop to gather all the images

dobot_pose1 = [-0.3008 -0.0024 0.1663 0]; % Remember that the Dobot has 4 joints by default. 
dobot_pose2 = [-0.1982 0.0011 0.1692 0];
dobot_pose3 = [-0.333 0.0012 0.1284 0];
dobot_pose4 = [0.0670 -0.0135 0.1283 0];
dobot_pose5 = [0.0383 0.0103 0.3679 0];
dobot_pose6 = [0.0388 0.1255 0.4170 0];
dobot_pose7 = [-0.1635 0.1210 0.4418 0];
dobot_pose8 = [0.2014 0.1245 0.5182 0];
dobot_pose9 = [0.1894 0.1351 0.2520 0];
dobot_pose10 = [-0.0688 0.1119 0.3018 0];

% bag = rosbagwriter('Dobot_pose_images.bag'); %Create a rosbag file

% Define an array of dobot poses
dobot_poses = [dobot_pose1; dobot_pose2; dobot_pose3; dobot_pose4;
               dobot_pose5; dobot_pose6; dobot_pose7; dobot_pose8; dobot_pose9; dobot_pose10];

% Create a ROS publisher for target joint states
[targetJointTrajPub, targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');


% Create a ROS subscriber for the camera image
rgbSub = rossubscriber('/camera/color/image_raw');
% Create a ROS subscriber for end effector poses
endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses');

imagePub = rospublisher('/camera/color/image_raw', 'sensor_msgs/Image');

% Initialize an array to store the captured images
num_images = length(dobot_poses);
captured_images = cell(1, num_images);   % Creates a cell array with one row and num_images of columns

% Initialize a cell array to store the end effector transformation matrices
T_EE = cell(1, num_images);

output_directory = 'home/david/Desktop/DobotImg';




% Loop through the dobot poses      
for i = 1:num_images
    % Set the trajectory point positions
    trajectoryPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    trajectoryPoint.Positions = dobot_poses(i, :);  %all the elements in the 'ith row
    targetJointTrajMsg.Points = trajectoryPoint;

    % Send the joint states
    send(targetJointTrajPub, targetJointTrajMsg);

    % Capture an image
    image = readImage(rgbSub.LatestMessage);

    % Store the captured image in the corresponding variable
    captured_images{i} = image;


    % Obtain the end effector pose
    currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
    currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X;
                                  currentEndEffectorPoseMsg.Pose.Position.Y;
                                  currentEndEffectorPoseMsg.Pose.Position.Z];
    currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W;
                             currentEndEffectorPoseMsg.Pose.Orientation.X;
                             currentEndEffectorPoseMsg.Pose.Orientation.Y;
                             currentEndEffectorPoseMsg.Pose.Orientation.Z];
    currentEndEffectorQuat = currentEndEffectorQuat.';
%    [roll, pitch, yaw] = quat2eul(currentEndEffectorQuat(1:3));
    %euler_angles = quat2eul(currentEndEffectorQuat);

    % Create a 4x4 transformation matrix for the end effector pose
    T_EE{i} = eye(4);
    T_EE{i}(1:3, 1:3) = quat2rotm(currentEndEffectorQuat); % rows 1:3, and coloumns 1:3
    T_EE{i}(1:3, 4) = currentEndEffectorPosition';
           

    % Display the current pose and image
    pause(1);
    disp(['Dobot Pose ', num2str(i)]);
    imshow(image);

    % Pause to allow time for capturing and processing
    pause(3); % Adjust the pause duration as needed
end






%% just notes
% You now have captured_images{1}, captured_images{2}, ..., captured_images{9} containing the captured images.

% % Specify the directory where you want to save the images
% output_directory = '/path/to/your/directory/';
% 
% % Loop through the dobot poses
% for i = 1:num_images
%    % Specify the directory where you want to save the images
% output_directory = '/path/to/your/directory/';
% 
% % Loop through the dobot poses
% for i = 1:num_images
%     % ... (previous code)
% 
%     % Save the captured image to the directory
%     image_filename = fullfile(output_directory, ['image', num2str(i), '.png']);
%     imwrite(image, image_filename);
% 
%     % ... (rest of the loop)
% end % ... (previous code)
% 
%     % Save the captured image to the directory
%     image_filename = fullfile(output_directory, ['image', num2str(i), '.png']);
%     imwrite(image, image_filename);
% 
%     % ... (rest of the loop)
% end


    
    %image_filename = fullfile(output_directory, ['image', num2str(i), '.jpg']);
    %imwrite(image, image_filename);
    
    % Convert the captured image to a ROS message

    % imgMsg = rosmessage('sensor_msgs/Image');
    % 
    % imgMsg.Data = reshape(image, 1, []); % Convert the image to a vector
    % imgMsg.Width = size(image, 2); % Set the width of the image
    % imgMsg.Height = size(image, 1); % Set the height of the image
    % imgMsg.Encoding = 'rgb8'; % Set the image encoding (adjust as needed)
    % 
    % % Publish the image message to the bag
    % send(imagePub, imgMsg);









