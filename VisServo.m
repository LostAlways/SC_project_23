 % Code is incomplete for now
                Target = [  446,946;
                            446,446;
                            946,946;
                            946,446      % This is the s*                           
                    ];

                f = 400;
                p = length(img)/2;
                Z = 50;
                l = 0.05; %lambda 

                [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
                trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");

                rgbSub = rossubscriber('/camera/color/image_raw');
                
                % MINE: Lambda should be changed to a lower number, this will ensure
                % accuracy in visual servoing
                
                % Populate Observation vector using values from "feature detection section"
                % Populate target vector using image size (define your own target size for
                % the square)

   %% Continuous while loop

            while 1 == 1
                

                % Calculate corner points using Harris Feature detector
                img = readImage(rgbSub.LatestMessage);

                cornerPoints = detectHarrisFeatures(img);
                
                cornerPoints.Location

                Obs = [];                
                Obs = cornerPoints.location;
                
                xy = (Target-p)/f;
                Obsxy = (Obs-p)/f;
                
                % MINE: What this code does is since the image assumes that the origin is
                % in the top left hand corner. We are using the principal point to make the
                % origin the center of the image 
                
                n = length(Target(:,1));
                
                Lx = [];
                for i=1:n
                    Lxi = FuncLx(xy(i,1),xy(i,2),Z);
                    Lx = [Lx;Lxi];
                end
                
                
                e2 = Obsxy-xy;
                e = reshape(e2',[],1);
                de = -e*l;
                
                
                Lx2 = inv(Lx'*Lx)*Lx';  % This part is the pseudo inverse of the lx matrix
                Vc = -l*Lx2*e;

                disp(Vc);

                % I created the below

                opt_Vc = Vc * 0.1;

                jointTarget = [0 + opt_Vc(1),0.4 + opt_Vc(2),0.3 + opt_Vc(3),0]; % Remember that the Dobot has 4 joints by default.

                
                trajectoryPoint.Positions = jointTarget;
                targetJointTrajMsg.Points = trajectoryPoint;
                
                send(targetJointTrajPub,targetJointTrajMsg);

               
            
            end 



 





%% Lx function


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
