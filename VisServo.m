 % Code is incomplete for now

classdef VisServo
 

properties

end

methods

    function image = VisServo()
        [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

        %Get feedback about where the end effector pose is currently
        endEffectorSub = rossubscriber('/dobot_magician/current_end_effector_pose');  

        check_constantly();

    end 

    function check_constantly(Vc)
            while 1 == 1
                endEffectorRotation = [Vc(4), Vc(5), Vc(6)];
                    
                    targetEndEffectorMsg.Position.X = Vc(1); 
                    targetEndEffectorMsg.Position.Y = Vc(2);
                    targetEndEffectorMsg.Position.Z = Vc(3);
                    
                    qua = eul2quat(endEffectorRotation);
            
                    targetEndEffectorMsg.Orientation.W = qua(1);
                    targetEndEffectorMsg.Orientation.X = qua(2);
                    targetEndEffectorMsg.Orientation.Y = qua(3);
                    targetEndEffectorMsg.Orientation.Z = qua(4);
            
            
                    send(targetEndEffectorPub,targetEndEffectorMsg);
            
            end 
    end

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

%% Feature detection

function Vc = velFunc(Lx)

img = imread('image_2.png');

% Calculate corner points using Harris Feature detector
cp = detectHarrisFeatures(img);

cp.Location
disp("howdy")



%% Control

f = 400;
p = length(img)/2;
Z = 50;
l = 0.1; %lambda 

% MINE: Lambda should be changed to a lower number, this will ensure
% accuracy in visual servoing

% Populate Observation vector using values from "feature detection section"
% Populate target vector using image size (define your own target size for
% the square)

%% image_1.png
% Target = [250,250;
%           250,750;
%           750,250;
%           750,750];
%       
% Obs = [ 100.4890,  100.4890;
%         100.4890,  598.5110;
%         598.5110,  100.4890;
%         598.5110,  598.5110;];

%% image_2.png
Target = [  446,946;
            446,446;
            946,946;
            946,446      % This is the s*
            
    ];

Obs = [313.5,    547.7;
    599.0,    140.2;
    720.9,    833.3;
    1006.4 ,   425.7;];   % This is the s

Obs = cp.location;


xy = (Target-p)/f;
Obsxy = (Obs-p)/f;

% MINE: What this code does is since the image assumes that the origin is
% in the top left hand corner. We are using the principal point to make the
% origin the center of the image 

n = length(Target(:,1));

Lx = [];
for i=1:n;
    Lxi = FuncLx(xy(i,1),xy(i,2),Z);
    Lx = [Lx;Lxi];
end


e2 = Obsxy-xy;
e = reshape(e2',[],1);
de = -e*l;


Lx2 = inv(Lx'*Lx)*Lx';  % This part is the pseudo inverse of the lx matrix
Vc = -l*Lx2*e

end
end
 end