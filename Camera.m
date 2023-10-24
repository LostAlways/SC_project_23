<<<<<<< HEAD
%%Drawing data from the camera

%%CameraReading = rossubscriber("/camera/color/image_raw/compressed");
=======
% //Drawing data from the camera

% //CameraReading = rossubscriber("/camera/color/image_raw/compressed");
>>>>>>> 53cb872ca5da2370244de65350e1bbcebe7185c3

rgbSub = rossubscriber("/camera/color/image_raw/compressed");
pause(1);
image_h = imshow(readImage(rgbSub.LatestMessage));   %take a pic

<<<<<<< HEAD
%%Make the figure large
set(gcf,'units','normalized','outerposition',[0 0 1 1]);








%%if you want to take a video
%%Now loop through and update the image data with the latest image data (ctrl + c to stop)
=======
% //Make the figure large
set(gcf,'units','normalized','outerposition',[0 0 1 1]);


% // if you want to take a video
% //Now loop through and update the image data with the latest image data (ctrl + c to stop)
>>>>>>> 53cb872ca5da2370244de65350e1bbcebe7185c3

tic
while 1
image_h.CData = readImage(rgbSub.LatestMessage);
drawnow;
toc;
end