%%Drawing data from the camera

%%CameraReading = rossubscriber("/camera/color/image_raw/compressed");

rgbSub = rossubscriber("/camera/color/image_raw/compressed");
pause(1);
image_h = imshow(readImage(rgbSub.LatestMessage));   %take a pic

%%Make the figure large
set(gcf,'units','normalized','outerposition',[0 0 1 1]);








%%if you want to take a video
%%Now loop through and update the image data with the latest image data (ctrl + c to stop)

tic
while 1
image_h.CData = readImage(rgbSub.LatestMessage);
drawnow;
toc;
end