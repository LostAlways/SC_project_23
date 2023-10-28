% cla

% This code is to get the desired corner points. We will need to then put
% the matrix in our main vis servo code in the 'target' matrix

rgbSub = rossubscriber('/camera/color/image_raw');
image = readImage(rgbSub.LatestMessage);
l = rgb2gray(image);
cornerPoints = detectHarrisFeatures(l);
disp(cornerPoints.location)
imshow(l)
hold on
plot(cornerPoints)