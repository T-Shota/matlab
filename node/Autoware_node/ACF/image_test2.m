clear; clc;
rosshutdown;
rosinit;

global obj;

obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);

node = robotics.ros.Node('test1');
sub = robotics.ros.Subscriber(node, '/image_raw', 'sensor_msgs/Image', @image_callback);

function image_callback(~, msg)
	global obj;
    latestImg = readImage(msg);

    step(obj.videoPlayer, latestImg);
%    imshow(latestImg);
end