clear; clc;
rosshutdown;
rosinit;

global obj;
obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);

node = robotics.ros.Node('test');
sub = robotics.ros.Subscriber(node, '/image_raw', 'sensor_msgs/Image', @image_callback);

function image_callback(~, msg)
	global obj;

	detector = peopleDetectorACF('caltech');
	resizeRatio = 1.0;
	I = readImage(msg);
	I = imresize(I, resizeRatio, 'Antialiasing', false);
    [bboxes, scores] = detect(detector, I);

    if ~isempty(scores)
    	I = insertObjectAnnotation(I, 'rectangle', bboxes, scores);
    end

     step(obj.videoPlayer, I);
end