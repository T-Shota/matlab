clear; clc;
rosshutdown;
rosinit;

global obj;
global detector;
global resizeRatio;

obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);
detector = peopleDetectorACF('inria');
resizeRatio = 1.0;

node = robotics.ros.Node('ACF_detection_matlab');
sub = robotics.ros.Subscriber(node, '/image_raw', 'sensor_msgs/Image', @ACF_detection_callback);

function ACF_detection_callback(~, msg)
	global obj;
	global detector;
	global resizeRatio;

	I = readImage(msg);
	I = imresize(I, resizeRatio, 'Antialiasing', false);
    [bboxes, scores] = detect(detector, I);

    if ~isempty(scores)
    	I = insertObjectAnnotation(I, 'rectangle', bboxes, scores);
    end

     step(obj.videoPlayer, I);
end