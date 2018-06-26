%% Initialization
clear; clc;
rosshutdown;
rosinit;

global obj;
obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);

%% Define node
node = robotics.ros.Node('grid_downsampling_matlab');
sub = robotics.ros.Subscriber(node, '/points_raw', 'sensor_msgs/PointCloud2', @grid_downsampling_callback);

function grid_downsampling_callback(~, msg)
	global obj;
	ptCloud = readXYZ(msg);

	gridStep = 2;
	filtered_ptCloud = pcdownsample(ptCloud,'gridAverage',gridStep);

	step(obj.videoPlayer, filtered_ptCloud);

end