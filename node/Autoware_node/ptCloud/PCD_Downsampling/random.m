%% Initialization
clear; clc;
rosshutdown;
rosinit;

obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);

%% Define node
node = robotics.ros.Node('grid_downsampling_matlab');
sub = robotics.ros.Subscriber(node, '/points_raw', 'sensor_msgs/PointCloud2', @random_downsampling_callback);

function random_downsampling_callback(~, msg)
	ptCloud = pointCloud(readXYZ(msg));

	% must set percentage in [0, 1]
	percentage = 0.5;
	filtered_ptCloud = pcdownsample(ptCloud, 'random', percentage);

    pcshow(filtered_ptCloud);

end