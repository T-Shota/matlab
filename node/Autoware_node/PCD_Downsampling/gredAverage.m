%% Initialization
clear; clc;
rosshutdown;
rosinit;

obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);

%% Define node
node = robotics.ros.Node('grid_downsampling_matlab');
sub = robotics.ros.Subscriber(node, '/points_raw', 'sensor_msgs/PointCloud2', @grid_downsampling_callback);

function grid_downsampling_callback(~, msg)
	ptCloud = pointCloud(readXYZ(msg));

	gridStep = 0.1;
	filtered_ptCloud = pcdownsample(ptCloud,'gridAverage',gridStep);

    pcshow(filtered_ptCloud);
end