%% Initialization
clear; clc;
rosshutdown;
rosinit;

obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);

%% Define node
node = robotics.ros.Node('grid_downsampling_matlab');
sub = robotics.ros.Subscriber(node, '/points_raw', 'sensor_msgs/PointCloud2', @nonuniformgrid_downsampling_callback);

function nonuniformgrid_downsampling_callback(~, msg)
	ptCloud = pointCloud(readXYZ(msg));

	% must set maxNumPoints >= 6
	maxNumPoints = 10;
	filtered_ptCloud = pcdownsample(ptCloud, 'nonuniformGridSample', maxNumPoints);

    pcshow(filtered_ptCloud);
end