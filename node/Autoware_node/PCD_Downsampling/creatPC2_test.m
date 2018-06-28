%% Initialization
clear; clc;
rosshutdown;
rosinit;

obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);

%% Define node
node = robotics.ros.Node('grid_downsampling_matlab');
sub = robotics.ros.Subscriber(node, '/points_raw', 'sensor_msgs/PointCloud2', @grid_downsampling_callback);

function grid_downsampling_callback(~, msg)
	ptCloud = pointCloud(readXYZ(msg), 'intensity', readField(msg, 'intensity'));

	% gridStep is voxelsize [m]
	gridStep = 2.0;
	filtered_ptCloud = pcdownsample(ptCloud, 'gridAverage', gridStep);

    ptCloud2 = creatPointCloud2msg(filtered_ptCloud)

    plane_pub = rospublisher('/filterd_points','sensor_msgs/PointCloud2');
    send(plane_pub, ptCloud2);

    % PointCloud to PointCloud2
    function ptCloud2 = creatPointCloud2msg(ptCloud)
    	ptCloud2 = rosmessage('sensor_msgs/PointCloud2')
    	data = [ptCloud.Location ptCloud.Color]
    	dataCasted = typecast(data, 'uint8')
    	ptCloud2.Data = dataCasted
    end
end