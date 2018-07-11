%% Initialization
clear; clc;
rosshutdown;
rosinit;

%% Define node
node = robotics.ros.Node('grid_downsampling_matlab');
sub = robotics.ros.Subscriber(node, '/points_raw', 'sensor_msgs/PointCloud2', @grid_downsampling_callback);

function grid_downsampling_callback(~, msg)
	ptCloud = pointCloud(readXYZ(msg), 'intensity', readField(msg, 'intensity'));

	% gridStep is voxelsize [m]
	gridStep = 2.0;
	filtered_ptCloud = pcdownsample(ptCloud, 'gridAverage', gridStep);

    ptMsg = ConvertPointcloudToPointcloud2(filtered_ptCloud);

    plane_pub = rospublisher('/filterd_points','sensor_msgs/PointCloud2');
    send(plane_pub, ptMsg);

% PointCloud to PointCloud2
    function ptMsg = ConvertPointcloudToPointcloud2(ptcloud)
        %% create a publisher
        plane_pub = rospublisher('/filtered_points','sensor_msgs/PointCloud2');

        %% create "sensor_msgs/PointCloud2" message
        ptMsg = rosmessage('sensor_msgs/PointCloud2');
        ptMsg.Header = rosmessage('std_msgs/Header');
        ptMsg.Height = size(ptcloud.Location,1);
        ptMsg.Width = size(ptcloud.Location,2);
        ptMsg.PointStep = 32;
        ptMsg.RowStep = ptMsg.Width * ptMsg.PointStep;
        for k = 1:4
            ptMsg.Fields(k) = rosmessage('sensor_msgs/PointField');
        end
        ptMsg.Fields(1).Name = 'x';
        ptMsg.Fields(1).Offset = 0;
        ptMsg.Fields(1).Datatype = 7;
        ptMsg.Fields(1).Count = 1;
        ptMsg.Fields(2).Name = 'y';
        ptMsg.Fields(2).Offset = 4;
        ptMsg.Fields(2).Datatype = 7;
        ptMsg.Fields(2).Count = 1;
        ptMsg.Fields(3).Name = 'z';
        ptMsg.Fields(3).Offset = 8;
        ptMsg.Fields(3).Datatype = 7;
        ptMsg.Fields(3).Count = 1;
        ptMsg.Fields(4).Name = 'intensity';
        ptMsg.Fields(4).Offset = 16;
        ptMsg.Fields(4).Datatype = 7;
        ptMsg.Fields(4).Count = 1;

        locs = reshape(ptcloud.Location,[],3);
        colors = uint32(reshape(ptcloud.Intensity,[],3));
        rgb = typecast(colors(:,1)*2^16+colors(:,2)*2^8+colors(:,3),'single');
        data = [locs zeros(size(locs,1),1,'single') rgb zeros(size(locs,1),1,'single')];
        dataPacked = reshape(permute(data,[2 1]),[],1);
        dataCasted = typecast(dataPacked,'uint8');
        ptMsg.Data = dataCasted;

    end
end