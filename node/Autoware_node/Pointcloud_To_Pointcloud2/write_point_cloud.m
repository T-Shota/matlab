%% Load pointCloud data
dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
load(dataFile);
ptcloud = livingRoomData{10};



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


%% create a subscriber
plane_sub = rossubscriber('/plane_detection_output','sensor_msgs/PointCloud2');

%% publish the generated message
send(plane_pub,ptMsg);

%% you can also subscribe the published plane message
pause(1)
plane_msg_new = plane_sub.LatestMessage;
plane_msg_new.PreserveStructureOnRead = true;
ptCloud2 = pointCloud(readXYZ(plane_msg_new),'Color',readRGB(plane_msg_new));
figure,pcshow(ptCloud2,'VerticalAxis','Y','VerticalAxisDir','Down');
title('Subscribed point cloud');

% Copyright 2018 The MathWorks, Inc.