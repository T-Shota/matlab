%% Lidar�ɂ��n�ʂ̌��o�Ə�Q�����m

%% �|�C���g�N���E�h�f�[�^�̃��[�h
d = load('01_city_c2s_fcw_10s_Lidar.mat');
pcloud = d.LidarPointCloud;

%% �Ԃ̑O�����40m�ƍ��E20m�̗̈�ɒ���
pc = pcloud(1).ptCloud;

% ROS��ݒ�
xBound  = 40; % in meters
yBound  = 20; % in meters
xlimits = [-xBound, xBound];
ylimits = [-yBound, yBound];
zlimits = pc.ZLimits;

player = pcplayer(xlimits, ylimits, zlimits);

% �Ώۗ̈��؂�o�����߂̃C���f�b�N�X
indices = find(pc.Location(:, 2) >= -yBound ...
             & pc.Location(:, 2) <=  yBound ...
             & pc.Location(:, 1) >= -xBound ...
             & pc.Location(:, 1) <=  xBound);
             
% �\��
pc = select(pc, indices);
view(player, pc)

%% �n�ʂƏ�Q���𕪗�����
maxDistance = 0.2; % in meters
referenceVector = [0, 0, 1];
[~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);

%% �n�ʂ�΁A10m�ȓ��̏�Q�����
labelSize   = [pc.Count, 1];
colorLabels = zeros(labelSize, 'single');
    
% Set the colormap for labeling the points.
colors = [0 0 1; ...  % Blue for unlabeled points; specified as [R, G, B]
          0 1 0; ...  % Green for ground plane points
          1 0 0; ...  % Red for obstacle points
          0 0 0];     % Black for ego-vehicle points

blueIdx  = 0; % the entire point cloud is initially blue 
greenIdx = 1;
redIdx   = 2;
blackIdx = 3;

% Label the ground plane points.
colorLabels(inPlanePointIndices) = greenIdx;
 
% Select the points that are not part of the ground plane.
pcWithoutGround = select(pc, outliers);

%% 10m�ȓ��̏�Q���𒊏o
sensorLocation   = [0,0,0]; % place the Lidar sensor at the center of coordinate system
radius           = 10;      % in meters

nearIndices  = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);    
nearPointIndices = outliers(nearIndices);

% Label the obstacle points.
colorLabels(nearPointIndices) = redIdx;
    
%% ���Ԃ����O
radius      = 3; % in meters
nearIndices = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);

vehiclePointIndices = outliers(nearIndices);
pcVehicle           = select(pcWithoutGround, nearIndices);

%% ���Ԃ��͂�
delta = 0.1;
selfCube = [pcVehicle.XLimits(1)-delta, pcVehicle.XLimits(2)+delta ...
            pcVehicle.YLimits(1)-delta, pcVehicle.YLimits(2)+delta ...
            pcVehicle.ZLimits(1)-delta, pcVehicle.ZLimits(2)+delta];
 
colorLabels(vehiclePointIndices) = blackIdx;

colormap(player.Axes, colors)
view(player, pc.Location, colorLabels);
title(player.Axes, 'Segmented Point Cloud');

%% �_�Q�V�[�P���X
for k = 2:length(pcloud)
    pc = pcloud(k).ptCloud;    

    % ROI�𒊏o
    indices = find(pc.Location(:, 2) >= -yBound ...
                 & pc.Location(:, 2) <=  yBound ...
                 & pc.Location(:, 1) >= -xBound ...    
                 & pc.Location(:, 1) <=  xBound);
    pc = select(pc, indices);
    
    colorLabels = zeros(pc.Count, 1, 'single'); % create label array
    
    % �n�ʂ̌��o
    [~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);    
    colorLabels(inPlanePointIndices) = greenIdx;

    pcWithoutGround = select(pc, outliers);
    
    % ���a10m�ȓ��̏�Q�������m
    radius           = 10; % in meters
    nearIndices      = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);    
    nearPointIndices = outliers(nearIndices);
    
    colorLabels(nearPointIndices) = redIdx;

    % ���Ԃ̓_�Q�𒊏o
    nearIndices         = findPointsInROI(pcWithoutGround, selfCube);
    vehiclePointIndices = outliers(nearIndices);
    
    colorLabels(vehiclePointIndices) = blackIdx;
    
    % �v���b�g
    view(player, pc.Location, colorLabels);
end
