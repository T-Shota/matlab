%% Lidarによる地面の検出と障害物検知

%% ポイントクラウドデータのロード
d = load('01_city_c2s_fcw_10s_Lidar.mat');
pcloud = d.LidarPointCloud;

%% 車の前方後方40mと左右20mの領域に注目
pc = pcloud(1).ptCloud;

% ROSを設定
xBound  = 40; % in meters
yBound  = 20; % in meters
xlimits = [-xBound, xBound];
ylimits = [-yBound, yBound];
zlimits = pc.ZLimits;

player = pcplayer(xlimits, ylimits, zlimits);

% 対象領域を切り出すためのインデックス
indices = find(pc.Location(:, 2) >= -yBound ...
             & pc.Location(:, 2) <=  yBound ...
             & pc.Location(:, 1) >= -xBound ...
             & pc.Location(:, 1) <=  xBound);
             
% 表示
pc = select(pc, indices);
view(player, pc)

%% 地面と障害物を分離する
maxDistance = 0.2; % in meters
referenceVector = [0, 0, 1];
[~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);

%% 地面を緑、10m以内の障害物を赤
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

%% 10m以内の障害物を抽出
sensorLocation   = [0,0,0]; % place the Lidar sensor at the center of coordinate system
radius           = 10;      % in meters

nearIndices  = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);    
nearPointIndices = outliers(nearIndices);

% Label the obstacle points.
colorLabels(nearPointIndices) = redIdx;
    
%% 自車を除外
radius      = 3; % in meters
nearIndices = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);

vehiclePointIndices = outliers(nearIndices);
pcVehicle           = select(pcWithoutGround, nearIndices);

%% 自車を囲む
delta = 0.1;
selfCube = [pcVehicle.XLimits(1)-delta, pcVehicle.XLimits(2)+delta ...
            pcVehicle.YLimits(1)-delta, pcVehicle.YLimits(2)+delta ...
            pcVehicle.ZLimits(1)-delta, pcVehicle.ZLimits(2)+delta];
 
colorLabels(vehiclePointIndices) = blackIdx;

colormap(player.Axes, colors)
view(player, pc.Location, colorLabels);
title(player.Axes, 'Segmented Point Cloud');

%% 点群シーケンス
for k = 2:length(pcloud)
    pc = pcloud(k).ptCloud;    

    % ROIを抽出
    indices = find(pc.Location(:, 2) >= -yBound ...
                 & pc.Location(:, 2) <=  yBound ...
                 & pc.Location(:, 1) >= -xBound ...    
                 & pc.Location(:, 1) <=  xBound);
    pc = select(pc, indices);
    
    colorLabels = zeros(pc.Count, 1, 'single'); % create label array
    
    % 地面の検出
    [~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);    
    colorLabels(inPlanePointIndices) = greenIdx;

    pcWithoutGround = select(pc, outliers);
    
    % 半径10m以内の障害物を検知
    radius           = 10; % in meters
    nearIndices      = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);    
    nearPointIndices = outliers(nearIndices);
    
    colorLabels(nearPointIndices) = redIdx;

    % 自車の点群を抽出
    nearIndices         = findPointsInROI(pcWithoutGround, selfCube);
    vehiclePointIndices = outliers(nearIndices);
    
    colorLabels(vehiclePointIndices) = blackIdx;
    
    % プロット
    view(player, pc.Location, colorLabels);
end
