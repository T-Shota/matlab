%% Monocular Camera Sensor Design：単眼カメラモジュールの動作モデル
% 白線と車両の検出・ワールド座標で結果出力
% 自車と障害物までの距離の推定
% Copyright 2016 The MathWorks, Inc. 
clc;clear;close all;imtool close all;

%% カメラ内部と配置のパラメータを設定（monoCameraクラス：imageToWorld, worldToImage メソッド(点座標の変換)を提供）
% カメラキャリブレーションで求めたカメラ内部パラメータを設定（専用クラス）
%    （レンズ歪が少ないため、ここではレンズ歪補正はなし）
focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] 光学中心 in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);   % 内部パラメータのクラス

% 車両に対するカメラの配置を指定（外部パラメータからの算出も可能）
height = 2.1798;    % 地面からの高さ(ここではmを単位に使用)
pitch  = 14;        % カメラのピッチ(水平からの角度：単位は度) ここではRollとYawはゼロ

% 上記パラメータから、monoCameraオブジェクトを定義
%     車両座標：自車に対して固定。原点はカメラの焦点の真下の地面。X：前方が正、Y：左が正
%     平らな地面を仮定
% monoCameraオブジェクト：カメラ内部・外部パラメータを格納、画像の座標と車両座標を相互変換する関数を提供
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch)

%% 動画ファイル確認
winopen('caltech_cordova1.avi');

%% 動画ファイルから画像を一枚読込み
videoReader = VideoReader('caltech_cordova1.avi');
videoReader.CurrentTime = 0.06667;        % 読込むフレームのタイムスタンプを指定
frame = readFrame(videoReader);           % 1フレーム読込む
figure; imshow(frame)                     % 表示

%% 鳥瞰図への変換（birdsEyeViewクラス）
%    以降の解析の簡単化のため、先ず鳥瞰図へ変換（線の太さ一定、車線が平行になる）
% 出力画像の領域を指定（monoCameraオブジェクト生成時のカメラ高さの距離単位を使用）
%web(fullfile(docroot, 'driving/ref/birdseyeview-class.html#input_argument_d0e22799')); % 定義

% birdsEyeView クラスの生成・変換
bottomOffset      = 3;   % 手前3m先から
distAheadOfSensor = 30;  % 遠方30m先までの鳥瞰図
spaceToOneSide    = 6;   % 中心から左右6mずつの鳥瞰図
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax] 車両座標系
imageSize = [NaN, 250];      % 変換後の鳥瞰図の画像ピクセルサイズ（幅250pix、縦は自動計算）

% 鳥瞰図生成用クラス（monoCameraクラスから生成）
birdsEyeConfig  = birdsEyeView(sensor, outView, imageSize);

% 鳥瞰図へ変換
birdsEyeImage = transformImage(birdsEyeConfig,frame);

figure; imshow(birdsEyeImage);

%% 線のペイント領域の検出（車両座標、本数は任意）
%    車両座標を使用するので、下記はカメラ等に依存しない
birdsEyeImage = rgb2gray(birdsEyeImage);    % 鳥瞰図をグレースケールへ変換
vehicleROI    = outView - [-1, 2, -3, 3];   % [xmin, xmax, ymin, ymax] 検出領域を少し内側に指定
approxLaneMarkerWidthVehicle = 0.25;          % 白線の幅（25cm）

% Detect lane features：ここでは例としてシンプルなものを使用（ワールド座標ベースで計算）
%   左右よりも明るい、～25cm幅の領域
%   M. Nieto, J. A. Laborda, L. Salgado, "Road enviornment modeling using robust perspective analysis and recursive Bayesian segmentation," Machine Vision and Applications, Volume 22, Issue 6, pp.927-945, 2011.
laneSensitivity = 0.25;
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, approxLaneMarkerWidthVehicle, ...
                       'ROI', vehicleROI, 'Sensitivity', laneSensitivity);  % M言語で実装
figure; imshow(birdsEyeViewBW);

%% 自車線に沿った方向の区画線の検出：ax^2 + bx + c へカーブフィッティング（ここでは、最大2本）
[imageX, imageY] = find(birdsEyeViewBW);        % 全ての前景ピクセルの、X座標・Y座標
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);  % 車両座標へ変換

% RANdom SAmple Consensus (RANSAC)を用いて、ノイズがある中で安定してカーブフィッティング
% parabolicLaneBoundary オブジェクトのベクトルを返す
boundaryWidth = 3*approxLaneMarkerWidthVehicle;  % 二重線の場合も一本の線として検出するために幅を3倍へ
maxLanes      = 2;                               % 最大2本まで検出 => "可変"

% 2次関数で補間
[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints, boundaryWidth, ...
    'MaxNumBoundaries',maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn_J);

% 結果の確認：y = ax^2 + bx + c の係数
boundaries(:).Parameters

%% 自車線の区画線の決定
% 前処理：Heuristicsにより、横断歩道の白線等の除去

% 短い白線を除去（横断歩道）
maxPossibleXLength = diff(vehicleROI(1:2));      % 車両座標ROIの縦の長さ
minXLength         = maxPossibleXLength * 0.60;  % ROIの縦長の60％(minXLength)以上のもののみ選択
isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
boundaries    = boundaries(isOfMinLength);

% かすれ等の多いものを除去
% Remove additional boundaries based on the strength metric computed by the
% <matlab:doc('findParabolicLaneBoundaries'); |findParabolicLaneBoundaries|> function. Set a lane strength threshold
% based on ROI and image size.
birdsImageROI = vehicleToImageROI_J(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

maxPointsInOneLane = numel(unique(vehiclePoints(:,1)));  %  Find the maximum number of unique x-axis locations possible for any lane boundary
maxLaneLength = diff(vehicleROI(1:2));            % 車両座標ROIの縦の長さ
maxStrength   = maxPointsInOneLane/maxLaneLength; % maximum possible lane strength for this image size/ROI size specification
% Reject weak boundaries
isStrong      = [boundaries.Strength] > 0.4*maxStrength;
boundaries    = boundaries(isStrong);

%% 白線の種類の分類（実線/点線）と、自車線の左右の線の認識
boundaries = classifyLaneTypes_J(boundaries, boundaryPoints);

% Locate two ego-lanes if they are present.
xOffset    = 0;   %  0 meters from the sensor
% parabolicBoundary クラスのメソッドを使用して、xOffsetに対応するY座標を計算
distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
% Find candidate ego boundaries
leftEgoBoundaryIndex  = [];
rightEgoBoundaryIndex = [];
minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
if ~isempty(minLDistance)
    leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
end
if ~isempty(minRDistance)
    rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
end
leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);
% 線の種類の判別結果の確認
leftEgoBoundary.BoundaryType
rightEgoBoundary.BoundaryType

%% 検出した白線を、カメラ画像と、鳥瞰図上へ表示
xVehiclePoints = bottomOffset:distAheadOfSensor;
%     第3引数が birdsEyeView オブジェクトの場合、鳥瞰図への挿入
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeWithEgoLane, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
%     第3引数が monoCamera オブジェクトの場合、元の画像への挿入
frameWithEgoLane = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
frameWithEgoLane = insertLaneBoundary(frameWithEgoLane, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

figure
subplot('Position', [0, 0, 0.5, 1.0]) % [left, bottom, width, height] in normalized units
imshow(birdsEyeWithEgoLane);     % 鳥瞰図（右）
subplot('Position', [0.5, 0, 0.5, 1.0])
imshow(frameWithEgoLane)         % カメラ画像（左）

%% [前方車両検出] 車両座標系
% Aggregate Channel Features (ACF) を用いた車両検出器
% （ここではトラッキングは未使用）
detector = vehicleDetectorACF();
%detector = vehicleDetectorFasterRCNN();
vehicleWidth = [1.5, 2.5];        % 検出する車両幅：1.5-2.5m

% 単眼カメラ検出器オブジェクト（路面にある、指定した幅の物体のみ検出）
monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);
[bboxes, scores] = detect(monoDetector, frame)    % 車両検出の実行

%% 前方車両までの距離を計算・可視化
locations = computeVehicleLocations_J(bboxes, sensor);  % 境界ボックス下線の中心までの距離を計算
imgOut = insertVehicleDetections_J(frameWithEgoLane, locations, bboxes);
figure; imshow(imgOut);

%% 動画に対してループ処理を行う
% Simulate a Complete Sensor with Video Input
% Now that you have an idea about the inner workings of the individual
% steps, let's put them together and apply them to a video sequence where
% we can also take advantage of temporal information.
%
% Rewind the video to the beginning, and then process the video. The code
% below is shortened because all the key parameters were defined in
% the previous steps. Here, the parameters are used without further
% explanation.
videoReader.CurrentTime = 0;

isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
   
    % Grab a frame of video
    frame = readFrame(videoReader);
    
    % Compute birdsEyeView image
    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = rgb2gray(birdsEyeImage);
    
    % Detect lane boundary features
    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, ...
        approxLaneMarkerWidthVehicle, 'ROI', vehicleROI, ...
        'Sensitivity', laneSensitivity);

    % Obtain lane candidate points in vehicle coordinates
    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);

    % Find lane boundary candidates
    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn_J);
    
    % Reject boundaries based on their length and strength    
    isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
    boundaries    = boundaries(isOfMinLength);
    isStrong      = [boundaries.Strength] > 0.2*maxStrength;
    boundaries    = boundaries(isStrong);
                
    % Classify lane marker type
    boundaries = classifyLaneTypes_J(boundaries, boundaryPoints);        
        
    % Find ego lanes
    xOffset    = 0;   %  0 meters from the sensor
    distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
    % Find candidate ego boundaries
    leftEgoBoundaryIndex  = [];
    rightEgoBoundaryIndex = [];
    minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
    minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
    if ~isempty(minLDistance)
        leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
    end
    if ~isempty(minRDistance)
        rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
    end
    leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
    rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);
    
    % Detect vehicles
    [bboxes, scores] = detect(monoDetector, frame);
    locations = computeVehicleLocations_J(bboxes, sensor);
    
    % Visualize sensor outputs and intermediate results. Pack the core
    % sensor outputs into a struct.
    sensorOut.leftEgoBoundary  = leftEgoBoundary;
    sensorOut.rightEgoBoundary = rightEgoBoundary;
    sensorOut.vehicleLocations = locations;
    
    sensorOut.xVehiclePoints   = bottomOffset:distAheadOfSensor;
    sensorOut.vehicleBoxes     = bboxes;
    
    % Pack additional visualization data, including intermediate results
    intOut.birdsEyeImage   = birdsEyeImage;    
    intOut.birdsEyeConfig  = birdsEyeConfig;
    intOut.vehicleScores   = scores;
    intOut.vehicleROI      = vehicleROI;
    intOut.birdsEyeBW      = birdsEyeViewBW;
    
    closePlayers = ~hasFrame(videoReader);
    isPlayerOpen = visualizeSensorResults_J(frame, sensor, sensorOut, ...
        intOut, closePlayers);
    
    timeStamp = 7.5333; % take snapshot for publishing at timeStamp seconds
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot_J(frame, sensor, sensorOut);
    end    
end

%%
% Display the video frame. Snapshot is taken at |timeStamp| seconds.
if ~isempty(snapshot)
    figure; imshow(snapshot); shg;
end

%% 異なる動画に対して処理
% The <matlab:edit('helperMonoSensor'); |helperMonoSensor|> class assembles the
% setup and all the necessary steps to simulate the monocular camera sensor
% into a complete package that can be applied to any video. Since most 
% parameters used by the sensor design are based on world units, the
% design is robust to changes in camera parameters, including the image
% size. Note that the code inside the |helperMonoSensor| class is different
% from the loop in the previous section, which was used to illustrate basic concepts.
%
% Besides providing a new video, you must supply a camera configuration
% corresponding to that video.  The process is shown here.  Try it
% on your own videos.

% Sensor configuration
focalLength    = [309.4362, 344.2161];
principalPoint = [318.9034, 257.5352];
imageSize      = [480, 640];
height         = 2.1798;    % mounting height in meters from the ground
pitch          = 14;        % pitch of the camera in degrees

camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
sensor        = monoCamera(camIntrinsics, height, 'Pitch', pitch);

videoReader = VideoReader('caltech_washington1.avi');

%
% Create the |helperMonoSensor| object and apply it to the video.
monoSensor   = helperMonoSensor(sensor);
monoSensor.LaneXExtentThreshold = 0.5;
% To remove false detections from shadows in this video, we only return
% vehicle detections with higher scores.
monoSensor.VehicleDetectionThreshold = 20;    

isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
    
    frame = readFrame(videoReader); % get a frame
    
    sensorOut = processFrame(monoSensor, frame);

    closePlayers = ~hasFrame(videoReader);
            
    isPlayerOpen = displaySensorOutputs(monoSensor, frame, sensorOut, closePlayers);
    
    timeStamp = 11.1333; % take snapshot for publishing at timeStamp seconds
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot_J(frame, sensor, sensorOut);
    end    
   
end

%%
% Display the video frame. Snapshot is taken at |timeStamp| seconds.
if ~isempty(snapshot)
    figure; imshow(snapshot)
end

