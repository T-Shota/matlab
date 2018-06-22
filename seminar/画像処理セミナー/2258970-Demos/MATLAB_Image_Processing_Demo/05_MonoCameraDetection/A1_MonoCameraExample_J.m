%% Monocular Camera Sensor Design�F�P��J�������W���[���̓��샂�f��
% �����Ǝԗ��̌��o�E���[���h���W�Ō��ʏo��
% ���ԂƏ�Q���܂ł̋����̐���
% Copyright 2016 The MathWorks, Inc. 
clc;clear;close all;imtool close all;

%% �J���������Ɣz�u�̃p�����[�^��ݒ�imonoCamera�N���X�FimageToWorld, worldToImage ���\�b�h(�_���W�̕ϊ�)��񋟁j
% �J�����L�����u���[�V�����ŋ��߂��J���������p�����[�^��ݒ�i��p�N���X�j
%    �i�����Y�c�����Ȃ����߁A�����ł̓����Y�c�␳�͂Ȃ��j
focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] ���w���S in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);   % �����p�����[�^�̃N���X

% �ԗ��ɑ΂���J�����̔z�u���w��i�O���p�����[�^����̎Z�o���\�j
height = 2.1798;    % �n�ʂ���̍���(�����ł�m��P�ʂɎg�p)
pitch  = 14;        % �J�����̃s�b�`(��������̊p�x�F�P�ʂ͓x) �����ł�Roll��Yaw�̓[��

% ��L�p�����[�^����AmonoCamera�I�u�W�F�N�g���`
%     �ԗ����W�F���Ԃɑ΂��ČŒ�B���_�̓J�����̏œ_�̐^���̒n�ʁBX�F�O�������AY�F������
%     ����Ȓn�ʂ�����
% monoCamera�I�u�W�F�N�g�F�J���������E�O���p�����[�^���i�[�A�摜�̍��W�Ǝԗ����W�𑊌ݕϊ�����֐����
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch)

%% ����t�@�C���m�F
winopen('caltech_cordova1.avi');

%% ����t�@�C������摜���ꖇ�Ǎ���
videoReader = VideoReader('caltech_cordova1.avi');
videoReader.CurrentTime = 0.06667;        % �Ǎ��ރt���[���̃^�C���X�^���v���w��
frame = readFrame(videoReader);           % 1�t���[���Ǎ���
figure; imshow(frame)                     % �\��

%% ���Ր}�ւ̕ϊ��ibirdsEyeView�N���X�j
%    �ȍ~�̉�͂̊ȒP���̂��߁A�悸���Ր}�֕ϊ��i���̑������A�Ԑ������s�ɂȂ�j
% �o�͉摜�̗̈���w��imonoCamera�I�u�W�F�N�g�������̃J���������̋����P�ʂ��g�p�j
%web(fullfile(docroot, 'driving/ref/birdseyeview-class.html#input_argument_d0e22799')); % ��`

% birdsEyeView �N���X�̐����E�ϊ�
bottomOffset      = 3;   % ��O3m�悩��
distAheadOfSensor = 30;  % ����30m��܂ł̒��Ր}
spaceToOneSide    = 6;   % ���S���獶�E6m���̒��Ր}
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax] �ԗ����W�n
imageSize = [NaN, 250];      % �ϊ���̒��Ր}�̉摜�s�N�Z���T�C�Y�i��250pix�A�c�͎����v�Z�j

% ���Ր}�����p�N���X�imonoCamera�N���X���琶���j
birdsEyeConfig  = birdsEyeView(sensor, outView, imageSize);

% ���Ր}�֕ϊ�
birdsEyeImage = transformImage(birdsEyeConfig,frame);

figure; imshow(birdsEyeImage);

%% ���̃y�C���g�̈�̌��o�i�ԗ����W�A�{���͔C�Ӂj
%    �ԗ����W���g�p����̂ŁA���L�̓J�������Ɉˑ����Ȃ�
birdsEyeImage = rgb2gray(birdsEyeImage);    % ���Ր}���O���[�X�P�[���֕ϊ�
vehicleROI    = outView - [-1, 2, -3, 3];   % [xmin, xmax, ymin, ymax] ���o�̈�����������Ɏw��
approxLaneMarkerWidthVehicle = 0.25;          % �����̕��i25cm�j

% Detect lane features�F�����ł͗�Ƃ��ăV���v���Ȃ��̂��g�p�i���[���h���W�x�[�X�Ōv�Z�j
%   ���E�������邢�A�`25cm���̗̈�
%   M. Nieto, J. A. Laborda, L. Salgado, "Road enviornment modeling using robust perspective analysis and recursive Bayesian segmentation," Machine Vision and Applications, Volume 22, Issue 6, pp.927-945, 2011.
laneSensitivity = 0.25;
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, approxLaneMarkerWidthVehicle, ...
                       'ROI', vehicleROI, 'Sensitivity', laneSensitivity);  % M����Ŏ���
figure; imshow(birdsEyeViewBW);

%% ���Ԑ��ɉ����������̋����̌��o�Fax^2 + bx + c �փJ�[�u�t�B�b�e�B���O�i�����ł́A�ő�2�{�j
[imageX, imageY] = find(birdsEyeViewBW);        % �S�Ă̑O�i�s�N�Z���́AX���W�EY���W
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);  % �ԗ����W�֕ϊ�

% RANdom SAmple Consensus (RANSAC)��p���āA�m�C�Y�����钆�ň��肵�ăJ�[�u�t�B�b�e�B���O
% parabolicLaneBoundary �I�u�W�F�N�g�̃x�N�g����Ԃ�
boundaryWidth = 3*approxLaneMarkerWidthVehicle;  % ��d���̏ꍇ����{�̐��Ƃ��Č��o���邽�߂ɕ���3�{��
maxLanes      = 2;                               % �ő�2�{�܂Ō��o => "��"

% 2���֐��ŕ��
[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints, boundaryWidth, ...
    'MaxNumBoundaries',maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn_J);

% ���ʂ̊m�F�Fy = ax^2 + bx + c �̌W��
boundaries(:).Parameters

%% ���Ԑ��̋����̌���
% �O�����FHeuristics�ɂ��A���f�����̔������̏���

% �Z�������������i���f�����j
maxPossibleXLength = diff(vehicleROI(1:2));      % �ԗ����WROI�̏c�̒���
minXLength         = maxPossibleXLength * 0.60;  % ROI�̏c����60��(minXLength)�ȏ�̂��̂̂ݑI��
isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
boundaries    = boundaries(isOfMinLength);

% �����ꓙ�̑������̂�����
% Remove additional boundaries based on the strength metric computed by the
% <matlab:doc('findParabolicLaneBoundaries'); |findParabolicLaneBoundaries|> function. Set a lane strength threshold
% based on ROI and image size.
birdsImageROI = vehicleToImageROI_J(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

maxPointsInOneLane = numel(unique(vehiclePoints(:,1)));  %  Find the maximum number of unique x-axis locations possible for any lane boundary
maxLaneLength = diff(vehicleROI(1:2));            % �ԗ����WROI�̏c�̒���
maxStrength   = maxPointsInOneLane/maxLaneLength; % maximum possible lane strength for this image size/ROI size specification
% Reject weak boundaries
isStrong      = [boundaries.Strength] > 0.4*maxStrength;
boundaries    = boundaries(isStrong);

%% �����̎�ނ̕��ށi����/�_���j�ƁA���Ԑ��̍��E�̐��̔F��
boundaries = classifyLaneTypes_J(boundaries, boundaryPoints);

% Locate two ego-lanes if they are present.
xOffset    = 0;   %  0 meters from the sensor
% parabolicBoundary �N���X�̃��\�b�h���g�p���āAxOffset�ɑΉ�����Y���W���v�Z
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
% ���̎�ނ̔��ʌ��ʂ̊m�F
leftEgoBoundary.BoundaryType
rightEgoBoundary.BoundaryType

%% ���o�����������A�J�����摜�ƁA���Ր}��֕\��
xVehiclePoints = bottomOffset:distAheadOfSensor;
%     ��3������ birdsEyeView �I�u�W�F�N�g�̏ꍇ�A���Ր}�ւ̑}��
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeWithEgoLane, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
%     ��3������ monoCamera �I�u�W�F�N�g�̏ꍇ�A���̉摜�ւ̑}��
frameWithEgoLane = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
frameWithEgoLane = insertLaneBoundary(frameWithEgoLane, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

figure
subplot('Position', [0, 0, 0.5, 1.0]) % [left, bottom, width, height] in normalized units
imshow(birdsEyeWithEgoLane);     % ���Ր}�i�E�j
subplot('Position', [0.5, 0, 0.5, 1.0])
imshow(frameWithEgoLane)         % �J�����摜�i���j

%% [�O���ԗ����o] �ԗ����W�n
% Aggregate Channel Features (ACF) ��p�����ԗ����o��
% �i�����ł̓g���b�L���O�͖��g�p�j
detector = vehicleDetectorACF();
%detector = vehicleDetectorFasterRCNN();
vehicleWidth = [1.5, 2.5];        % ���o����ԗ����F1.5-2.5m

% �P��J�������o��I�u�W�F�N�g�i�H�ʂɂ���A�w�肵�����̕��̂̂݌��o�j
monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);
[bboxes, scores] = detect(monoDetector, frame)    % �ԗ����o�̎��s

%% �O���ԗ��܂ł̋������v�Z�E����
locations = computeVehicleLocations_J(bboxes, sensor);  % ���E�{�b�N�X�����̒��S�܂ł̋������v�Z
imgOut = insertVehicleDetections_J(frameWithEgoLane, locations, bboxes);
figure; imshow(imgOut);

%% ����ɑ΂��ă��[�v�������s��
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

%% �قȂ铮��ɑ΂��ď���
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

