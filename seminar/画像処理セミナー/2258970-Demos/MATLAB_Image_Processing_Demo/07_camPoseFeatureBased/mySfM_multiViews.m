%% SfM�ɂ��J�����ʒu�����3�����_�Q�\�z

%% ������
clc;clear;close all;imtool close all; rng('default');

%% �p�����[�^
minNumPts = 1000;
minDisparity = 20;

%% �e�I�u�W�F�N�g�̍쐬

% ����ǂݍ��݃I�u�W�F�N�g�̍쐬
videoReader = vision.VideoFileReader('cube.mp4','VideoOutputDataType','uint8');
% �r�f�I�v���C���[�I�u�W�F�N�g�̍쐬
videoPlayer = vision.DeployableVideoPlayer;
% �|�C���g�g���b�J�[�I�u�W�F�N�g�̍쐬
tracker = vision.PointTracker('MaxBidirectionalError', 0.1,'NumPyramidLevels',3);

%% �X�g�b�v�{�^���̍쐬
a = true;
sz = get(0,'ScreenSize');
figure('MenuBar','none','Toolbar','none','Position',[20 sz(4)-100 100 70])
uicontrol('Style', 'pushbutton', 'String', 'Stop',...
        'Position', [20 20 100 40],...
        'Callback', 'a=false;');
    
%% �J�����̃p�����[�^�ǂݍ���
load('cameraParams2');   % �J�����p�����[�^�̓Ǎ���

%% 1���ڂ̉摜������
% 1���ڂ̉摜�̃����Y�c������\��
Irgb = step(videoReader);

Ic = undistortImage(Irgb, cameraParams);

% �O���C�X�P�[���ϊ�
I = rgb2gray(Ic);

% ����30�s�N�Z���͏����āA�����_���o�i�摜�G�b�W�ł̓����_��r���j
border = 30;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];

% �����_���o�F��ǓI�������擾����ׂɁANumOctaves��傫���ݒ�
initPoints   = detectSURFFeatures(I, 'NumOctaves', 5, 'MetricThreshold', 1, 'ROI', roi);

% ���o���ꂽ�R�[�i�[�_�ŁA�|�C���g�g���b�J�[��������
initPoints = initPoints.Location;

% 1���ڂ̉摜�̃f�[�^(�����_���W�A�J�����ʒu��p��)���AviewID=1�Ƃ��ēo�^
% ���̂Ƃ��̃J�����ʒu(���w���S)��World���W�̌��_
% �J�����̌�����World���W�n��Z���̐�����
vSet = viewSet       % ���viewSet�I�u�W�F�N�g�𐶐�
viewID = 1;
vSet = addView(vSet, viewID, 'Points', initPoints, 'Orientation', eye(3),...
    'Location', [0 0 0]);
vSet.Views     % ViewId=1 ���o�^����Ă���

initialize(tracker, initPoints, Ic);

images{1} = Ic;
prevI = Ic;
oldPoints = initPoints;

%% �c��̉摜�̃f�[�^���A���[�v������vSet�I�u�W�F�N�g�֓o�^
while a && viewID < 5
    Irgb = step(videoReader);

    % �����Y�c����
    Ic = undistortImage(Irgb, cameraParams);
    
    % �\���p�摜
    Iout = Ic;
        
    % �O���[�X�P�[���֕ϊ�
    I = rgb2gray(Ic);
    
    % ���摜��̑Ή��_�����o
    [currPoints, validIdx] = step(tracker, Ic);
    
    % �Ή��֌W���������������_�𒊏o
    matchedPoints1 = oldPoints(validIdx,:);
    matchedPoints2 = currPoints(validIdx,:);
    
    % �摜�\��
    Iout = insertShape(Iout,'line',[matchedPoints1, matchedPoints2]);
    Iout = insertText(Iout,[0 0],num2str(size(matchedPoints2,1),'# of tracked: %d'));
    Iout = insertText(Iout,[0 30],num2str(viewID,'View ID: %d'));
    
    % ���ϓI�Ȉړ���
    d = mean(sqrt(sum((matchedPoints1-matchedPoints2).^2,2)));
    
    % �ړ��ʂ��������l�𒴂������b�s�񐄒�
    if d > minDisparity
        viewID = viewID+1;
        images{viewID} = Ic;
        % �Ή��_�̏�񂩂�
        % ��O�̃J�����ʒu��p���ɑ΂���ʒu��p���𐄒�i������1�Ɖ���j
        % ���Bundle Adjustment�ŁA�����̌덷���͕␳�����
        %for i = 1:100
            % ��b�s��(2�̉摜��̓_�̑Ή��֌W)�̐���
            [fMatrix, inlierIdx] = estimateFundamentalMatrix( ...
                matchedPoints1, matchedPoints2, 'Method','MSAC', 'NumTrials',1000, 'DistanceThreshold',4);
            
            % �G�s�|�[���S���𖞂����Ȃ�����(��Ή��_)�̏���
            inlierPoints1 = matchedPoints1(inlierIdx, :);
            inlierPoints2 = matchedPoints2(inlierIdx, :);
            
            % ��b�s�񂩂�A��O�̃J�����ʒu��p���ɑ΂���A���J�����ʒu�E�p���𐄒�
            [relativeOrient, relativeLoc, validPointFraction] = ...
                cameraPose(fMatrix, cameraParams, inlierPoints1, inlierPoints2);
            
            % �L���ȓ_�̊����������Ȃ�܂ŌJ��Ԃ��A��b�s��̐�����s��
            %if validPointFraction > .8
            %   break;
            %elseif i == 100;
            %   % 100�񔽕����Ă�validPointFraction���Ⴂ�ꍇ�́A�G���[�ɂ���
            %   error('Unable to compute the Fundamental matrix');
            %end
        %end
        
        % 1�O�̃J�����ʒu��p�����擾 (addView �ŃZ�b�g��������)
        prevPose = poses(vSet, viewID-1);
        prevOrientation = prevPose.Orientation{1};
        prevLocation    = prevPose.Location{1};
        
        % ��Ԗڂ�View�ɑ΂���A���J�����ʒu��p�������߂�.
        orientation = relativeOrient * prevOrientation;
        location    = prevLocation + relativeLoc * prevOrientation;
        
        % ���ݒǐՂ��Ă���_��
        numPts = size(matchedPoints2,1);
        
        if numPts < minNumPts
            % �ǐՂ��Ă���_�����ő�ݒ�l�������Ȃ��ꍇ
            
            % ���ݒǐՂ��Ă���_�����O���邽�߂̃}�X�N�쐬
            mask = ones(size(I));
            mask = insertShape(mask,'FilledCircle',...
                [matchedPoints2 4*ones(size(matchedPoints2,1),1)],...
                'Opacity',1,'Color','black');
            mask = mask > 0;
            
            % �V���ɒǐՂ���_�����o
            newPoints = detectSURFFeatures(I, 'NumOctaves', 5, 'MetricThreshold', 10, 'ROI', roi);
            newPoints = newPoints.Location;
            
            % ���o�����_�������̒ǐՓ_�ɋ߂��Ȃ����m�F
            notTrackedIdx = mask(sub2ind(size(mask),round(newPoints(:,2)),round(newPoints(:,1))));
            
            % �߂��Ȃ����̂����𒊏o
            newPoints = newPoints(notTrackedIdx,:);
            numNewPts = size(newPoints,1);
            
            % �ǐՑΏۂ̓_��ǉ�
            trackedPoints = [matchedPoints2; newPoints];
        else
            trackedPoints = matchedPoints2;
        end
        
        % �����_�̍��W�A�J�����ʒu��p�����AvSet�֓o�^
        vSet = addView(vSet, viewID, 'Points', trackedPoints, 'Orientation', orientation, ...
            'Location', location);
        
        % ��O�̉摜�Ƃ̓����_�̑Ή��֌W���AvSet�֓o�^
        matches = [find(validIdx) (1:sum(validIdx))'];
        vSet = addConnection(vSet, viewID-1, viewID, 'Matches', matches);
        
        % �g���b�N�F����View�ɂ܂�����A�_�̑S�Ή��֌W���i�ꕔ�̃g���b�N�́A�SView�ɂ܂������Ă���j
        tracks1 = findTracks(vSet);  % ���݂܂ł̑SView�Ԃ̃g���b�N��񒊏o
        
        % ���݂܂ł̑SView�́A�J�����ʒu��p�����擾
        camPoses1 = poses(vSet);
        
        % �����摜��̓_�Ή��֌W����A�e�_��3�����ʒu�𐄒�
        xyzPoints1 = triangulateMultiview(tracks1, camPoses1, cameraParams);
        
        % �o���h�������œ_�Q�̈ʒu�ƃJ�����ʒu��p�����œK������
        [xyzPoints1, camPoses1, reprojectionErrors] = bundleAdjustment(xyzPoints1, ...
           tracks1, camPoses1, cameraParams, 'FixedViewId', 1, ...
           'PointsUndistorted', true);
        
        % �o���h�������Ŕ��C�������J�����ʒu��p����o�^
        vSet = updateView(vSet, camPoses1);       % �e�[�u���FcamPoses1 �̏��iViewID, Orientation, Location�j�ŃA�b�v�f�[�g

        % �ǐՓ_���Đݒ�
        setPoints(tracker, trackedPoints);
        
        % �ۑ�
        prevI = I;
        oldPoints = trackedPoints;
    end
    
    % �\��
    step(videoPlayer,Iout);
    
    drawnow limitrate;
    
    if isDone(videoReader)
        break;
    end

end
release(videoReader);

%% �g���b�N�FView�Ԃ̓_�̑Ή��֌W���A�ꕔ�̃g���b�N�́A�SView�ɂ܂������Ă���
tracks = findTracks(vSet);      % �SView�Ԃ̃g���b�N���̒��o
idx = arrayfun(@(x) numel(x.ViewIds),tracks) > 2; % 3�t���[���ȏ�g���b�L���O���Ă���_
tracks2 = tracks(idx);

% �|�C���g�N���E�h�̐F�����擾
color = zeros(numel(tracks2),3,'uint8');
for k = 1:numel(tracks2)
    Itracked = images{tracks2(k).ViewIds(1)};
    x = round(tracks2(k).Points(1,1));
    y = round(tracks2(k).Points(1,2));
    x = min(size(Itracked,2),x);
    y = min(size(Itracked,1),y);
    color(k,:) = Itracked(y,x,:);
end

%% ����3�����č\�����ʂ̕\��(�o���h�������O)
% �SView�́A�J�����ʒu��p�����擾
camPoses2 = poses(vSet);

% �����摜��̑Ή��_�g��p���A�e�_��3�������[���h���W���v�Z
xyzPoints2 = triangulateMultiview(tracks2, camPoses2, cameraParams);

figure; plotCamera(camPoses2, 'Size', 1);   % �J�����ʒu��p�����v���b�g
hold on;
ptCloud = pointCloud(xyzPoints2, 'Color', color);
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down','MarkerSize',20);

%xlim([-5 100]); ylim([-50 50]); zlim([0 100]);
xlabel('X');ylabel('Y');zlabel('Z');camorbit(0, -30); grid on; box on;
title('3�����č\������(�o���h�������O)');

%% ����3�����č\�����ʂ̕\��(�o���h��������)

% �SView�́A3�����_�Q���W�ƃJ�����ʒu��p�����o���h�������Ŕ��C��
[xyzPoints3, camPoses3, reprojectionErrors] = bundleAdjustment(...
    xyzPoints2, tracks2, camPoses2, cameraParams, 'FixedViewId', 1, 'PointsUndistorted', true);

figure; plotCamera(camPoses3, 'Size', 1);   % �J�����ʒu��p�����v���b�g
hold on;

goodIdx = (reprojectionErrors < 5);  % �G���[�l���傫���_������

ptCloud = pointCloud(xyzPoints3(goodIdx, :), 'Color', color(goodIdx,:));
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down','MarkerSize',20);

%xlim([-50 100]); ylim([-50 50]); zlim([0 100]);
xlabel('X');ylabel('Y');zlabel('Z');camorbit(0, -30); grid on; box on;
title('3�����č\������(�o���h��������)');

%% 3�����_�Q���畽�ʂ𐄒�
pc = ptCloud;
maxDistance = 1; % ���ʓ��̓_�Ԃ̍ő勗�����w��
referenceVector = [0,1,1]; % ���o���镽�ʂ̖@���x�N�g�����w��
maxAngularDistance = 5; % �@���x�N�g������20�x�ȓ��̕��ʂ�������
[planeMdl1, ~, outlierIndices] = pcfitplane(pc, maxDistance);
remainPtCloud = select(pc, outlierIndices);
hold on;
h1 = plot(planeMdl1);
h1.FaceAlpha = 0.5;
shg;

%% ���ʂ̕������̌W�����g���ČX���␳
n_a = planeMdl1.Normal; % ���ʂ܂ł̖@���x�N�g��
n_b = [0 0 1]; % ���낦���������x�N�g��(�����Z��)
n_c = cross(n_b,n_a); % �O��
v = n_c/norm(n_c); % ��]��(�P�ʃx�N�g��)
cos_theta = dot(n_a,n_b)/(norm(n_a)*norm(n_b)); % ��]�p
% ���h���Q�X�̌����ŉ�]�s����v�Z
R = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
sin_theta = sqrt(1-cos_theta^2);
M = eye(3)+sin_theta*R+(1-cos_theta)*R^2; % ��]�s��

% ���ʂ̕��s�ړ��ʒ��o(���_���畽�ʂ܂ł̐����̈�x�N�g��)
s = -planeMdl1.Parameters(4)/(planeMdl1.Parameters(1:3)*planeMdl1.Normal');
p_a = s*planeMdl1.Normal; % ���_���畽�ʂ܂ł̃x�N�g��(����)
p_b = [0 0 0]; % ���_(���s�ړ���͌��_)

% ���s�ړ�
tform = affine3d([eye(3) zeros(3,1); p_b-p_a 1]);
tform.T
ptCloudScene3 = pctransform(pc, tform);
figure, pcshow(ptCloudScene3, 'VerticalAxis','Z', 'VerticalAxisDir', 'Down','MarkerSize',20)
title('Shifted scene')
xlabel('X (mm)'), ylabel('Y (mm)'), zlabel('Z (mm)')

% ��]
tform2 = affine3d([M zeros(3,1); 0 0 0 1]);
tform2.T
ptCloudScene3 = pctransform(ptCloudScene3, tform2);
figure, pcshow(ptCloudScene3, 'VerticalAxis','Z', 'VerticalAxisDir', 'Down','MarkerSize',20)
title('Shifted and rotated scene')
xlabel('X (mm)'), ylabel('Y (mm)'), zlabel('Z (mm)')
%axis([-50 50 -60 0 -20 0]);

%% ���̈ʒu�����o(ROI�̒��o)
% �ʓx�������̈��HSV�Œ��o
hsv = rgb2hsv(im2double(ptCloudScene3.Color));
channel1Min = 0.953;
channel1Max = 0.046;
channel2Min = 0.614;
channel2Max = 1.000;
idx = ( (hsv(:,1) >= channel1Min) | (hsv(:,1) <= channel1Max) ) & ...
    (hsv(:,2) >= channel2Min ) & (hsv(:,2) <= channel2Max);
redPts = ptCloudScene3.Location(idx,:);
centroid = mean(redPts); % �d�S�ʒu�v�Z
sigma = sqrt(mean((redPts-repmat(centroid,[size(redPts,1) 1])).^2)); % 2�����[�����g�v�Z

% +/- 4sigma�̗̈�̉���
vert = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1];
vert = vert.*repmat(sigma,[8 1])*4+repmat(centroid,[8 1]);
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
hold on;
patch('Vertices',vert,'Faces',fac,...
      'FaceVertexCData',hsv(6),'FaceColor','flat','FaceAlpha',.3);
shg;

%% ���o���ꂽ���̂̋ߖT�𒊏o
roi_view = [centroid-4*sigma; centroid+4*sigma];
roi_view = reshape(roi_view,1,[]);
indices = findPointsInROI(ptCloudScene3, roi_view);
ptCloudB = select(ptCloudScene3,indices);
hFig = figure;
ptCloudB = pcdenoise(ptCloudB);
if centroid(3) < 0
    tform3 = affine3d([1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]);
    ptCloudB = pctransform(ptCloudB, tform3);
end
pcshow(ptCloudB, 'VerticalAxis','Z','MarkerSize',30)
xlabel('X'), ylabel('Y'), zlabel('Z')

%% ���b�V���\�z
tri = delaunay(ptCloudB.Location(:,1),ptCloudB.Location(:,2));
figure, hh = trisurf(tri,ptCloudB.Location(:,1),ptCloudB.Location(:,2),ptCloudB.Location(:,3));
set(hh,...
       'FaceVertexCData',ptCloudB.Color,...
       'LineStyle','none',...
       'FaceColor','interp');
view(-30,45);
axis equal;

%% Copyright 2016 The MathWorks, Inc. 