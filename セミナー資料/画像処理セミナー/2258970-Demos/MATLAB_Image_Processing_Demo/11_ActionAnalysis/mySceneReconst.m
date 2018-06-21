%% �X�e���I�J�����̃L�����u���[�V�����ƃV�[���F��

%% ������
close all;imtool close all

%% �X�e���I�J�����L�����u���[�V���� (�R�}���h���C���������̓A�v���P�[�V�����^�u)
% �`�F�b�J�[�{�[�h�T�C�Y=108mm
folder1 = fullfile('images_20160928_1207','folder1');
folder2 = fullfile('images_20160928_1207','folder2');
squareSize = 108;
stereoCameraCalibrator(folder1,folder2,squareSize)

%% �ۑ����Ă���L�����u���[�V�����f�[�^�p����Ƃ�
load('stereoParams_20160928.mat');

%% �X�e���I���s��
% ���E�y�A�̉摜��Ǎ��݁E�\��
I1 = imread('I1_7.jpg');
I2 = imread('I2_7.jpg');

figure;imshowpair(I1, I2, 'montage');

%% �d�˂ĕ\��
figure; imshow(stereoAnaglyph(I1, I2), 'InitialMagnification', 50);

%% �L�����u���[�V�����f�[�^��p���A�X�e���I���s���E�\��
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);
figure;imshowpair(J1, J2, 'montage');

%% �d�˂ĕ\�� (J1���V�A��)
figure, imshow(stereoAnaglyph(J1, J2), 'InitialMagnification', 50);
J1 = imhistmatch(J1,J2,255);

%% ����(�J��������̋����ɔ����)�̌v�Z�E�����}�b�v�̕\���i�߂������j
disparityMap = disparity(rgb2gray(J1), rgb2gray(J2), 'DisparityRange',[0 64]);
figure; imshow(disparityMap, [0 64], 'InitialMagnification', 50);

%% �摜�́A3�������W�n�ւ̍č\�z�E�\��
%  �����}�b�v��̊e�_��3���� world ���W�n�̓_�փ}�b�s���O
%  I1�̌��w���S��world���W�n�̌��_
pointCloud = reconstructScene(disparityMap, stereoParams);   % 799x1122x3 single: disparityMap�̊e�s�N�Z���ɑ΂��AWorld���W�n��[x,y.z]���v�Z��3���������ɑ}��
pointCloud = pointCloud / 1000;  %�P�ʂ� mm ���� m �֕ϊ�
 
% �J��������3�`7m���ꂽ�_�̂݃v���b�g
zdisp = pointCloud(:, :, 3);    % 3���������̑�O�v�f(����)�̂ݒ��o
zdisp(zdisp < 3.5 | zdisp > 8) = NaN;   % 1m���߂��E7m��藣��Ă���_������
pointCloud(:,:,3) = zdisp;

figure;showPointCloud(pointCloud, J1, 'VerticalAxis', 'Y',...    %  �F����J1����BpointCloud�I�u�W�F�N�g�̕\�����\
    'VerticalAxisDir', 'Down' );
xlabel('X');ylabel('Y');zlabel('Z');
xlim([-2 3]); ylim([-1.5 2]);zlim([3 8])
box on;

%% �l����ʒu�̂R������Ԃ��������o���@%%%%%%%%%%%
% �l���o
bbox1 = detectPeopleACF(J1,'WindowStride',4,'Threshold',-1);
I_ann = insertObjectAnnotation(J1,'rectangle',bbox1,1, 'Color','yellow','FontSize',18);
figure, imshow(I_ann);
sz = size(J1);
mask = zeros(sz(1),sz(2));
mask = mask > 0;

% �l���o����Box�̂���ɒ�������������؂�o���}�X�N���쐬
mask(bbox1(2)+floor(bbox1(4)*0.25):bbox1(2)+floor(bbox1(4)*0.75)-1,...
    bbox1(1)+floor(bbox1(3)*0.4):bbox1(1)+floor(bbox1(3)*0.7)) = 1;

% mask����Ă��Ȃ��ꏊ�̏��͂Ȃ���
disparityMap(~mask) = 0;
disp_ann = insertObjectAnnotation(disparityMap,'rectangle',bbox1,1, 'Color','yellow','FontSize',18);

% �؂�o���������d�˂ĕ\��
figure; imshowpair(I_ann, disp_ann,'falsecolor');

%% �l���ӂ̕��������ēx�R������
pointCloud = reconstructScene(disparityMap, stereoParams);   % 799x1122x3 single: disparityMap�̊e�s�N�Z���ɑ΂��AWorld���W�n��[x,y.z]���v�Z��3���������ɑ}��
pointCloud = pointCloud / 1000; 

zdisp = pointCloud(:, :, 3);    % 3���������̑�O�v�f(����)�̂ݒ��o
zdisp(zdisp < 3 | zdisp > 10) = NaN;   % 1m���߂��E7m��藣��Ă���_������
pointCloud(:,:,3) = zdisp;

figure;showPointCloud(pointCloud, J1, 'VerticalAxis', 'Y',...    %  �F����J1����BpointCloud�I�u�W�F�N�g�̕\�����\
    'VerticalAxisDir', 'Down' );
xlabel('X');ylabel('Y');zlabel('Z');
xlim([-2 4]); ylim([-2 3]);zlim([0 10])
box on;

%% ����ꂽ�_�Q��񂩂�ʒu����� 
figure, 
h = histogram(zdisp);
[~,Idx] = max(h.BinCounts);
z = h.BinEdges(Idx);
xIdx = zdisp > z-0.1 & zdisp < z+0.1;
xdisp = pointCloud(:, :, 1);

% median�֐���p���Ē����l�����o��
xmed = median(xdisp(xIdx),'omitnan');
position = [xmed z] %#ok<NOPTS>

%%
% Copyright 2016 The MathWorks, Inc.
