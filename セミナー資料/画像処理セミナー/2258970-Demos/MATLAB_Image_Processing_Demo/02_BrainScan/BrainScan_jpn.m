%% Brain Scan Demo (DICOM image processing)
% �{�f���ł́A3�����̔]�X�L����DICOM�f�[�^��ǂݍ��݁A�]�����݂̂�
% �{�����[���\�������݂܂�
% Copyright 2012 The MathWorks, Inc. 

%% ������
clear all; close all; clc; imtool close all; %#ok<*CLALL>

%% NIfTI�摜�̓ǂݎ��Ή�(R2017b)
% NIfTI (Neuroimaging Informatics Technology Initiative) 
% DICOM�t�@�C���Q��P��t�@�C���Ƃ��Ď�舵����
V = niftiread('brain.nii');
figure, montage(permute(V,[1 2 4 3]),'DisplayRange',[]);

%% DICOM�u���E�U�[��DICOM�t�@�C���̊m�F(R2017b)
fileFolder = fullfile(pwd, 'Series 8');
dicomBrowser(fileFolder);

%% DICOM�t�@�C���Q�̓ǂݍ���(R2017b)
details = dicomCollection(fileFolder)

%% DICOM�t�@�C���̃��^���ǂݏo��
info = dicominfo(details.Filenames{1}(1));
voxel_size = [info.PixelSpacing; info.SliceThickness]';

%% DICOM����{�����[���f�[�^�ǂݍ���(R2017b)
[D,s,d] = dicomreadVolume(fileFolder);
montage(D,'DisplayRange',[]);

%% ���ɋP�x�̏��������̂��폜
mriAdjust = D;
lb = 40;  % lower threshold (ignore CSF & air)
mriAdjust(mriAdjust <= lb) = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% ���W���ƐڐG���Ă��镔���Ȃǂ̍폜
ub = 100; % upper threshold (ignore skull & other hard tissue)
mriAdjust(mriAdjust >= ub) = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% �s�v�Ȕ]�̉��̗̈��؂���
mriAdjust(175:end,:,:)  = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% �Q�l��
bw    = mriAdjust > 0;
figure, montage(bw,'DisplayRange',[]);

%% �I�[�v�������ł���̈�ȉ��̕������폜
nhood = ones([7 7 3]);
bw = imopen(bw,nhood);
figure, montage(bw,'DisplayRange',[]);

%% �]�̕����̃Z�O�����e�[�V�������s���܂�
% regionprops�Œ��S�_�Ɩʐς��m�F���܂�
L       = bwlabeln(bw);
stats   = regionprops('table',L,'Area')

%% �ł��ʐς��傫��������I��
A       = stats.Area;
biggest = find(A == max(A));
mriAdjust(L ~= biggest) = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% �R���g���X�g����
imA     = imadjust(mriAdjust(:, :, 30));
figure, montage(mriAdjust,'DisplayRange',[]);

%% �]�̕����̂ݒ��o���āA�\��
level = 65;
mriBrainPartition = uint8(zeros(size(mriAdjust)));    %0=outside brain (head/air)
mriBrainPartition(mriAdjust<level & mriAdjust>0) = 2; %2=gray matter
mriBrainPartition(mriAdjust>=level) = 3;              %3=white matter
figure,imshow(mriBrainPartition(:,:,30),[])

%% �]�����݂̂�3�����\��
Ds = imresize(mriBrainPartition,0.25,'nearest');

% �f�[�^�̌������C��
Ds = flip(Ds,1);
Ds = flip(Ds,2);
Ds = squeeze(Ds);
Ds = permute(Ds,[3 2 1]);

% �{�N�Z���̃X�P�[�����O
voxel_size2 = voxel_size([1 3 2]).*[4 1 4];

%���������ƃO���[�̕����̃T�[�t�F�X���쐬
white_vol = isosurface(Ds,2.5);
gray_vol  = isosurface(Ds,1.5);

% ����
h = figure('visible','off','outerposition',[0 0 800 600],'renderer','openGL');
patch(white_vol,'FaceColor','b','EdgeColor','none');
patch(gray_vol,'FaceColor','y' ,'EdgeColor','none',...
  'FaceAlpha',0.5);
view(45,15); daspect(1./voxel_size2); axis tight;axis off;
camlight; camlight(-80,-10); lighting phong;
movegui(h, 'center');
set(h,'visible','on');

%% �I��
