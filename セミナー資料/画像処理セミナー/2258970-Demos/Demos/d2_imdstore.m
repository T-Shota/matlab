clear all, close all, clc;

%% �摜�f�[�^�m�F
imgDir   = fullfile(toolboxdir('vision'), 'visiondata','imageSets');
winopen(fullfile(imgDir))

%% ��ʂ̉摜�f�[�^�̎�舵�� : imageDatastore
imds = imageDatastore(fullfile(imgDir), 'IncludeSubfolders', true, ...
    'LabelSource', 'foldernames');

%% Apps�ŉ���
imageBrowser(imds)
helperDisplayImageMontage(imds.Files(1:8));

%% ���x�����
imds.Files(1)
imds.Labels(1)

%% �C���[�W�̐��⃉�x����\��
countEachLabel(imds)

%% 1���ڂ�ǂݍ���
img = readimage(imds, 1);
figure, imshow(img)

%% �J�b�v�̉摜(4����)
trypano = find(imds.Labels == 'cups', 1);
img = readimage(imds, trypano+3);
figure, imshow(img)

