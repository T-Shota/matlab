%% ����3����CT�X�L�����f�[�^����̑��풊�o�Ɖ���
%#ok<*NOPTS>

%% ������
clear all, close all, clc; %#ok<CLALL,DUALC>

%% DICOM�u���E�U�[�ɂ��t�@�C���̊m�F
dicomBrowser(fullfile(matlabroot,'toolbox/images/imdata'))

%% ���̂�CT�X�L�����{�����[���f�[�^�̃��[�h
% ���O�ɃT���v���f�[�^���_�E�����[�h���Ă����K�v������
% MATLAB�̃f�X�N�g�b�v�́u�z�[���^�u�v�́u�A�h�I���v���J��
% �������ŁuImage Processing Toolbox Image Data�v�������B
load chestVolume

% CT�X�L�����f�[�^��0�`1�ɐ��K��
V = im2single(int16(V));

% �ŏ���12����\��
figure, montage(reshape(V(:,:,1:25:25*12),512,512,1,12),'DisplayRange',[]);

%% �{�����[���r���[���[�ŉ���(R2017a�V�@�\)
volumeViewer(V) 

%% �x�̗̈�؂�o��
% �A�N�e�B�u �R���^�[���g���ė̈撊�o

% XY���ʂ�XZ���ʂŒ����̃X���C�X��؂�o��
XY = V(:,:,160);
XZ = squeeze(V(256,:,:));

% 2D�̃X���C�X������
figure, imshow(XY, []);
figure, imshow(XZ, []);

%% �x�̗̈�؂�o��(XY����)
% �C���[�W�̗̈敪���A�v���P�[�V�����Ő؂�o��
imageSegmenter(XY)

% �u�������l�v��I�����A���\�b�h���u�蓮�������l�v�ɐݒ�B
% �������l��0.5098�t�߂ɁB
% �u�}�X�N�̔��]�v�Ŕx�Ƃ���ȊO�̕�����I��
% �u���E�̃N���A�v�ŊO�g�ɐڂ��镔�����폜
% �u���̓h��Ԃ��v�ōׂ������𖄂߂�
% �u�����t�H���W�[�v�A�u�}�X�N�̏k���v�ōׂ����S�~������
% �u�G�N�X�|�[�g�v�A�u�C���[�W�̃G�N�X�|�[�g�v�ŁA
% �}�X�N�C���[�W�Ƀ`�F�b�N������"mask_XY"�Ƃ������O�ŃG�N�X�|�[�g

%% ���������֐����g���Ĕx�̗̈�؂�o��(XY����)
[~,mask_XY] = segmentImageXY(XY);

%% �x�̗̈�؂�o��(XZ����)
imageSegmenter(XZ)

% �����菇��XZ�ɂ��Ă��x�𒊏o
% �u�������l�v��I�����A���\�b�h���u�O���[�o���������l�v�ɐݒ�
% �u�}�X�N�̔��]�v�Ŕx��O�i�Ƃ���
% �u���E�̃N���A�v�ŊO�g�ɐڂ��镔�����폜
% �u���̓h��Ԃ��v�ōׂ������𖄂߂�
% �u�����t�H���W�[�v�A�u�}�X�N�̏k���v�ōׂ����S�~�������B���a�𑝂₵�Ȃ���K�؂Ȓl��T��

%% ���������֐����g���Ĕx�̗̈�؂�o��(XZ����)
[~,mask_XZ] = segmentImageXZ(XZ);

%% 3�����A�N�e�B�u�R���^�[�Ŕx�S�̂𒊏o
% �A�N�e�B�u�R���^�[�̃V�[�h�ƂȂ�_���z����쐬
mask = false(size(V));
mask(:,:, 160) = mask_XY;
mask(256, :, :) = mask(256, :, :)|reshape(mask_XZ, [1, 512, 318]);

%% 3�����A�N�e�B�u�R���^�[�Ŕx�S�̂𒊏o
% (���Ԃ�������ꍇ�̓X�L�b�v)

% �q�X�g�O�����ϓ���
V = histeq(V);

% 3�����A�N�e�B�u�R���^�[(R2017a��3�����Ή�)
BW  = activecontour(V,mask,100,'Chan-Vese'); %#ok<NASGU>

%% ���炩���ߎ��s�������ʂ��g��
vars = load('BWs','BWs');
BWs = vars.BWs;
BW = BWs(:,:,:,end);

%% �}�X�N���g���Ĕx�̃{�����[���f�[�^�𒊏o���A����
segmentedImage = V.*single(BW);
volumeViewer(segmentedImage);

%% �x�̗e�ʂ��v�Z

% �o�C�i���̃{�����[���f�[�^�ɑ΂��ė̈��͂Ńs�N�Z�������v�Z
% (R2017b�V�@�\)
volLungsPixels = regionprops3(BW, 'Volume') 

% �ʐς̑傫������2���o��
areaLungsSorted = sort(volLungsPixels.Volume,'descend');
areaLungs = sum(areaLungsSorted(1:2));

% �e�s�N�Z���̃X�P�[����ݒ�
xSpacing = 0.76; %(mm)
ySpacing = 0.76; %(mm)
zSpacing = 1.25; %(mm)

% �e�ʂ����b�g��(L)�Ōv�Z
volLungsLiters = areaLungs*xSpacing*ySpacing*zSpacing*1e-6
% ���l�j���̔x�̗e��(6L)�Ƃ����ނˈ�v���Ă���

%% 3�����A�N�e�B�u�R���^�[�̏����ߒ�������
figure;
view(-185,25)
set(gcf,'Position',[979   372   560   649]);
zlim([0 300]);
camlight; camlight(-10,-80);
set(gcf, 'WindowStyle', 'docked') 
for ii = 2:13
    segmentedImage = V;
    segmentedImage(BWs(:,:,:,ii)) = .58;
    testVis = imresize(segmentedImage,.125);
    
    white_vol = isosurface(testVis(20:50,:,:),.57);
    gray_vol = isosurface(testVis(20:50,:,:),.52);
    
    white_patch = patch(white_vol,'FaceColor',[.8 .4 .5],'EdgeColor','none');
    gray_patch = patch(gray_vol,'FaceColor',[.4 .2 0],'EdgeColor','none','FaceAlpha',0.1);
    
    shg;
    drawnow limitrate;
end
