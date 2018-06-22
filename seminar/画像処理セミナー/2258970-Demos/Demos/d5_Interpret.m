clear all, close all, clc;

%% ���̃X�N���v�g�����s�O�ɁA
% createImage�@�֐��Ńe�X�g�p�摜�𐶐����Ă��g��������

%% �摜�f�[�^�ǂݍ���
I = imread('frame10.png');
figure, imshow(I)

%% 3��������
Ilim = I;
Ilim(Ilim > 30) = 30;
Ilim(Ilim < 15) = 15;
figure, surf(double(Ilim)), shading interp, colormap jet
zlim([15 30])

%% �P�x�l��2�l�����Ă݂�
bw = I > 75;
imshow(bw)

%% ������
bw = imfill(bw, 'holes');
imshow(bw)

%% ROI�͈͊O�̗̈���t���b�g��
I(~bw) = 0;
figure, surf(double(I)), shading interp, colormap jet

%% �����t�H���W�[�������g���đO����1
I2 = uint16(I);

I2x = 2*I2; %�P�x�l��2�{
strtbl = {'diamond', 'disk', 'cube', 'sphere', 'square', 'octagon'};
incl = 6;
Id = imdilate(I2, strel(strtbl{incl},9)); %�C���[�W��c��
imshowpair(I2x, Id, 'montage')

%% �������o���ăZ�����E������
I3 = I2x - Id;
imshow(I3,[])

%% ���̉摜�Ɣ�r
imshowpair(I(150:200, 150:200), I3(150:200, 150:200), 'montage')

%% �����t�H���W�[�������g���đO����2
I4 = imdilate(I3, strel('square',7)); 
imshowpair(I3, I4, 'montage')

%% ���ׂĔ�r
I5 = I4 - I3;
imshow(I5,[])

%% Otsu�@���g����2�l��
I6 = ~imbinarize(I5, 'adaptive', 'Sensitivity',1);
imshow(I6)

%% ���E�ɐڂ��Ă���I�u�W�F�N�g(�w�i)����
I6 = imclearborder(I6);
imshow(I6)

%%
imageRegionAnalyzer(I6)

%% �m�C�Y����
I6 = bwareaopen(I6, 10);

%% �v���p�e�B���
stats = regionprops(I6, 'Centroid');

%% �}�[�J�[�}��
pos = [cat(1,stats.Centroid)];

I_sc = insertMarker(I, pos, 'Size', 2, 'marker', 'star');
imshow(I_sc)

%% �Z�����e�L�X�g�}��
sz = size(pos);
frameBlobTxt = sprintf('Count %d', sz(1));
I_sc = insertText(I_sc, [1 1], frameBlobTxt, 'FontSize', 36, 'BoxOpacity', 0, 'TextColor', 'white');
imshow(I_sc)
