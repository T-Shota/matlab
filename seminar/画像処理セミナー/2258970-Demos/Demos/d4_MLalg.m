clear all, close all, clc;

%% ���̃X�N���v�g�����s�O�ɁA
% createImage�@�֐��Ńe�X�g�p�摜�𐶐����Ă��g��������

%% �摜�f�[�^�ǂݍ���
I = imread('frame10.png');
figure, imshow(I)

%% �G�b�W���o���g���Ă݂�
Ie = edge(I);
imshow(Ie)

%% ������
If = imfill(Ie, 'holes');
imshow(If)

%% 3�����v���b�g�ŉ摜�̗l�q�𗧑̓I�Ɋm�F
figure, surf(double(I)), shading interp
%zlim([0 30])

%% 2�l��
bw = imbinarize(I);
imshow(bw)

%% �t���b�h�t�B��(������)
bw2 = imfill(bw, 'holes');
imshow(bw2)

%% �Z�����ʂ��Ă���̈�𒊏o
I2 = I;
I2(~bw2) = 0;
imshow(I2)

%% �����t�H���W�[�������g���đO����1
I2 = uint16(I2);

I2x = 2*I2; %�P�x�l��2�{
Id = imdilate(I2, strel('square',7)); %�C���[�W��c��
imshowpair(I2x, Id, 'montage')

%% �Z�����E������
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

%% �蓮��2�l���Ƀg���C
I6 = I5 > 100;
imshow(I6)

%% �K��2�l��
I6 = ~imbinarize(I5, 'adaptive', 'Sensitivity',0.95);
imshow(I6)

%% ���E�ɐڂ��Ă���I�u�W�F�N�g(�w�i)����
I6 = imclearborder(I6);
imshow(I6)

%% �m�C�Y�̑傫�����m�F
imageRegionAnalyzer(I6)
%�t�B���^�Ŗʐ�10�ȉ����폜������ɁA"�C���[�W�̃G�N�X�|�[�g"�ł���

%% �m�C�Y����
I6 = bwareaopen(I6, 10);

%% �v���p�e�B���(������156��)
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

%% �֗��ȃA�v��
imageSegmenter(I)
