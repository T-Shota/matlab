clear all, close all, clc;

%% �X�J���[�f�[�^
a = 3;

%% �s��쐬(3x3)
a = [1 2 3; 4 5 6; 7 8 9];

%% �s��쐬(5x5 : �֐����p)
a = randi(255, 5);
imshow(a,[])

%% �e�X�g�p�摜�𐶐�
createImage

%% �摜�ǂݍ���
I = imread('frame10.png');

%% ����
imshow(I)

%% �q�X�g�O�����m�F
imhist(I)

%% 2�l��(�蓮)
bw = I > 80;
imshow(bw)

%% 2�l��
bw = imbinarize(I);
imshow(bw)

%% �t���b�h�t�B��(������)
bw2 = imfill(bw, 'holes');
imshow(bw2)

%% �Z�����ʂ��Ă���̈�𒊏o
I(~bw2) = 255;
imshow(I)

%% ����m�F
implay('ecolicells.avi')
