%% eSFR�`���[�g�����Ԏ��g������(SFR)���v���b�g

%% �`���[�g�摜�ǂݍ���
I = imread('eSFRTestImage.jpg');
figure, imshow(I);

%% �t�K���}�␳
I_lin = rgb2lin(I);

%% eSFR�`���[�g�̐���
chart = esfrChart(I_lin);
figure, displayChart(chart)

%% �eROI�̃G�b�W�̐�s�x���v��
sharpnessTable = measureSharpness(chart)

%% 25�Ԗڂ�26�Ԗڂ�SFR���v���b�g
plotSFR(sharpnessTable,'ROIIndex',[25 26]);
