%% ���̏���
% 
%% �摜�ǂݍ���
A = imread('foggysf2.jpg');
figure, imshow(A);

%% ���̏���
B = imreducehaze(A, 0.9, 'method', 'approxdcp');
figure, imshow(B);

%% ���ׂĕ\��
figure, imshowpair(A, B, 'montage')