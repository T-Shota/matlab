%% �摜�Ԃ̈ʒu���킹

%% �摜�̓Ǎ��݁E�ό`
% ���摜
I1 = imread('mri.tif');

% -30�x��]
I2 = imrotate(I1,-30);

%% �A�v���P�[�V�����̋N��
registrationEstimator(I2,I1);

%%