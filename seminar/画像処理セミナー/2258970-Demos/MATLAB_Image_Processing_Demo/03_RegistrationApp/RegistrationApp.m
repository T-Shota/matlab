%% 画像間の位置合わせ

%% 画像の読込み・変形
% 元画像
I1 = imread('mri.tif');

% -30度回転
I2 = imrotate(I1,-30);

%% アプリケーションの起動
registrationEstimator(I2,I1);

%%