clear all, close all, clc;

%% スカラーデータ
a = 3;

%% 行列作成(3x3)
a = [1 2 3; 4 5 6; 7 8 9];

%% 行列作成(5x5 : 関数利用)
a = randi(255, 5);
imshow(a,[])

%% テスト用画像を生成
createImage

%% 画像読み込み
I = imread('frame10.png');

%% 可視化
imshow(I)

%% ヒストグラム確認
imhist(I)

%% 2値化(手動)
bw = I > 80;
imshow(bw)

%% 2値化
bw = imbinarize(I);
imshow(bw)

%% フラッドフィル(穴埋め)
bw2 = imfill(bw, 'holes');
imshow(bw2)

%% セルが写っている領域を抽出
I(~bw2) = 255;
imshow(I)

%% 動画確認
implay('ecolicells.avi')
