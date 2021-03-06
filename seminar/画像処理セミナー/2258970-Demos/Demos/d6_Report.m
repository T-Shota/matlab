clear all, close all, clc;

%% このスクリプトを実行前に、
% createImage　関数でテスト用画像を生成してお使い下さい

%% 画像データ読み込み
%動画像の10フレーム目を予め静止画として保存してあります
I = imread('frame10.png');
figure, imshow(I)

%% 3次元可視化
%セルが写っている領域以外の輝度のフラットネスを確認します
Ilim = I;
Ilim(Ilim > 30) = 30;
Ilim(Ilim < 15) = 15;
figure, surf(double(Ilim)), shading interp, colormap jet
zlim([15 30])

%% ROIを設定します
%輝度値で2値化してみます
bw = I > 70;
imshow(bw)

%% 穴埋め
bw = imfill(bw, 'holes');
imshow(bw)

%% ROI範囲外の領域をフラットに
I(~bw) = 0;
figure, surf(double(I)), shading interp, colormap jet

%% ここからセルの分離処理に移ります
%モルフォロジー処理を使って前処理1
I2 = uint16(I);

I2x = 2*I2; %輝度値を2倍
strtbl = {'diamond', 'disk', 'cube', 'sphere', 'square', 'octagon'};
incl = 5;
Id = imdilate(I2, strel(strtbl{incl}, 7)); %イメージを膨張
imshowpair(I2x, Id, 'montage')

%% 差分を出してセル境界を強調
I3 = I2x - Id;
imshow(I3,[])

%% 元の画像と比較
imshowpair(I(150:200, 150:200), I3(150:200, 150:200), 'montage')

%% モルフォロジー処理を使って前処理2
I4 = imdilate(I3, strel('square',7)); 
imshowpair(I3, I4, 'montage')

%% 並べて比較
I5 = I4 - I3;
imshow(I5,[])

%% Otsu法を使った2値化
I6 = ~imbinarize(I5, 'adaptive', 'Sensitivity',0.95);
imshow(I6)

%% 境界に接しているオブジェクト(背景)除去
I6 = imclearborder(I6);
imshow(I6)

%% ノイズ除去
%面積が10ピクセル以下の領域を削除します
I6 = bwareaopen(I6, 10);

%% プロパティ解析
%セルの各重心点にマーキングを施すため、重心の位置を算出します
stats = regionprops(I6, 'Centroid');

%% マーカー挿入
pos = [cat(1,stats.Centroid)];
I_sc = insertMarker(I, pos, 'Size', 2, 'marker', 'star');
imshow(I_sc)

%% セル数テキスト挿入
sz = size(pos);
frameBlobTxt = sprintf('Count %d', sz(1));
I_sc = insertText(I_sc, [1 1], frameBlobTxt, 'FontSize', 36, 'BoxOpacity', 0, 'TextColor', 'white');
imshow(I_sc)
