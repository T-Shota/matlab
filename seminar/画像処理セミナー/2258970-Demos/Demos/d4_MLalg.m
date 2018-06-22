clear all, close all, clc;

%% このスクリプトを実行前に、
% createImage　関数でテスト用画像を生成してお使い下さい

%% 画像データ読み込み
I = imread('frame10.png');
figure, imshow(I)

%% エッジ検出を使ってみる
Ie = edge(I);
imshow(Ie)

%% 穴埋め
If = imfill(Ie, 'holes');
imshow(If)

%% 3次元プロットで画像の様子を立体的に確認
figure, surf(double(I)), shading interp
%zlim([0 30])

%% 2値化
bw = imbinarize(I);
imshow(bw)

%% フラッドフィル(穴埋め)
bw2 = imfill(bw, 'holes');
imshow(bw2)

%% セルが写っている領域を抽出
I2 = I;
I2(~bw2) = 0;
imshow(I2)

%% モルフォロジー処理を使って前処理1
I2 = uint16(I2);

I2x = 2*I2; %輝度値を2倍
Id = imdilate(I2, strel('square',7)); %イメージを膨張
imshowpair(I2x, Id, 'montage')

%% セル境界を強調
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

%% 手動で2値化にトライ
I6 = I5 > 100;
imshow(I6)

%% 適応2値化
I6 = ~imbinarize(I5, 'adaptive', 'Sensitivity',0.95);
imshow(I6)

%% 境界に接しているオブジェクト(背景)除去
I6 = imclearborder(I6);
imshow(I6)

%% ノイズの大きさを確認
imageRegionAnalyzer(I6)
%フィルタで面積10以下を削除した後に、"イメージのエクスポート"でも可

%% ノイズ除去
I6 = bwareaopen(I6, 10);

%% プロパティ解析(正解は156個)
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

%% 便利なアプリ
imageSegmenter(I)
