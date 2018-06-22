clear all, close all, clc;

%% 画像データ確認
imgDir   = fullfile(toolboxdir('vision'), 'visiondata','imageSets');
winopen(fullfile(imgDir))

%% 大量の画像データの取り扱い : imageDatastore
imds = imageDatastore(fullfile(imgDir), 'IncludeSubfolders', true, ...
    'LabelSource', 'foldernames');

%% Appsで可視化
imageBrowser(imds)
helperDisplayImageMontage(imds.Files(1:8));

%% ラベル情報
imds.Files(1)
imds.Labels(1)

%% イメージの数やラベルを表示
countEachLabel(imds)

%% 1枚目を読み込み
img = readimage(imds, 1);
figure, imshow(img)

%% カップの画像(4枚目)
trypano = find(imds.Labels == 'cups', 1);
img = readimage(imds, trypano+3);
figure, imshow(img)

