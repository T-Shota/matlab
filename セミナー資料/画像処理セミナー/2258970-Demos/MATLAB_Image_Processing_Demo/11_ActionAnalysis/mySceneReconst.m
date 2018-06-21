%% ステレオカメラのキャリブレーションとシーン認識

%% 初期化
close all;imtool close all

%% ステレオカメラキャリブレーション (コマンドラインもしくはアプリケーションタブ)
% チェッカーボードサイズ=108mm
folder1 = fullfile('images_20160928_1207','folder1');
folder2 = fullfile('images_20160928_1207','folder2');
squareSize = 108;
stereoCameraCalibrator(folder1,folder2,squareSize)

%% 保存してあるキャリブレーションデータ用いるとき
load('stereoParams_20160928.mat');

%% ステレオ平行化
% 左右ペアの画像を読込み・表示
I1 = imread('I1_7.jpg');
I2 = imread('I2_7.jpg');

figure;imshowpair(I1, I2, 'montage');

%% 重ねて表示
figure; imshow(stereoAnaglyph(I1, I2), 'InitialMagnification', 50);

%% キャリブレーションデータを用い、ステレオ平行化・表示
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);
figure;imshowpair(J1, J2, 'montage');

%% 重ねて表示 (J1がシアン)
figure, imshow(stereoAnaglyph(J1, J2), 'InitialMagnification', 50);
J1 = imhistmatch(J1,J2,255);

%% 視差(カメラからの距離に半比例)の計算・視差マップの表示（近くが白）
disparityMap = disparity(rgb2gray(J1), rgb2gray(J2), 'DisparityRange',[0 64]);
figure; imshow(disparityMap, [0 64], 'InitialMagnification', 50);

%% 画像の、3次元座標系への再構築・表示
%  視差マップ上の各点を3次元 world 座標系の点へマッピング
%  I1の光学中心がworld座標系の原点
pointCloud = reconstructScene(disparityMap, stereoParams);   % 799x1122x3 single: disparityMapの各ピクセルに対し、World座標系の[x,y.z]を計算し3次元部分に挿入
pointCloud = pointCloud / 1000;  %単位を mm から m へ変換
 
% カメラから3～7m離れた点のみプロット
zdisp = pointCloud(:, :, 3);    % 3次元部分の第三要素(距離)のみ抽出
zdisp(zdisp < 3.5 | zdisp > 8) = NaN;   % 1mより近い・7mより離れている点を除去
pointCloud(:,:,3) = zdisp;

figure;showPointCloud(pointCloud, J1, 'VerticalAxis', 'Y',...    %  色情報はJ1から。pointCloudオブジェクトの表示も可能
    'VerticalAxisDir', 'Down' );
xlabel('X');ylabel('Y');zlabel('Z');
xlim([-2 3]); ylim([-1.5 2]);zlim([3 8])
box on;

%% 人いる位置の３次元空間だけを取り出す　%%%%%%%%%%%
% 人検出
bbox1 = detectPeopleACF(J1,'WindowStride',4,'Threshold',-1);
I_ann = insertObjectAnnotation(J1,'rectangle',bbox1,1, 'Color','yellow','FontSize',18);
figure, imshow(I_ann);
sz = size(J1);
mask = zeros(sz(1),sz(2));
mask = mask > 0;

% 人検出したBoxのさらに中央部分だけを切り出すマスクを作成
mask(bbox1(2)+floor(bbox1(4)*0.25):bbox1(2)+floor(bbox1(4)*0.75)-1,...
    bbox1(1)+floor(bbox1(3)*0.4):bbox1(1)+floor(bbox1(3)*0.7)) = 1;

% maskされていない場所の情報はなくす
disparityMap(~mask) = 0;
disp_ann = insertObjectAnnotation(disparityMap,'rectangle',bbox1,1, 'Color','yellow','FontSize',18);

% 切り出し部分を重ねて表示
figure; imshowpair(I_ann, disp_ann,'falsecolor');

%% 人周辺の部分だけ再度３次元化
pointCloud = reconstructScene(disparityMap, stereoParams);   % 799x1122x3 single: disparityMapの各ピクセルに対し、World座標系の[x,y.z]を計算し3次元部分に挿入
pointCloud = pointCloud / 1000; 

zdisp = pointCloud(:, :, 3);    % 3次元部分の第三要素(距離)のみ抽出
zdisp(zdisp < 3 | zdisp > 10) = NaN;   % 1mより近い・7mより離れている点を除去
pointCloud(:,:,3) = zdisp;

figure;showPointCloud(pointCloud, J1, 'VerticalAxis', 'Y',...    %  色情報はJ1から。pointCloudオブジェクトの表示も可能
    'VerticalAxisDir', 'Down' );
xlabel('X');ylabel('Y');zlabel('Z');
xlim([-2 4]); ylim([-2 3]);zlim([0 10])
box on;

%% 得られた点群情報から位置を解析 
figure, 
h = histogram(zdisp);
[~,Idx] = max(h.BinCounts);
z = h.BinEdges(Idx);
xIdx = zdisp > z-0.1 & zdisp < z+0.1;
xdisp = pointCloud(:, :, 1);

% median関数を用いて中央値を取り出す
xmed = median(xdisp(xIdx),'omitnan');
position = [xmed z] %#ok<NOPTS>

%%
% Copyright 2016 The MathWorks, Inc.
