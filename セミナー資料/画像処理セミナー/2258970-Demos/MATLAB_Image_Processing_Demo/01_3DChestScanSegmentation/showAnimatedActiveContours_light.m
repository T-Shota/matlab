%% 胸部3次元CTスキャンデータからの臓器抽出と可視化
%#ok<*NOPTS>

%% 初期化
clear all, close all, clc; %#ok<CLALL,DUALC>

%% DICOMブラウザーによるファイルの確認
dicomBrowser(fullfile(matlabroot,'toolbox/images/imdata'))

%% 胴体のCTスキャンボリュームデータのロード
% 事前にサンプルデータをダウンロードしておく必要がある
% MATLABのデスクトップの「ホームタブ」の「アドオン」を開く
% 検索窓で「Image Processing Toolbox Image Data」を検索。
load chestVolume

% CTスキャンデータを0～1に正規化
V = im2single(int16(V));

% 最初の12枚を表示
figure, montage(reshape(V(:,:,1:25:25*12),512,512,1,12),'DisplayRange',[]);

%% ボリュームビューワーで可視化(R2017a新機能)
volumeViewer(V) 

%% 肺の領域切り出し
% アクティブ コンターを使って領域抽出

% XY平面とXZ平面で中央のスライスを切り出し
XY = V(:,:,160);
XZ = squeeze(V(256,:,:));

% 2Dのスライスを可視化
figure, imshow(XY, []);
figure, imshow(XZ, []);

%% 肺の領域切り出し(XY平面)
% イメージの領域分割アプリケーションで切り出し
imageSegmenter(XY)

% 「しきい値」を選択し、メソッドを「手動しきい値」に設定。
% しきい値を0.5098付近に。
% 「マスクの反転」で肺とそれ以外の部分を選択
% 「境界のクリア」で外枠に接する部分を削除
% 「穴の塗りつぶし」で細かい穴を埋める
% 「モルフォロジー」、「マスクの縮小」で細かいゴミを除去
% 「エクスポート」、「イメージのエクスポート」で、
% マスクイメージにチェックを入れて"mask_XY"という名前でエクスポート

%% 生成した関数を使って肺の領域切り出し(XY平面)
[~,mask_XY] = segmentImageXY(XY);

%% 肺の領域切り出し(XZ平面)
imageSegmenter(XZ)

% 同じ手順でXZについても肺を抽出
% 「しきい値」を選択肢、メソッドを「グローバルしきい値」に設定
% 「マスクの反転」で肺を前景とする
% 「境界のクリア」で外枠に接する部分を削除
% 「穴の塗りつぶし」で細かい穴を埋める
% 「モルフォロジー」、「マスクの縮小」で細かいゴミを除去。半径を増やしながら適切な値を探索

%% 生成した関数を使って肺の領域切り出し(XZ平面)
[~,mask_XZ] = segmentImageXZ(XZ);

%% 3次元アクティブコンターで肺全体を抽出
% アクティブコンターのシードとなる論理配列を作成
mask = false(size(V));
mask(:,:, 160) = mask_XY;
mask(256, :, :) = mask(256, :, :)|reshape(mask_XZ, [1, 512, 318]);

%% 3次元アクティブコンターで肺全体を抽出
% (時間がかかる場合はスキップ)

% ヒストグラム均等化
V = histeq(V);

% 3次元アクティブコンター(R2017aで3次元対応)
BW  = activecontour(V,mask,100,'Chan-Vese'); %#ok<NASGU>

%% あらかじめ実行した結果を使う
vars = load('BWs','BWs');
BWs = vars.BWs;
BW = BWs(:,:,:,end);

%% マスクを使って肺のボリュームデータを抽出し、可視化
segmentedImage = V.*single(BW);
volumeViewer(segmentedImage);

%% 肺の容量を計算

% バイナリのボリュームデータに対して領域解析でピクセル数を計算
% (R2017b新機能)
volLungsPixels = regionprops3(BW, 'Volume') 

% 面積の大きい順に2つ取り出す
areaLungsSorted = sort(volLungsPixels.Volume,'descend');
areaLungs = sum(areaLungsSorted(1:2));

% 各ピクセルのスケールを設定
xSpacing = 0.76; %(mm)
ySpacing = 0.76; %(mm)
zSpacing = 1.25; %(mm)

% 容量をリットル(L)で計算
volLungsLiters = areaLungs*xSpacing*ySpacing*zSpacing*1e-6
% 成人男性の肺の容量(6L)とおおむね一致している

%% 3次元アクティブコンターの処理過程を可視化
figure;
view(-185,25)
set(gcf,'Position',[979   372   560   649]);
zlim([0 300]);
camlight; camlight(-10,-80);
set(gcf, 'WindowStyle', 'docked') 
for ii = 2:13
    segmentedImage = V;
    segmentedImage(BWs(:,:,:,ii)) = .58;
    testVis = imresize(segmentedImage,.125);
    
    white_vol = isosurface(testVis(20:50,:,:),.57);
    gray_vol = isosurface(testVis(20:50,:,:),.52);
    
    white_patch = patch(white_vol,'FaceColor',[.8 .4 .5],'EdgeColor','none');
    gray_patch = patch(gray_vol,'FaceColor',[.4 .2 0],'EdgeColor','none','FaceAlpha',0.1);
    
    shg;
    drawnow limitrate;
end
