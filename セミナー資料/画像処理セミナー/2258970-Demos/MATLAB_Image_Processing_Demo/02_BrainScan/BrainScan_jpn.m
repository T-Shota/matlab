%% Brain Scan Demo (DICOM image processing)
% 本デモでは、3次元の脳スキャンDICOMデータを読み込み、脳部分のみの
% ボリューム表示を試みます
% Copyright 2012 The MathWorks, Inc. 

%% 初期化
clear all; close all; clc; imtool close all; %#ok<*CLALL>

%% NIfTI画像の読み取り対応(R2017b)
% NIfTI (Neuroimaging Informatics Technology Initiative) 
% DICOMファイル群を単一ファイルとして取り扱える
V = niftiread('brain.nii');
figure, montage(permute(V,[1 2 4 3]),'DisplayRange',[]);

%% DICOMブラウザーでDICOMファイルの確認(R2017b)
fileFolder = fullfile(pwd, 'Series 8');
dicomBrowser(fileFolder);

%% DICOMファイル群の読み込み(R2017b)
details = dicomCollection(fileFolder)

%% DICOMファイルのメタ情報読み出し
info = dicominfo(details.Filenames{1}(1));
voxel_size = [info.PixelSpacing; info.SliceThickness]';

%% DICOMからボリュームデータ読み込み(R2017b)
[D,s,d] = dicomreadVolume(fileFolder);
montage(D,'DisplayRange',[]);

%% 非常に輝度の小さいものを削除
mriAdjust = D;
lb = 40;  % lower threshold (ignore CSF & air)
mriAdjust(mriAdjust <= lb) = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% 頭蓋骨と接触している部分などの削除
ub = 100; % upper threshold (ignore skull & other hard tissue)
mriAdjust(mriAdjust >= ub) = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% 不要な脳の下の領域を切り取り
mriAdjust(175:end,:,:)  = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% ２値化
bw    = mriAdjust > 0;
figure, montage(bw,'DisplayRange',[]);

%% オープン処理である領域以下の部分を削除
nhood = ones([7 7 3]);
bw = imopen(bw,nhood);
figure, montage(bw,'DisplayRange',[]);

%% 脳の部分のセグメンテーションを行います
% regionpropsで中心点と面積を確認します
L       = bwlabeln(bw);
stats   = regionprops('table',L,'Area')

%% 最も面積が大きい部分を選択
A       = stats.Area;
biggest = find(A == max(A));
mriAdjust(L ~= biggest) = 0;
figure, montage(mriAdjust,'DisplayRange',[]);

%% コントラスト調整
imA     = imadjust(mriAdjust(:, :, 30));
figure, montage(mriAdjust,'DisplayRange',[]);

%% 脳の部分のみ抽出して、表示
level = 65;
mriBrainPartition = uint8(zeros(size(mriAdjust)));    %0=outside brain (head/air)
mriBrainPartition(mriAdjust<level & mriAdjust>0) = 2; %2=gray matter
mriBrainPartition(mriAdjust>=level) = 3;              %3=white matter
figure,imshow(mriBrainPartition(:,:,30),[])

%% 脳部分のみの3次元表現
Ds = imresize(mriBrainPartition,0.25,'nearest');

% データの向きを修正
Ds = flip(Ds,1);
Ds = flip(Ds,2);
Ds = squeeze(Ds);
Ds = permute(Ds,[3 2 1]);

% ボクセルのスケーリング
voxel_size2 = voxel_size([1 3 2]).*[4 1 4];

%白い部分とグレーの部分のサーフェスを作成
white_vol = isosurface(Ds,2.5);
gray_vol  = isosurface(Ds,1.5);

% 可視化
h = figure('visible','off','outerposition',[0 0 800 600],'renderer','openGL');
patch(white_vol,'FaceColor','b','EdgeColor','none');
patch(gray_vol,'FaceColor','y' ,'EdgeColor','none',...
  'FaceAlpha',0.5);
view(45,15); daspect(1./voxel_size2); axis tight;axis off;
camlight; camlight(-80,-10); lighting phong;
movegui(h, 'center');
set(h,'visible','on');

%% 終了
