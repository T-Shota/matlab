%% SfMによるカメラ位置推定と3次元点群構築

%% 初期化
clc;clear;close all;imtool close all; rng('default');

%% パラメータ
minNumPts = 1000;
minDisparity = 20;

%% 各オブジェクトの作成

% 動画読み込みオブジェクトの作成
videoReader = vision.VideoFileReader('cube.mp4','VideoOutputDataType','uint8');
% ビデオプレイヤーオブジェクトの作成
videoPlayer = vision.DeployableVideoPlayer;
% ポイントトラッカーオブジェクトの作成
tracker = vision.PointTracker('MaxBidirectionalError', 0.1,'NumPyramidLevels',3);

%% ストップボタンの作成
a = true;
sz = get(0,'ScreenSize');
figure('MenuBar','none','Toolbar','none','Position',[20 sz(4)-100 100 70])
uicontrol('Style', 'pushbutton', 'String', 'Stop',...
        'Position', [20 20 100 40],...
        'Callback', 'a=false;');
    
%% カメラのパラメータ読み込み
load('cameraParams2');   % カメラパラメータの読込み

%% 1枚目の画像を処理
% 1枚目の画像のレンズ歪除去･表示
Irgb = step(videoReader);

Ic = undistortImage(Irgb, cameraParams);

% グレイスケール変換
I = rgb2gray(Ic);

% 周囲30ピクセルは除いて、特徴点抽出（画像エッジでの特徴点を排除）
border = 30;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];

% 特徴点検出：大局的特徴を取得する為に、NumOctavesを大きく設定
initPoints   = detectSURFFeatures(I, 'NumOctaves', 5, 'MetricThreshold', 1, 'ROI', roi);

% 検出されたコーナー点で、ポイントトラッカーを初期化
initPoints = initPoints.Location;

% 1枚目の画像のデータ(特徴点座標、カメラ位置･姿勢)を、viewID=1として登録
% このときのカメラ位置(光学中心)がWorld座標の原点
% カメラの向きがWorld座標系のZ軸の正方向
vSet = viewSet       % 空のviewSetオブジェクトを生成
viewID = 1;
vSet = addView(vSet, viewID, 'Points', initPoints, 'Orientation', eye(3),...
    'Location', [0 0 0]);
vSet.Views     % ViewId=1 が登録されている

initialize(tracker, initPoints, Ic);

images{1} = Ic;
prevI = Ic;
oldPoints = initPoints;

%% 残りの画像のデータも、ループ処理でvSetオブジェクトへ登録
while a && viewID < 5
    Irgb = step(videoReader);

    % レンズ歪除去
    Ic = undistortImage(Irgb, cameraParams);
    
    % 表示用画像
    Iout = Ic;
        
    % グレースケールへ変換
    I = rgb2gray(Ic);
    
    % 現画像上の対応点を検出
    [currPoints, validIdx] = step(tracker, Ic);
    
    % 対応関係が見つかった特徴点を抽出
    matchedPoints1 = oldPoints(validIdx,:);
    matchedPoints2 = currPoints(validIdx,:);
    
    % 画像表示
    Iout = insertShape(Iout,'line',[matchedPoints1, matchedPoints2]);
    Iout = insertText(Iout,[0 0],num2str(size(matchedPoints2,1),'# of tracked: %d'));
    Iout = insertText(Iout,[0 30],num2str(viewID,'View ID: %d'));
    
    % 平均的な移動量
    d = mean(sqrt(sum((matchedPoints1-matchedPoints2).^2,2)));
    
    % 移動量がしきい値を超えたら基礎行列推定
    if d > minDisparity
        viewID = viewID+1;
        images{viewID} = Ic;
        % 対応点の情報から
        % 一つ前のカメラ位置･姿勢に対する位置･姿勢を推定（距離は1と仮定）
        % 後のBundle Adjustmentで、距離の誤差等は補正される
        %for i = 1:100
            % 基礎行列(2つの画像上の点の対応関係)の推定
            [fMatrix, inlierIdx] = estimateFundamentalMatrix( ...
                matchedPoints1, matchedPoints2, 'Method','MSAC', 'NumTrials',1000, 'DistanceThreshold',4);
            
            % エピポーラ拘束を満たさないもの(誤対応点)の除去
            inlierPoints1 = matchedPoints1(inlierIdx, :);
            inlierPoints2 = matchedPoints2(inlierIdx, :);
            
            % 基礎行列から、一つ前のカメラ位置･姿勢に対する、現カメラ位置・姿勢を推定
            [relativeOrient, relativeLoc, validPointFraction] = ...
                cameraPose(fMatrix, cameraParams, inlierPoints1, inlierPoints2);
            
            % 有効な点の割合が高くなるまで繰り返し、基礎行列の推定を行う
            %if validPointFraction > .8
            %   break;
            %elseif i == 100;
            %   % 100回反復してもvalidPointFractionが低い場合は、エラーにする
            %   error('Unable to compute the Fundamental matrix');
            %end
        %end
        
        % 1つ前のカメラ位置･姿勢を取得 (addView でセットしたもの)
        prevPose = poses(vSet, viewID-1);
        prevOrientation = prevPose.Orientation{1};
        prevLocation    = prevPose.Location{1};
        
        % 一番目のViewに対する、現カメラ位置･姿勢を求める.
        orientation = relativeOrient * prevOrientation;
        location    = prevLocation + relativeLoc * prevOrientation;
        
        % 現在追跡している点数
        numPts = size(matchedPoints2,1);
        
        if numPts < minNumPts
            % 追跡している点数が最大設定値よりも少ない場合
            
            % 現在追跡している点を除外するためのマスク作成
            mask = ones(size(I));
            mask = insertShape(mask,'FilledCircle',...
                [matchedPoints2 4*ones(size(matchedPoints2,1),1)],...
                'Opacity',1,'Color','black');
            mask = mask > 0;
            
            % 新たに追跡する点を検出
            newPoints = detectSURFFeatures(I, 'NumOctaves', 5, 'MetricThreshold', 10, 'ROI', roi);
            newPoints = newPoints.Location;
            
            % 検出した点が既存の追跡点に近くないか確認
            notTrackedIdx = mask(sub2ind(size(mask),round(newPoints(:,2)),round(newPoints(:,1))));
            
            % 近くないものだけを抽出
            newPoints = newPoints(notTrackedIdx,:);
            numNewPts = size(newPoints,1);
            
            % 追跡対象の点を追加
            trackedPoints = [matchedPoints2; newPoints];
        else
            trackedPoints = matchedPoints2;
        end
        
        % 特徴点の座標、カメラ位置･姿勢を、vSetへ登録
        vSet = addView(vSet, viewID, 'Points', trackedPoints, 'Orientation', orientation, ...
            'Location', location);
        
        % 一つ前の画像との特徴点の対応関係を、vSetへ登録
        matches = [find(validIdx) (1:sum(validIdx))'];
        vSet = addConnection(vSet, viewID-1, viewID, 'Matches', matches);
        
        % トラック：複数Viewにまたがる、点の全対応関係情報（一部のトラックは、全Viewにまたがっている）
        tracks1 = findTracks(vSet);  % 現在までの全View間のトラック情報抽出
        
        % 現在までの全Viewの、カメラ位置･姿勢を取得
        camPoses1 = poses(vSet);
        
        % 複数画像上の点対応関係から、各点の3次元位置を推定
        xyzPoints1 = triangulateMultiview(tracks1, camPoses1, cameraParams);
        
        % バンドル調整で点群の位置とカメラ位置･姿勢を最適化する
        [xyzPoints1, camPoses1, reprojectionErrors] = bundleAdjustment(xyzPoints1, ...
           tracks1, camPoses1, cameraParams, 'FixedViewId', 1, ...
           'PointsUndistorted', true);
        
        % バンドル調整で微修正したカメラ位置･姿勢を登録
        vSet = updateView(vSet, camPoses1);       % テーブル：camPoses1 の情報（ViewID, Orientation, Location）でアップデート

        % 追跡点を再設定
        setPoints(tracker, trackedPoints);
        
        % 保存
        prevI = I;
        oldPoints = trackedPoints;
    end
    
    % 表示
    step(videoPlayer,Iout);
    
    drawnow limitrate;
    
    if isDone(videoReader)
        break;
    end

end
release(videoReader);

%% トラック：View間の点の対応関係情報、一部のトラックは、全Viewにまたがっている
tracks = findTracks(vSet);      % 全View間のトラック情報の抽出
idx = arrayfun(@(x) numel(x.ViewIds),tracks) > 2; % 3フレーム以上トラッキングしている点
tracks2 = tracks(idx);

% ポイントクラウドの色情報を取得
color = zeros(numel(tracks2),3,'uint8');
for k = 1:numel(tracks2)
    Itracked = images{tracks2(k).ViewIds(1)};
    x = round(tracks2(k).Points(1,1));
    y = round(tracks2(k).Points(1,2));
    x = min(size(Itracked,2),x);
    y = min(size(Itracked,1),y);
    color(k,:) = Itracked(y,x,:);
end

%% 密な3次元再構成結果の表示(バンドル調整前)
% 全Viewの、カメラ位置･姿勢を取得
camPoses2 = poses(vSet);

% 複数画像上の対応点組を用い、各点の3次元ワールド座標を計算
xyzPoints2 = triangulateMultiview(tracks2, camPoses2, cameraParams);

figure; plotCamera(camPoses2, 'Size', 1);   % カメラ位置･姿勢をプロット
hold on;
ptCloud = pointCloud(xyzPoints2, 'Color', color);
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down','MarkerSize',20);

%xlim([-5 100]); ylim([-50 50]); zlim([0 100]);
xlabel('X');ylabel('Y');zlabel('Z');camorbit(0, -30); grid on; box on;
title('3次元再構成結果(バンドル調整前)');

%% 密な3次元再構成結果の表示(バンドル調整後)

% 全Viewの、3次元点群座標とカメラ位置･姿勢をバンドル調整で微修正
[xyzPoints3, camPoses3, reprojectionErrors] = bundleAdjustment(...
    xyzPoints2, tracks2, camPoses2, cameraParams, 'FixedViewId', 1, 'PointsUndistorted', true);

figure; plotCamera(camPoses3, 'Size', 1);   % カメラ位置･姿勢をプロット
hold on;

goodIdx = (reprojectionErrors < 5);  % エラー値が大きい点を除去

ptCloud = pointCloud(xyzPoints3(goodIdx, :), 'Color', color(goodIdx,:));
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down','MarkerSize',20);

%xlim([-50 100]); ylim([-50 50]); zlim([0 100]);
xlabel('X');ylabel('Y');zlabel('Z');camorbit(0, -30); grid on; box on;
title('3次元再構成結果(バンドル調整後)');

%% 3次元点群から平面を推定
pc = ptCloud;
maxDistance = 1; % 平面内の点間の最大距離を指定
referenceVector = [0,1,1]; % 検出する平面の法線ベクトルを指定
maxAngularDistance = 5; % 法線ベクトルから20度以内の平面を見つける
[planeMdl1, ~, outlierIndices] = pcfitplane(pc, maxDistance);
remainPtCloud = select(pc, outlierIndices);
hold on;
h1 = plot(planeMdl1);
h1.FaceAlpha = 0.5;
shg;

%% 平面の方程式の係数を使って傾き補正
n_a = planeMdl1.Normal; % 平面までの法線ベクトル
n_b = [0 0 1]; % そろえたい方向ベクトル(今回はZ軸)
n_c = cross(n_b,n_a); % 外積
v = n_c/norm(n_c); % 回転軸(単位ベクトル)
cos_theta = dot(n_a,n_b)/(norm(n_a)*norm(n_b)); % 回転角
% ロドリゲスの公式で回転行列を計算
R = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
sin_theta = sqrt(1-cos_theta^2);
M = eye(3)+sin_theta*R+(1-cos_theta)*R^2; % 回転行列

% 平面の平行移動量抽出(原点から平面までの垂線の一ベクトル)
s = -planeMdl1.Parameters(4)/(planeMdl1.Parameters(1:3)*planeMdl1.Normal');
p_a = s*planeMdl1.Normal; % 原点から平面までのベクトル(垂線)
p_b = [0 0 0]; % 原点(平行移動先は原点)

% 平行移動
tform = affine3d([eye(3) zeros(3,1); p_b-p_a 1]);
tform.T
ptCloudScene3 = pctransform(pc, tform);
figure, pcshow(ptCloudScene3, 'VerticalAxis','Z', 'VerticalAxisDir', 'Down','MarkerSize',20)
title('Shifted scene')
xlabel('X (mm)'), ylabel('Y (mm)'), zlabel('Z (mm)')

% 回転
tform2 = affine3d([M zeros(3,1); 0 0 0 1]);
tform2.T
ptCloudScene3 = pctransform(ptCloudScene3, tform2);
figure, pcshow(ptCloudScene3, 'VerticalAxis','Z', 'VerticalAxisDir', 'Down','MarkerSize',20)
title('Shifted and rotated scene')
xlabel('X (mm)'), ylabel('Y (mm)'), zlabel('Z (mm)')
%axis([-50 50 -60 0 -20 0]);

%% 物体位置を検出(ROIの抽出)
% 彩度が高い領域をHSVで抽出
hsv = rgb2hsv(im2double(ptCloudScene3.Color));
channel1Min = 0.953;
channel1Max = 0.046;
channel2Min = 0.614;
channel2Max = 1.000;
idx = ( (hsv(:,1) >= channel1Min) | (hsv(:,1) <= channel1Max) ) & ...
    (hsv(:,2) >= channel2Min ) & (hsv(:,2) <= channel2Max);
redPts = ptCloudScene3.Location(idx,:);
centroid = mean(redPts); % 重心位置計算
sigma = sqrt(mean((redPts-repmat(centroid,[size(redPts,1) 1])).^2)); % 2次モーメント計算

% +/- 4sigmaの領域の可視化
vert = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1];
vert = vert.*repmat(sigma,[8 1])*4+repmat(centroid,[8 1]);
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
hold on;
patch('Vertices',vert,'Faces',fac,...
      'FaceVertexCData',hsv(6),'FaceColor','flat','FaceAlpha',.3);
shg;

%% 検出された物体の近傍を抽出
roi_view = [centroid-4*sigma; centroid+4*sigma];
roi_view = reshape(roi_view,1,[]);
indices = findPointsInROI(ptCloudScene3, roi_view);
ptCloudB = select(ptCloudScene3,indices);
hFig = figure;
ptCloudB = pcdenoise(ptCloudB);
if centroid(3) < 0
    tform3 = affine3d([1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]);
    ptCloudB = pctransform(ptCloudB, tform3);
end
pcshow(ptCloudB, 'VerticalAxis','Z','MarkerSize',30)
xlabel('X'), ylabel('Y'), zlabel('Z')

%% メッシュ構築
tri = delaunay(ptCloudB.Location(:,1),ptCloudB.Location(:,2));
figure, hh = trisurf(tri,ptCloudB.Location(:,1),ptCloudB.Location(:,2),ptCloudB.Location(:,3));
set(hh,...
       'FaceVertexCData',ptCloudB.Color,...
       'LineStyle','none',...
       'FaceColor','interp');
view(-30,45);
axis equal;

%% Copyright 2016 The MathWorks, Inc. 