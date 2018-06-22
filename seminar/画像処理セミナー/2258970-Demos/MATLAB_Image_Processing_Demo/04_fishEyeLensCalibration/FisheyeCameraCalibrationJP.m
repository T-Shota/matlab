%% 魚眼カメラのキャリブレーション

%% 画像データの収集
images = imageDatastore(fullfile(toolboxdir('vision'),'visiondata',...
        'calibration','gopro'));
imageFileNames = images.Files;
figure, montage(imageFileNames,'Size',[3 4]);

%% キャリブレーションパターンの検出
[imagePoints,boardSize] = detectCheckerboardPoints(imageFileNames);

%% キャリブレーションボードのワールド座標系定義
squareSize = 29; % millimeters
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

%% 魚眼レンズのパラメーター推定
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
params = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);

%% キャリブレーション精度の可視化
figure
showReprojectionErrors(params);

%% カメラの外部パラメータを可視化
figure
showExtrinsics(params);
drawnow

%% 検出点と再投影点の可視化
figure 
imshow(I); 
hold on
plot(imagePoints(:,1,1),imagePoints(:,2,1),'go');
plot(params.ReprojectedPoints(:,1,1),params.ReprojectedPoints(:,2,1),'r+');
legend('Detected Points','Reprojected Points');
hold off

%% ひずみ除去
J1 = undistortFisheyeImage(I,params.Intrinsics);
figure
imshowpair(I,J1,'montage')
title('Original Image (left) vs. Corrected Image (right)')

J2 = undistortFisheyeImage(I,params.Intrinsics,'OutputView','full');
figure
imshow(J2)
title('Full Output View')
