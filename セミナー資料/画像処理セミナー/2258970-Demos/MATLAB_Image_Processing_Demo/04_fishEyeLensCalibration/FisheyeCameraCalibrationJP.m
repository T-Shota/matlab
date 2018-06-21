%% ����J�����̃L�����u���[�V����

%% �摜�f�[�^�̎��W
images = imageDatastore(fullfile(toolboxdir('vision'),'visiondata',...
        'calibration','gopro'));
imageFileNames = images.Files;
figure, montage(imageFileNames,'Size',[3 4]);

%% �L�����u���[�V�����p�^�[���̌��o
[imagePoints,boardSize] = detectCheckerboardPoints(imageFileNames);

%% �L�����u���[�V�����{�[�h�̃��[���h���W�n��`
squareSize = 29; % millimeters
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

%% ���჌���Y�̃p�����[�^�[����
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
params = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);

%% �L�����u���[�V�������x�̉���
figure
showReprojectionErrors(params);

%% �J�����̊O���p�����[�^������
figure
showExtrinsics(params);
drawnow

%% ���o�_�ƍē��e�_�̉���
figure 
imshow(I); 
hold on
plot(imagePoints(:,1,1),imagePoints(:,2,1),'go');
plot(params.ReprojectedPoints(:,1,1),params.ReprojectedPoints(:,2,1),'r+');
legend('Detected Points','Reprojected Points');
hold off

%% �Ђ��ݏ���
J1 = undistortFisheyeImage(I,params.Intrinsics);
figure
imshowpair(I,J1,'montage')
title('Original Image (left) vs. Corrected Image (right)')

J2 = undistortFisheyeImage(I,params.Intrinsics,'OutputView','full');
figure
imshow(J2)
title('Full Output View')
