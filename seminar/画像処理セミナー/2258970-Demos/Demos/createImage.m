clear all, close all, clc;
%% 各種オブジェクトの定義
videoFReader = vision.VideoFileReader('ecolicells.avi');

%% 1フレーム毎に読み込む
frame = 0;
while ~isDone(videoFReader)
  videoFrame = videoFReader();
  frame = frame + 1;
  if frame == 11 %10Frame目を保存
      img = rgb2gray(videoFrame);
  end
end

%% 画像を保存
imwrite(img, 'frame10.png', 'png');
imwrite(imcrop(img,[145 50 273 270]), 'frame10s.png', 'png')

%% 定義したオブジェクトを解放
release(videoFReader);