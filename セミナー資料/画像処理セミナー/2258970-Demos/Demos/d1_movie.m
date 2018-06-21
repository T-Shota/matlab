clear all, close all, clc;
%% 各種オブジェクトの定義
videoFReader = vision.VideoFileReader('ecolicells.avi');
videoPlayer  = vision.VideoPlayer;

%% 1フレーム毎に読み込んで可視化
while ~isDone(videoFReader)
  videoFrame = videoFReader();
  videoPlayer(videoFrame);
end

%% 定義したオブジェクトを解放
release(videoPlayer);
release(videoFReader);