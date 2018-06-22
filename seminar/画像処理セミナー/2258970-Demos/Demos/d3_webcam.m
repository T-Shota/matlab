clear all, close all, clc;
imaqreset

%% 各種オブジェクトの定義
%お持ちのカメラにあわせてプロパティの変更が必要です。
webCam = imaq.VideoDevice('winvideo', 1, 'RGB24_640x480')
videoPlayer  = vision.VideoPlayer;

%% 1枚読み込んで可視化
videoFrame = webCam();
videoPlayer(videoFrame);

%% ループ実行
while isOpen(videoPlayer)
  videoFrame = webCam();
  videoPlayer(videoFrame);
end

%% 定義したオブジェクトを解放
release(webCam);
release(videoPlayer);