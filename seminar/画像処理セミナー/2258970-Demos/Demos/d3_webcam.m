clear all, close all, clc;
imaqreset

%% �e��I�u�W�F�N�g�̒�`
%�������̃J�����ɂ��킹�ăv���p�e�B�̕ύX���K�v�ł��B
webCam = imaq.VideoDevice('winvideo', 1, 'RGB24_640x480')
videoPlayer  = vision.VideoPlayer;

%% 1���ǂݍ���ŉ���
videoFrame = webCam();
videoPlayer(videoFrame);

%% ���[�v���s
while isOpen(videoPlayer)
  videoFrame = webCam();
  videoPlayer(videoFrame);
end

%% ��`�����I�u�W�F�N�g�����
release(webCam);
release(videoPlayer);