clear all, close all, clc;
%% �e��I�u�W�F�N�g�̒�`
videoFReader = vision.VideoFileReader('ecolicells.avi');
videoPlayer  = vision.VideoPlayer;

%% 1�t���[�����ɓǂݍ���ŉ���
while ~isDone(videoFReader)
  videoFrame = videoFReader();
  videoPlayer(videoFrame);
end

%% ��`�����I�u�W�F�N�g�����
release(videoPlayer);
release(videoFReader);