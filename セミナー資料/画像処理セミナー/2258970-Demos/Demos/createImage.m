clear all, close all, clc;
%% �e��I�u�W�F�N�g�̒�`
videoFReader = vision.VideoFileReader('ecolicells.avi');

%% 1�t���[�����ɓǂݍ���
frame = 0;
while ~isDone(videoFReader)
  videoFrame = videoFReader();
  frame = frame + 1;
  if frame == 11 %10Frame�ڂ�ۑ�
      img = rgb2gray(videoFrame);
  end
end

%% �摜��ۑ�
imwrite(img, 'frame10.png', 'png');
imwrite(imcrop(img,[145 50 273 270]), 'frame10s.png', 'png')

%% ��`�����I�u�W�F�N�g�����
release(videoFReader);