%% �X�e���I�r�W�����ɂ��s�����
clc; clear; close all;

%% ����t�@�C������摜����荞�ނ��߂̃V�X�e���I�u�W�F�N�g�̐���
load('plotdata2.mat');
vidReader1 = vision.VideoFileReader('myFile3_20160921_1.mp4');
viewer = vision.DeployableVideoPlayer('Location',[0 0]);
time1 = 0; time2 = 0; time3 = 0;

i = 1;
figure(1),
ax = axes;
h = animatedline('LineStyle','none',...
    'Marker','o','MarkerFaceColor','auto');
grid on;
hold on;
xlim([-2.5 2.5]); ylim([2 6.5]);
line([-2.5 2.5;-2.5 2.5]',[3.5 3.5;5 5]');

str = {'�G��','�R�[�i�[',[num2str(time1),' s']};
h1 = text(1,2.8,str,'FontSize',20);
str1 = {'�{','�R�[�i�[',[num2str(time2),' s']};
h2 = text(1,4.2,str1,'FontSize',20);
str2 = {'�h�����N','�R�[�i�[',[num2str(time3),' s']};
h3 = text(1,5.7,str2,'FontSize',20);
color = [];

for n = 1:1200
    % �J��������1��ʎ捞��
    I1 = step(vidReader1);
    
    step(viewer, I1);
    
    % ��������
    if 2 < z(i) && z(i) < 3.5
        % �G��
        time1 = time1+1/30;
    elseif z(i) < 5
        % �{
        time2 = time2+1/30;
    elseif z(i) < 6.5
        % ���ݕ�
        time3 = time3+1/30;
    end
    
    % �v���b�g��\��
    addpoints(h,xmed(i),z(i));
    h.MarkerFaceColor;
    
    % �◯���Ԃ�\��
    h1.String = {'�G��','�R�[�i�[',[num2str(time1),' s']};
    h2.String = {'�{','�R�[�i�[',[num2str(time2),' s']};
    h3.String = {'�h�����N','�R�[�i�[',[num2str(time3),' s']};
    
    drawnow limitrate;
    i = i+1;
end

%%

release(vidReader1);
release(viewer);