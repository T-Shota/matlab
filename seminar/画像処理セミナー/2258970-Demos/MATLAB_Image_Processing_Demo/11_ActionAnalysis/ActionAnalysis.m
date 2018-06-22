%% ステレオビジョンによる行動解析
clc; clear; close all;

%% 動画ファイルから画像を取り込むためのシステムオブジェクトの生成
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

str = {'雑貨','コーナー',[num2str(time1),' s']};
h1 = text(1,2.8,str,'FontSize',20);
str1 = {'本','コーナー',[num2str(time2),' s']};
h2 = text(1,4.2,str1,'FontSize',20);
str2 = {'ドリンク','コーナー',[num2str(time3),' s']};
h3 = text(1,5.7,str2,'FontSize',20);
color = [];

for n = 1:1200
    % カメラから1画面取込み
    I1 = step(vidReader1);
    
    step(viewer, I1);
    
    % 距離判定
    if 2 < z(i) && z(i) < 3.5
        % 雑貨
        time1 = time1+1/30;
    elseif z(i) < 5
        % 本
        time2 = time2+1/30;
    elseif z(i) < 6.5
        % 飲み物
        time3 = time3+1/30;
    end
    
    % プロットを表示
    addpoints(h,xmed(i),z(i));
    h.MarkerFaceColor;
    
    % 停留時間を表示
    h1.String = {'雑貨','コーナー',[num2str(time1),' s']};
    h2.String = {'本','コーナー',[num2str(time2),' s']};
    h3.String = {'ドリンク','コーナー',[num2str(time3),' s']};
    
    drawnow limitrate;
    i = i+1;
end

%%

release(vidReader1);
release(viewer);