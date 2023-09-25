figure(1)
clc;
% close all;
disp('A Star Path Planning start!!')
% d=0.235; %两轮间距

% map.XYMAX=6; %代表我们要画一个地图的长和宽

% map.XMAX=6;
% map.YMAX=7;

% map.XMAX=120;
% map.YMAX=100;
map.XMAX=170;
map.YMAX=130;

% map.start=[1,1];  %起始点 注意必须在地图范围内
% map.goal=[6,5];  %目标点 注意必须在地图范围内

% map.start=[20,20];  
% map.goal=[110,80];  
map.start=[10,10];
map.goal=[160,120];


obstacle=GetBoundary(map);%得到边界数据
obstacle=[obstacle;10,40;10,50;20,40;20,50;30,40;30,50;40,40;40,50;40,10;50,10;40,20;50,20;160,40;160,50;170,40;170,50;...
    70,80;70,90;70,100;70,110;80,80;80,90;80,100;80,110;100,60;100,70;110,60;110,70;110,90;110,100;120,90;120,100;...
    20,110;20,120;30,110;30,120;80,30;80,40;90,30;90,40;100,30;100,40;10,70;10,80;20,70;20,80;...
    130,20;130,30;130,40;140,20;140,30;140,40;140,80;140,90;150,80;150,90;160,80;160,90;170,80;170,90];


%load('obstacle1.mat');
%画出网格线
PlotGrid(map);
hold on;

%画出障碍点
FillPlot(obstacle,'k');

path=AStar(obstacle,map);%A*算法

%画出路径
%
if length(path)>=1
    plot(path(:,1),path(:,2),'r','LineWidth',3,'HandleVisibility','off');hold on;
end
%}
% [ReferencePath,deltaP]  = interplotation(path);
%0.56