function [xgnv,ygnv,Cnv,L0,Lc] = ClearanceValue(xg,yg,C,deltaL,newobstacle,Vx)
%计算clearancevalue 还没有考虑没有可行触须沿L0最长触须减速的情形！！！！！

%------------------------------------画出障碍物----------------------------------
% 
% plot(newobstacle(:,1),newobstacle(:,2),'.k','LineWidth',5,'MarkerSize',10) 
% 
% alpha=0:pi/40:2*pi;
% for i=1:size(newobstacle,1)
%     x=newobstacle(i,1)+2*cos(alpha);
%     y=newobstacle(i,2)+2*sin(alpha);
%     hold on
%     plot(x,y,'k','LineWidth',1)   
% end
% 
% hold on

%-----------------------------------------计算碰撞距离Lc------------------------------------
% Ls=10;%制动安全距离
Ls=6;
a=1.5;%制动减速度
Lc=Ls+Vx^2/(2*a);%可放到主函数中,这样就不需要输出Lc
%Lc=Ls+Vx^2/a

%-------------------------------------计算各触须预瞄距离Lck---------------------------------

% Lck=Lc;%存放各触须预瞄距离


%----------------------判断各触须上是否有碰撞点，如果有，算出沿触须到碰撞点的距离-----------

L0=[];%存放各条触须沿触须与第一个碰撞点的距离


for k=1:size(xg,2) %判断第k条触须
    
    flag=0;%标志符，flag为0表示触须上没有碰撞点，flag为1表示触须上有碰撞点，初始设为0
    
    for i=1:size(xg,1)%判断第k条触须上第i个点是否碰撞
        
        for j=1:size(newobstacle,1)%判断与第j个障碍物是否碰撞
            if (sqrt((xg(i,k)-newobstacle(j,1))^2+(yg(i,k)-newobstacle(j,2))^2)<4.5)%几何安全距离设为2，认为小于此距离即发生碰撞
                flag=1;%有碰撞点
                break
            end
        end
        
        if flag==1;
            break;
        end
        
    end
    
    if flag==1
        L0=[L0,deltaL*(i-1)];
    else
        L0=[L0,deltaL*i];%此时i=size(xg,1)
    end
%     hold on
%     plot(xg(:,k),yg(:,k),'k')%绘制所有触须
end


%----------------------------触须分类,与第一个障碍物碰撞距离小于Lc的触须是不可行触须-------------------------------------
%xgnv,ygnv存放可行的触须坐标
xgnv=xg;
ygnv=yg;
Cnv=C;

IndexOfUnnaviagable=[];%存放不可行的触须序号

for i=1:length(L0)
    if L0(i)<Lc
        IndexOfUnnaviagable=[IndexOfUnnaviagable,i];
    end
end

%除去不可行的触须
xgnv(:, IndexOfUnnaviagable)=[];
ygnv(:, IndexOfUnnaviagable)=[];
Cnv(:, IndexOfUnnaviagable)=[];
L0(:, IndexOfUnnaviagable)=[];

% %绘制可行触须
% for k=1:size(xgnv,2)
%     hold on
%     plot(xgnv(:,k),ygnv(:,k),'color',[0.69,0.88,0.90])
% end

end






