function  Vpath=PathValue(circlexgnv,circleygnv,ReferencePath,Lc,deltaLcircle,deltaP)
%计算各（可行）轨迹的PathValue
%先找到距离车辆当前位置最近的ReferencePath上的点作为参考轨迹上的起点，再找到其后碰撞距离Lc上的点

%目前只考虑了夹角alpha！！！！！


%if ReferencePath==[]，表明已到终点，在主程序中判断

%-----------------------------寻找参考轨迹起点-----------------------------------

% distance=sqrt((xgnv(1,1)-ReferencePath(1,1))^2+(ygnv(1,1)-ReferencePath(1,2))^2);%放入第一个点的distance
% 
% for i=2:size(ReferencePath,1)
%     distance=[distance,sqrt((xgnv(1,1)-ReferencePath(i,1))^2+(ygnv(1,1)-ReferencePath(i,2))^2)];
%     %xgnv(1,1),ygnv(1,1)分别是车辆当前位置
%     if distance(i)>distance(i-1)
%         break %distance应该是先减后增，一旦出现增，说明前一值为最小值
%     end
% end

CurrentPosition=[circlexgnv(1,1),circleygnv(1,1)];
StartIndexofRP= dsearchn(ReferencePath,CurrentPosition);
%StartIndexofRP为距离当前位置最近的ReferencePath上的点的序号

%-----------------------------寻找相应的Lc处的轨迹和触须终点-----------------------------------

LIndexRP=floor(Lc/deltaP);
if abs(LIndexRP*deltaP-Lc)>abs((LIndexRP+1)*deltaP-Lc)
    LIndexRP=LIndexRP+1;
end
EndIndexofRP=StartIndexofRP+LIndexRP;
%EndIndexofRP为相应的ReferencePath上的终点的序号

LcPath=(EndIndexofRP-StartIndexofRP)*deltaP;
%实际上的Lc，有插值误差

LIndexTentacle=floor(Lc/deltaLcircle);
if abs(LIndexTentacle*deltaLcircle-Lc)>abs((LIndexTentacle+1)*deltaLcircle-Lc)
    LIndexTentacle=LIndexTentacle+1;
end
StartIndexofTentacle=1;
EndIndexofTentacle=StartIndexofTentacle+LIndexTentacle;
%EndIndexofTentacle为相应的触须上的终点的序号

LcTentacle=(EndIndexofTentacle-StartIndexofTentacle)*deltaLcircle;
%实际上的Lc，有插值误差

%-----------------以下程序仅用于绘图示意各可行触须和参考路径上的点--------------------------------
% hold on
% plot(ReferencePath(:,1),ReferencePath(:,2),'r')
% hold on
% %绘出ReferencePath
% plot(circlexgnv(1,1),circleygnv(1,1),'o') 
% hold on
% %绘出当前位置
% plot(ReferencePath(StartIndexofRP,1),ReferencePath(StartIndexofRP,2),'o')
% hold on
% %绘出ReferencePath上与当前位置最近的点
% plot(ReferencePath(EndIndexofRP,1),ReferencePath(EndIndexofRP,2),'x')
% hold on
% %绘出ReferencePath上距离Lc的点
% 
% for i=1:size(circlexgnv,2)
%     plot(circlexgnv(EndIndexofTentacle,i),circleygnv(EndIndexofTentacle,i),'x')
%     hold on
% end
%绘出各触须上距离Lc的点

% hold on
% plot(ReferencePath(StartIndexofRP+1,1),ReferencePath(StartIndexofRP+1,2),'ro')
% axis equal 

%-----------------------------计算Vpath-----------------------------------
%斜率用i-1,i+1两点坐标来算

a=[];%存放各可行触须对应点的距离
alpha=[];%存放各可行触须对应点的夹角
SlopeOfRP=(ReferencePath(EndIndexofRP+1,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP+1,1)-ReferencePath(EndIndexofRP-1,1));
% 未考虑终点附近可能会超出的问题
% if SlopeOfRP<0
%     AngleOfRP=atan(SlopeOfRP)+pi;
% else
%     AngleOfRP=atan(SlopeOfRP);
% end

if ReferencePath(EndIndexofRP+1,1)==ReferencePath(EndIndexofRP-1,1)
    if ReferencePath(EndIndexofRP+1,2)>ReferencePath(EndIndexofRP-1,2)
        AngleOfRP=pi/2;
    else
        AngleOfRP=-pi/2;
    end
elseif ReferencePath(EndIndexofRP+1,1)>ReferencePath(EndIndexofRP-1,1)
    SlopeOfRP=(ReferencePath(EndIndexofRP+1,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP+1,1)-ReferencePath(EndIndexofRP-1,1));
    AngleOfRP=atan(SlopeOfRP);
else
    SlopeOfRP=(ReferencePath(EndIndexofRP+1,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP+1,1)-ReferencePath(EndIndexofRP-1,1));
    if SlopeOfRP>0
        AngleOfRP=atan(SlopeOfRP)-pi;            
    else
        AngleOfRP=atan(SlopeOfRP)+pi;
    end
end


for i=1:size(circlexgnv,2)
    hold on 
%     plot(xgnv(:,i),ygnv(:,i),'y')
    a=[a,sqrt((circlexgnv(EndIndexofTentacle,i)-ReferencePath(EndIndexofRP,1))^2+(circleygnv(EndIndexofTentacle,i)-ReferencePath(EndIndexofRP,2))^2)];
   
%     SlopeOfTentacle=(ygnv(EndIndexofTentacle+1,i)-ygnv(EndIndexofTentacle-1,i))/(xgnv(EndIndexofTentacle+1,i)-xgnv(EndIndexofTentacle-1,i));
%     if SlopeOfTentacle<0
%         AngleOfTentacle=atan(SlopeOfTentacle)+pi;
%     else
%         AngleOfTentacle=atan(SlopeOfTentacle); 
%     end
%     alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)];

    if circlexgnv(EndIndexofTentacle+1,i)==circlexgnv(EndIndexofTentacle-1,i)
        if circleygnv(EndIndexofTentacle+1,i)>circleygnv(EndIndexofTentacle-1,i)
            AngleOfTentacle=pi/2;
        else
            AngleOfTentacle=-pi/2;
        end
    elseif circlexgnv(EndIndexofTentacle+1,i)>circlexgnv(EndIndexofTentacle-1,i)
        SlopeOfTentacle=((circleygnv(EndIndexofTentacle+1,i)-circleygnv(EndIndexofTentacle-1,i)))/((circlexgnv(EndIndexofTentacle+1,i)-circlexgnv(EndIndexofTentacle-1,i)));
        AngleOfTentacle=atan(SlopeOfTentacle); 
    else
        SlopeOfTentacle=((circleygnv(EndIndexofTentacle+1,i)-circleygnv(EndIndexofTentacle-1,i)))/((circlexgnv(EndIndexofTentacle+1,i)-circlexgnv(EndIndexofTentacle-1,i)));
        if SlopeOfTentacle>0
            AngleOfTentacle=atan(SlopeOfTentacle)-pi;
        else
            AngleOfTentacle=atan(SlopeOfTentacle)+pi;
        end
    end
    alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)];
    
end

% Calpha=3;%常数，可调节
Calpha=0;
vdist=a+Calpha*alpha;
% vdist=Calpha*alpha;
vmin=min(vdist);
vmax=max(vdist);

Vpath=(vdist-vmin)/(vmax-vmin);


end

