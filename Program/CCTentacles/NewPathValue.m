function  Vpath=NewPathValue(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP)

%------------------------寻找触须起点及其对应ReferencePath上的点------------------------------

CurrentPosition=[xgnv(1,1),ygnv(1,1)];
StartIndexofRP= dsearchn(ReferencePath,CurrentPosition);
%StartIndexofRP为距离当前位置最近的ReferencePath上的点的序号

%-----------------------------寻找相应的Lcknv处的轨迹和触须终点-----------------------------------
LIndexRP=[];
for k=1:size(xgnv,2)
    LIndexRPk=floor(Lcknv(k)/deltaP);
    if abs(LIndexRPk*deltaP-Lcknv(k))>abs((LIndexRPk+1)*deltaP-Lcknv(k))
        LIndexRPk=LIndexRPk+1;
    end
    LIndexRP=[LIndexRP,LIndexRPk];
end
EndIndexofRP=StartIndexofRP+LIndexRP;
%EndIndexofRP为相应的ReferencePath上的终点的序号
LcPath=(EndIndexofRP-StartIndexofRP)*deltaP;
%实际上的Lc，有插值误差

LIndexTentacle=[];
for k=1:size(xgnv,2)
    LIndexTentaclek=floor(Lcknv(k)/deltaL);
    if abs(LIndexTentaclek*deltaL-Lcknv(k))>abs((LIndexTentaclek+1)*deltaL-Lcknv(k))
        LIndexTentaclek=LIndexTentaclek+1;
    end
    LIndexTentacle=[LIndexTentacle,LIndexTentaclek];
end
StartIndexofTentacle=1;
EndIndexofTentacle=StartIndexofTentacle+LIndexTentacle;
%EndIndexofTentacle为相应的触须上的终点的序号
LcTentacle=(EndIndexofTentacle-StartIndexofTentacle)*deltaL;
%实际上的Lc，有插值误差


%-----------------以下程序仅用于绘图示意各可行触须和参考路径上的点--------------------------------
% plot(ReferencePath(:,1),ReferencePath(:,2),'r')
% hold on
%绘出ReferencePath
% plot(xgnv(1,1),ygnv(1,1),'o','color',[0,0.45,0.74],'LineWidth',2) 
% hold on
% % 绘出当前位置
% plot(ReferencePath(StartIndexofRP,1),ReferencePath(StartIndexofRP,2),'o')
% hold on
% % 绘出ReferencePath上与当前位置最近的点
% % 
% % %
% plot(ReferencePath(EndIndexofRP,1),ReferencePath(EndIndexofRP,2),'x')
% hold on
%绘出ReferencePath上距离Lck的点
% % 
% for k=1:size(xgnv,2)
%     plot(xgnv(EndIndexofTentacle(k),k),ygnv(EndIndexofTentacle(k),k),'x')
%     hold on
% end
%绘出各触须上距离Lck的点


%-----------------------------计算Vpath-----------------------------------
%斜率用i-1,i+1两点坐标来算

a=[];%存放各可行触须对应点的距离
alpha=[];%存放各可行触须对应点的夹角

for k=1:size(xgnv,2)
    %求出ReferencePath上Lcknv处与x轴正方向夹角
    if ReferencePath(EndIndexofRP(k),1)==ReferencePath(EndIndexofRP(k)-1,1)
        if ReferencePath(EndIndexofRP(k),2)>ReferencePath(EndIndexofRP(k)-1,2)
            AngleOfRP=pi/2;
        else
            AngleOfRP=-pi/2;
        end
    elseif ReferencePath(EndIndexofRP(k),1)>ReferencePath(EndIndexofRP(k)-1,1)
        SlopeOfRP=(ReferencePath(EndIndexofRP(k),2)-ReferencePath(EndIndexofRP(k)-1,2))/(ReferencePath(EndIndexofRP(k),1)-ReferencePath(EndIndexofRP(k)-1,1));
        AngleOfRP=atan(SlopeOfRP);
    else
        SlopeOfRP=(ReferencePath(EndIndexofRP(k),2)-ReferencePath(EndIndexofRP(k)-1,2))/(ReferencePath(EndIndexofRP(k),1)-ReferencePath(EndIndexofRP(k)-1,1));
        if SlopeOfRP>0
            AngleOfRP=atan(SlopeOfRP)-pi;
        else
            AngleOfRP=atan(SlopeOfRP)+pi;
        end
    end
    
    %求出触须上Lcknv处与x轴正方向夹角
    if xgnv(EndIndexofTentacle(k),k)==xgnv(EndIndexofTentacle(k)-1,k)
        if ygnv(EndIndexofTentacle(k),k)>ygnv(EndIndexofTentacle(k)-1,k)
            AngleOfTentacle=pi/2;
        else
            AngleOfTentacle=-pi/2;
        end
    elseif xgnv(EndIndexofTentacle(k),k)>xgnv(EndIndexofTentacle(k)-1,k)
        SlopeOfTentacle=((ygnv(EndIndexofTentacle(k),k)-ygnv(EndIndexofTentacle(k)-1,k)))/((xgnv(EndIndexofTentacle(k),k)-xgnv(EndIndexofTentacle(k)-1,k)));
        AngleOfTentacle=atan(SlopeOfTentacle); 
    else
        SlopeOfTentacle=((ygnv(EndIndexofTentacle(k),k)-ygnv(EndIndexofTentacle(k)-1,k)))/((xgnv(EndIndexofTentacle(k),k)-xgnv(EndIndexofTentacle(k)-1,k)));
        if SlopeOfTentacle>0
            AngleOfTentacle=atan(SlopeOfTentacle)-pi;
        else
            AngleOfTentacle=atan(SlopeOfTentacle)+pi;
        end
    end
    
    %求出两者夹角
    alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)];
    
    %求出两者距离
    a=[a,sqrt((xgnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),1))^2+(ygnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),2))^2)];
    
end


% Calpha=3;%常数，可调节
Calpha=0;
vdist=a+Calpha*alpha;
% vdist=Calpha*alpha;
vmin=min(vdist);
vmax=max(vdist);

Vpath=(vdist-vmin)/(vmax-vmin);


end

