function  [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP)

%------------------------寻找触须起点及其对应ReferencePath上的点------------------------------

CurrentPosition=[xgnv(1,1),ygnv(1,1)];
StartIndexofRP= dsearchn(ReferencePath,CurrentPosition);
%StartIndexofRP为距离当前位置最近的ReferencePath上的点的序号

%------------------------------------判断剩下RP是否够长----------------------------------------

for k=1:size(xgnv,2)
    if (size(ReferencePath,1)-StartIndexofRP)*deltaP<Lcknv(k)
        Lcknv(k)=(size(ReferencePath,1)-StartIndexofRP)*deltaP;  
    end
end

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
% 绘出ReferencePath
% plot(xgnv(1,1),ygnv(1,1),'o','color',[0,0.45,0.74],'LineWidth',2) 
% hold on
% % 绘出当前位置
% plot(ReferencePath(StartIndexofRP,1),ReferencePath(StartIndexofRP,2),'o','color',[0,0.45,0.74],'LineWidth',2)
% hold on
% % 绘出ReferencePath上与当前位置最近的点

% plot(ReferencePath(EndIndexofRP,1),ReferencePath(EndIndexofRP,2),'xr','LineWidth',2,'MarkerSize',8)
% hold on
% 绘出ReferencePath上距离Lck的点

% plot(ReferencePath(EndIndexofRP(31),1),ReferencePath(EndIndexofRP(31),2),'+','color',[0,0.45,0.74],'LineWidth',2)
% hold on
% 绘出ReferencePath上距离Lck的点

% % 
% for k=1:size(xgnv,2)
%     plot(xgnv(EndIndexofTentacle(k),k),ygnv(EndIndexofTentacle(k),k),'x','color',[0,0.45,0.74],'LineWidth',2,'MarkerSize',8)
%     hold on
% end
% %绘出各触须上距离Lck的点


% plot(xgnv(EndIndexofTentacle(31),31),ygnv(EndIndexofTentacle(31),31),'+','color',[0,0.45,0.74],'LineWidth',2)

%绘出各触须上距离Lck的点

end

