% clear,clc
% figure

% 参考路径
% path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%转弯情形
% path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%避障/减速情形
% path=[130,100;120,100;110,100;100,100;90,100;80,100;70,100;60,100;50,100;40,100;40,90;40,80;40,70;40,60;40,50;40,40;30,40;20,40;20,30;20,20];%A*模拟
[ReferencePath,deltaP]  = interplotation(path);

% 绘出ReferencePath
% figure(1)
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5)
% hold on

% obstacle=[30,20.5];%避障情形
% obstacle=[0,100];%转弯情形
% obstacle=[20,50;30,50;40,30;50,50;50,60;90,100.5];%A*模拟
newobstacle=[52,70;110,119.5];

%局部路径规划
Vx=3; L=2.7; amax=4; x0=10; y0=10; fai=pi/2; rou0=0; CofBestTentacle=0; CurrentStotal=0; 
%初始车辆状态； CofBestTentacle是当前周期最优触须的曲率变化率，初值设为0
% deltamax=pi/6;%前轮最大转角
deltamax=35/180*pi;

deltaT=0.1; 
% deltaT=0.1;%规划周期 单位为s


CurvatureOfTotalTentacle=rou0;%整个路径各点曲率,放入初值rou0
Stotal=0;%整个路径各点总路程，放入初值0
COfTotalTentacle=[];%整个路径各段曲率变化率
TotalPlanningPath=[x0,y0];%存放整个规划路径各点坐标,先存入起点坐标
count=0;%当前处于第count个规划周期


while(1)
    
    if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
        %已到终点区域
        break;
    else
        %开始规划
        
        %生成触须
        [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0); 

        %选出可行触须，并计算其Vclearance
        [xgnv,ygnv,Cnv,L0,Vclearance,Lcknv] = ClearanceValue(xg,yg,C,deltaL,newobstacle,obstacle,Vx);

        %计算可行触须的Vpath和Vcurvature
        %Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
        [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
        Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   
%         Vpath=NewPathValue(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vcurvature = CurvatureValue(Cnv);
        Vcurvaturedot = CurvatureDotValue(Cnv,CofBestTentacle);
        Vsumcurvature = SumCurvatureValue(rou0,Cnv,Vx,deltaT,deltaL);

        %选出最优触须作为轨迹
%         a1=0;   a2=0.5622;   a3=0.2180;  a4=0.2197; a5=0; a6=0;
%         a1=0;   a2=0.5622;   a3=0.2180;  a4=0; a5=0; a6=0;
%         a1=0;   a2=0.6289;   a3=0.2832;  a4=0.0879; a5=0; a6=0;
          a1=0;   a2=0.5632;   a3=0.3436;  a4=0.0932; a5=0; a6=0;
%         a1=0;   a2=1;   a3=0;  a4=0; a5=0; a6=0;
%         Farbe='b';
%         Vcombined=a1*Vclearance+a2*Vpath+a3*Vcurvature+a4*Vcurvaturedot+a5*Vsumcurvature;
        Vcombined=a1*Vclearance+a2*Vdist+a3*Valpha+a4*Vcurvature+a5*Vcurvaturedot+a6*Vsumcurvature;
        [VcombinedMin,IndexMin]=min(Vcombined);

        %执行该触须，更新初始状态x0,y0,fai,rou0,CurrentC

        %计算每一周期向前行驶序号数
        S=Vx*deltaT;%规划周期内沿轨迹行驶路程
        SIndexTentacle=floor(S/deltaL);
        if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
            SIndexTentacle=SIndexTentacle+1;
        end

        %最优触须曲率变化率
        CofBestTentacle=Cnv(IndexMin);

        %存入总路径各段曲率变化率中
        COfTotalTentacle=[COfTotalTentacle,CofBestTentacle];

        %最优触须各点曲率Curvature
        for i=1:SIndexTentacle
            CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*CofBestTentacle];
        end

        %最优触须各点对应的总路程Stotal
        for i=1:SIndexTentacle
            Stotal=[Stotal,CurrentStotal+i*deltaL];
        end
% 
%         %绘制沿最优触须行驶S的过程中各点曲率随总路程的变化图
%         plot(Stotal,CurvatureOfTotalTentacle*100,'Color',[0,0.45,0.74])

        %绘制触须执行结果
        %plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[1,0.6,0],'LineWidth',2)
%         plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[0,0.45,0.74],'LineWidth',2)

%         figure(1)
%         plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'m','LineWidth',2)
%         title('局部路径规划图','FontSize',16);
%         xlabel('x/m','FontSize',14)
%         ylabel('y/m','FontSize',14)
%         set(gcf,'color','w')
%         plot(xgnv(:,31),ygnv(:,31),'Color',[0,0.45,0.74],'LineWidth',2)


        TotalPlanningPath=[TotalPlanningPath;xgnv(2:SIndexTentacle+1,IndexMin),ygnv(2:SIndexTentacle+1,IndexMin)];

        %更新x0,y0
        x0=xgnv(SIndexTentacle+1,IndexMin);
        y0=ygnv(SIndexTentacle+1,IndexMin);

        %更新fai
        if xgnv(SIndexTentacle+2,IndexMin)==xgnv(SIndexTentacle,IndexMin)
            if ygnv(SIndexTentacle+2,IndexMin)>ygnv(SIndexTentacle,IndexMin)
                fai=pi/2;
            else
                fai=-pi/2;
            end
        elseif xgnv(SIndexTentacle+2,IndexMin)>xgnv(SIndexTentacle,IndexMin)
            SlopeOfTentacle=((ygnv(SIndexTentacle+2,IndexMin)-ygnv(SIndexTentacle,IndexMin)))/((xgnv(SIndexTentacle+2,IndexMin)-xgnv(SIndexTentacle,IndexMin)));
            fai=atan(SlopeOfTentacle); 
        else
            SlopeOfTentacle=((ygnv(SIndexTentacle+2,IndexMin)-ygnv(SIndexTentacle,IndexMin)))/((xgnv(SIndexTentacle+2,IndexMin)-xgnv(SIndexTentacle,IndexMin)));
            if SlopeOfTentacle>0
                fai=atan(SlopeOfTentacle)-pi;
            else
                fai=atan(SlopeOfTentacle)+pi;
            end
        end

        %更新rou0
        rou0=rou0+CofBestTentacle*SIndexTentacle*deltaL;

        %更新当前位置总路径长度CurrentStotal
        CurrentStotal=CurrentStotal+SIndexTentacle*deltaL;

        count=count+1;%第count个规划周期结束
    end
end

%'Color',[1,0.6,0]
%'Color',[0,0.45,0.74]
figure(1)
hold on
% plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),Farbe,'LineWidth',2)
plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),'Color',[0,0.45,0.74],'LineWidth',2)
title('局部路径规划图','FontSize',16);
xlabel('x/m','FontSize',14)
ylabel('y/m','FontSize',14)
set(gcf,'color','w')
axis equal
hold on

% plot(newobstacle(:,1),newobstacle(:,2),'.k','LineWidth',5,'MarkerSize',10,'HandleVisibility','off') 
% alpha=0:pi/40:2*pi;
% for i=1:size(newobstacle,1)
%     x=newobstacle(i,1)+2*cos(alpha);
%     y=newobstacle(i,2)+2*sin(alpha);
%     hold on
%     plot(x,y,'k','LineWidth',1,'HandleVisibility','off')   
% end
% 
% alpha=0:pi/40:2*pi;
% for i=1:size(newobstacle,1)
%     x=newobstacle(i,1)+4.5*cos(alpha);
%     y=newobstacle(i,2)+4.5*sin(alpha);
%     hold on
%     plot(x,y,'k','LineWidth',1,'HandleVisibility','off')   
% end



%最终规划路径评价指标

%1.准确性

Error=[];%存放各点误差
for i=1:size(TotalPlanningPath,1)
    IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
    Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
end

figure(2)
% plot(Stotal,Error,Farbe,'LineWidth',2)
plot(Stotal,Error,'Color',[0,0.45,0.74],'LineWidth',2)
title('侧向偏差距离随路程变化图','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('侧向偏差距离/m','FontSize',14)
set(gcf,'color','w')
hold on

%(1)总路径各点与参考路径横向距离最大值
maxError=max(Error);
%(2)总路径各点与参考路径横向距离平均值
meanError=mean(Error);


% 2.平顺性
figure(3)
% subplot(3,1,3)
% plot(Stotal,CurvatureOfTotalTentacle,Farbe,'LineWidth',2)
plot(Stotal,CurvatureOfTotalTentacle,'Color',[0,0.45,0.74],'LineWidth',2)
title('曲率随路程变化图','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('曲率/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on
%(1)总路径各点曲率绝对值的平均值
meanCurvature=mean(abs(CurvatureOfTotalTentacle));
%(2)总路径各点曲率导数绝对值的平均值
meanC=sum(abs(COfTotalTentacle))/count;

maxCurvature=max(abs(CurvatureOfTotalTentacle));
maxC=max(abs(COfTotalTentacle));

COfTotalTentaclebydot=0;
for i=1:count
    for j=1:SIndexTentacle
        COfTotalTentaclebydot=[COfTotalTentaclebydot,COfTotalTentacle(i)];
    end
end

figure(4)
% subplot(3,1,3)
% plot(Stotal,COfTotalTentaclebydot,Farbe,'LineWidth',2)
plot(Stotal,COfTotalTentaclebydot,'Color',[0,0.45,0.74],'LineWidth',2)
title('曲率导数随路程变化图','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('曲率导数/m^-^2','FontSize',14)
set(gcf,'color','w')
hold on


%save('Astar_clothoid_data_w1=1', 'TotalPlanningPath', 'Stotal', 'Error','CurvatureOfTotalTentacle','COfTotalTentaclebydot')

%save('Astar_clothoid_data_w=0.5632_0.3436_0.0932', 'TotalPlanningPath', 'Stotal', 'Error','CurvatureOfTotalTentacle','COfTotalTentaclebydot')






