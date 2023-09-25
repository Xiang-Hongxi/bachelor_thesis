clear,clc
% figure

% 参考路径
% path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%转弯情形
path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%避障/减速情形
% path=[130,100;120,100;110,100;100,100;90,100;80,100;70,100;60,100;50,100;40,100;40,90;40,80;40,70;40,60;40,50;40,40;30,40;20,40;20,30;20,20];%A*模拟
[ReferencePath,deltaP]  = interplotation(path);

% 绘出ReferencePath
figure(1)
plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5)
hold on

obstacle=[30,20.5];%避障情形
% obstacle=[0,100];%转弯情形
% obstacle=[20,50;30,50;40,30;50,50;50,60;90,100.5];%A*模拟
newobstacle=[50,40];

%局部路径规划
Vx=3; L=2.7; amax=4; x0=0; y0=20; fai=0; rou0=0; CofBestTentacle=0; CurrentStotal=0; 
%初始车辆状态； CofBestTentacle是当前周期最优触须的曲率变化率，初值设为0
% deltamax=pi/6;%前轮最大转角
deltamax=35/180*pi;

deltaT=0.1; 
% deltaT=0.1;%规划周期 单位为s


CurvatureOfTotalTentacle=rou0;%整个路径各点曲率,放入初值rou0
Stotal=0;%整个路径各点总路程，放入初值0
COfTotalTentacle=0;%整个路径各段曲率变化率,放入初值0
TotalPlanningPath=[x0,y0];%存放整个规划路径各点坐标,先存入起点坐标
count=0;%当前处于第count个规划周期

load('circulartentaclesdata.mat')
%导入规划坐标系下圆弧触须坐标

while(1)
    
    if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
        %已到终点区域
        break;
    else
        %开始规划
        
        %全局坐标系下触须坐标
        xg=cos(fai)*xp-sin(fai)*yp+x0;
        yg=sin(fai)*xp+cos(fai)*yp+y0;

        %选出可行触须，并计算其Vclearance
        [xgnv,ygnv,Lcknv,rounv] = ClearanceValue(xg,yg,rou,deltaL,newobstacle,obstacle,Vx,EffectiveNum);

        %计算可行触须的Vpath和Vcurvature
        [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
        Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   

        %选出最优触须作为轨迹
        a1=1;   a2=0;   
        Farbe='b';
        Vcombined=a1*Vdist+a2*Valpha;
        [VcombinedMin,IndexMin]=min(Vcombined);

        %执行该触须，更新初始状态x0,y0,fai,rou0,CurrentC

        %计算每一周期向前行驶序号数
        S=Vx*deltaT;%规划周期内沿轨迹行驶路程
        SIndexTentacle=floor(S/deltaL);
        if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
            SIndexTentacle=SIndexTentacle+1;
        end
        
%         %存入总路径各段曲率变化率中
%         COfTotalTentacle=[COfTotalTentacle,CofBestTentacle];

        %最优触须各点曲率Curvature
        for i=1:SIndexTentacle
            CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rounv(IndexMin)];
        end 
        
        if rounv(IndexMin)==rou0 %曲率不突变
            for i=1:SIndexTentacle
                COfTotalTentacle=[COfTotalTentacle,0];
            end
        elseif rounv(IndexMin)>rou0 %向上突变
            for i=1:SIndexTentacle
                if i==1 %突变点
                    COfTotalTentacle=[COfTotalTentacle,1000000];
                else
                     COfTotalTentacle=[COfTotalTentacle,0];
                end
            end
        else %向下突变
            for i=1:SIndexTentacle
                if i==1 %突变点
                     COfTotalTentacle=[COfTotalTentacle,-1000000];
                else
                     COfTotalTentacle=[COfTotalTentacle,0];
                end
            end  
        end

        %最优触须各点对应的总路程Stotal
        for i=1:SIndexTentacle
            Stotal=[Stotal,CurrentStotal+i*deltaL];
        end

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
        rou0=rounv(IndexMin);
        
        %更新当前位置总路径长度CurrentStotal
        CurrentStotal=CurrentStotal+SIndexTentacle*deltaL;

        count=count+1;%第count个规划周期结束
    end
end

%'Color',[1,0.6,0]
%'Color',[0,0.45,0.74]
figure(1)
plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),Farbe,'LineWidth',2)
title('局部路径规划图','FontSize',16);
xlabel('x/m','FontSize',14)
ylabel('y/m','FontSize',14)
set(gcf,'color','w')
axis equal
hold on


%最终规划路径评价指标

%1.准确性

Error=[];%存放各点误差
for i=1:size(TotalPlanningPath,1)
    IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
    Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
end

figure(2)
plot(Stotal,Error,Farbe,'LineWidth',2)
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
plot(Stotal,CurvatureOfTotalTentacle,Farbe,'LineWidth',2)
title('曲率随路程变化图','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('曲率/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on

%(1)总路径各点曲率绝对值的最大值
maxCurvature=max(abs(CurvatureOfTotalTentacle));
%(2)总路径各点曲率绝对值的平均值
meanCurvature=mean(abs(CurvatureOfTotalTentacle));

% (3)总路径各点曲率导数绝对值的最大值
% maxC=max(abs(COfTotalTentacle));
% (4)总路径各点曲率导数绝对值的平均值
% meanC=sum(abs(COfTotalTentacle))/count;
 
figure(4)
% % subplot(3,1,3)
plot(Stotal,COfTotalTentacle,Farbe,'LineWidth',2)
title('曲率导数随路程变化图','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('曲率/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on










