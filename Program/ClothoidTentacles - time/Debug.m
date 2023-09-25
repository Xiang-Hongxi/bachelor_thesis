clear,clc
% figure

tg=[];%统计触须生成时间
ts=[];%统计触须选择时间

% 参考路径
% path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%转弯情形
path=[50,70;40,70;30,70;20,70;10,70;10,60;10,50];%转弯new
% path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%避障/减速情形
% path=[85,20;75,20;65,20;55,20;45,20;35,20;25,20;15,20];%避障/减速情形
[ReferencePath,deltaP]  = interplotation(path);

% 绘出ReferencePath
% figure(1)
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5,'HandleVisibility','off')
% hold on

% obstacle=[30,20.5];%避障情形
% obstacle=[0,100];%转弯情形
% obstacle=[20,50;30,50;40,30;50,50;50,60;90,100.5];%A*模拟
% newobstacle=[30,21.5];
newobstacle=[0,10];

deltaT=0.1; 


for expnum=1:5000;
    
    %局部路径规划
    Vx=3; L=2.7; amax=4; x0=10; y0=50; fai=pi/2; rou0=0; CofBestTentacle=0; CurrentStotal=0; 
    %初始车辆状态； CofBestTentacle是当前周期最优触须的曲率变化率，初值设为0
    % deltamax=pi/6;%前轮最大转角
    deltamax=35/180*pi;
    % deltaT=0.1;%规划周期 单位为s

%     CurvatureOfTotalTentacle=rou0;%整个路径各点曲率,放入初值rou0
    Stotal=0;%整个路径各点总路程，放入初值0
%     COfTotalTentacle=[];%整个路径各段曲率变化率
    
%     TotalPlanningPath=[x0,y0];%存放整个规划路径各点坐标,先存入起点坐标
    count=0;%当前处于第count个规划周期

    t_generate=[];%存放各周期生成触须所需时间
    t_select=[];%存放各周期选择触须所需时间

    while(1)

        if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
            %已到终点区域
            break;
        else
            %开始规划

            t0=tic;
            %生成触须
            [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0); 
            t_generate=[t_generate,toc(t0)];

            t1=tic;
            %选出可行触须，并计算其Vclearance
            [xgnv,ygnv,Cnv,L0,Lc] = ClearanceValue(xg,yg,C,deltaL,newobstacle,Vx);

            %计算可行触须的Vpath和Vcurvature
            %Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
            [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
            Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
            Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   
            Vcurvature = CurvatureValue(Cnv);


            %选出最优触须作为轨迹
            a1=0.5632;   a2=0.3436;   a3=0.0932; 
%             a1=1;   a2=0;   a3=0;
            Vcombined=a1*Vdist+a2*Valpha+a3*Vcurvature;
            [VcombinedMin,IndexMin]=min(Vcombined);
            t_select=[t_select,toc(t1)];
            
            colormatrix=[0.24,0.17,0.43;1,0.84,0;0,0.6,0.8;1,0.4,0.4];
            colorset=1;

            %执行该触须，更新初始状态x0,y0,fai,rou0,CurrentC

            %计算每一周期向前行驶序号数
            S=Vx*deltaT;%规划周期内沿轨迹行驶路程
            SIndexTentacle=floor(S/deltaL);
            if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
                SIndexTentacle=SIndexTentacle+1;
            end

%             最优触须曲率变化率
            CofBestTentacle=Cnv(IndexMin);

%             存入总路径各段曲率变化率中
%             COfTotalTentacle=[COfTotalTentacle,CofBestTentacle];

%             最优触须各点曲率Curvature
%             for i=1:SIndexTentacle
%                 CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*CofBestTentacle];
%             end

%             最优触须各点对应的总路程Stotal
            for i=1:SIndexTentacle
                Stotal=[Stotal,CurrentStotal+i*deltaL];
            end
            
%             TotalPlanningPath=[TotalPlanningPath;xgnv(2:SIndexTentacle+1,IndexMin),ygnv(2:SIndexTentacle+1,IndexMin)];

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

%     figure(1)
%     plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('局部路径规划图','FontSize',16);
%     xlabel('x/m','FontSize',14)
%     ylabel('y/m','FontSize',14)
%     set(gcf,'color','w')
%     axis equal
%     hold on
%     grid on
% 
%     plot(newobstacle(:,1),newobstacle(:,2),'.k','LineWidth',5,'MarkerSize',10,'HandleVisibility','off') 
% 
%     alpha=0:pi/40:2*pi;
%     for i=1:size(newobstacle,1)
%         x=newobstacle(i,1)+2*cos(alpha);
%         y=newobstacle(i,2)+2*sin(alpha);
%         hold on
%         plot(x,y,'k','LineWidth',1,'HandleVisibility','off')   
%     end
%     hold on
%     alpha=0:pi/40:2*pi;
%     for i=1:size(newobstacle,1)
%         x=newobstacle(i,1)+4.5*cos(alpha);
%         y=newobstacle(i,2)+4.5*sin(alpha);
%         hold on
%         plot(x,y,'k','LineWidth',1,'HandleVisibility','off')   
%     end
%     hold on
% 
%     %最终规划路径评价指标
% 
%     %1.准确性
% 
%     Error=[];%存放各点误差
%     for i=1:size(TotalPlanningPath,1)
%         IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
%         Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
%     end
% % 
%     figure(2)
%     plot(Stotal,Error,'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('侧向偏差距离随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('侧向偏差距离/m','FontSize',14)
%     set(gcf,'color','w')
%     hold on
%     grid on

%     %(1)总路径各点与参考路径横向距离最大值
%     maxError=max(Error);
%     %(2)总路径各点与参考路径横向距离平均值
%     meanError=mean(Error);


    % 2.平顺性
%     figure(3)
%     plot(Stotal,CurvatureOfTotalTentacle,'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('曲率随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('曲率/m^-^1','FontSize',14)
%     set(gcf,'color','w')
%     hold on
%     grid on
    %(1)总路径各点曲率绝对值的平均值
%     meanCurvature=mean(abs(CurvatureOfTotalTentacle));
%     %(2)总路径各点曲率导数绝对值的平均值
%     meanC=sum(abs(COfTotalTentacle))/count;
% 
%     maxCurvature=max(abs(CurvatureOfTotalTentacle));
%     maxC=max(abs(COfTotalTentacle));

%     COfTotalTentaclebydot=0;
%     for i=1:count
%         for j=1:SIndexTentacle
%             COfTotalTentaclebydot=[COfTotalTentaclebydot,COfTotalTentacle(i)];
%         end
%     end

%     figure(4)
%     plot(Stotal,COfTotalTentaclebydot,'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('曲率导数随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('曲率导数/m^-^2','FontSize',14)
%     set(gcf,'color','w')
%     hold on
%     grid on

    tg=[tg,mean(t_generate)];
    ts=[ts,mean(t_select)];
end

% 
save('timedata5000_tnew', 'ts', 'tg')


clear,clc
% figure

tg=[];%统计触须生成时间
ts=[];%统计触须选择时间

% 参考路径
% path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%转弯情形
% path=[50,70;40,70;30,70;20,70;10,70;10,60;10,50];
% path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%避障/减速情形
path=[75,20;65,20;55,20;45,20;35,20;25,20;15,20];%避障/减速情形
[ReferencePath,deltaP]  = interplotation(path);

% 绘出ReferencePath
% figure(1)
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5,'HandleVisibility','off')
% hold on

% obstacle=[30,20.5];%避障情形
% obstacle=[0,100];%转弯情形
% obstacle=[20,50;30,50;40,30;50,50;50,60;90,100.5];%A*模拟
newobstacle=[30,21.5];
% newobstacle=[0,120];

deltaT=0.1; 


for expnum=1:5000;
    
    %局部路径规划
    Vx=3; L=2.7; amax=4; x0=15; y0=20; fai=0; rou0=0; CofBestTentacle=0; CurrentStotal=0; 
    %初始车辆状态； CofBestTentacle是当前周期最优触须的曲率变化率，初值设为0
    % deltamax=pi/6;%前轮最大转角
    deltamax=35/180*pi;
    % deltaT=0.1;%规划周期 单位为s

%     CurvatureOfTotalTentacle=rou0;%整个路径各点曲率,放入初值rou0
    Stotal=0;%整个路径各点总路程，放入初值0
%     COfTotalTentacle=[];%整个路径各段曲率变化率
    
%     TotalPlanningPath=[x0,y0];%存放整个规划路径各点坐标,先存入起点坐标
    count=0;%当前处于第count个规划周期

    t_generate=[];%存放各周期生成触须所需时间
    t_select=[];%存放各周期选择触须所需时间

    while(1)

        if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
            %已到终点区域
            break;
        else
            %开始规划

            t0=tic;
            %生成触须
            [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0); 
            t_generate=[t_generate,toc(t0)];

            t1=tic;
            %选出可行触须，并计算其Vclearance
            [xgnv,ygnv,Cnv,L0,Lc] = ClearanceValue(xg,yg,C,deltaL,newobstacle,Vx);

            %计算可行触须的Vpath和Vcurvature
            %Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
            [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
            Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
            Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   
            Vcurvature = CurvatureValue(Cnv);


            %选出最优触须作为轨迹
            a1=0.5632;   a2=0.3436;   a3=0.0932; 
%             a1=1;   a2=0;   a3=0;
            Vcombined=a1*Vdist+a2*Valpha+a3*Vcurvature;
            [VcombinedMin,IndexMin]=min(Vcombined);
            t_select=[t_select,toc(t1)];
            
            colormatrix=[0.24,0.17,0.43;1,0.84,0;0,0.6,0.8;1,0.4,0.4];
            colorset=1;

            %执行该触须，更新初始状态x0,y0,fai,rou0,CurrentC

            %计算每一周期向前行驶序号数
            S=Vx*deltaT;%规划周期内沿轨迹行驶路程
            SIndexTentacle=floor(S/deltaL);
            if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
                SIndexTentacle=SIndexTentacle+1;
            end

%             最优触须曲率变化率
            CofBestTentacle=Cnv(IndexMin);

%             存入总路径各段曲率变化率中
%             COfTotalTentacle=[COfTotalTentacle,CofBestTentacle];

%             最优触须各点曲率Curvature
%             for i=1:SIndexTentacle
%                 CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*CofBestTentacle];
%             end

%             最优触须各点对应的总路程Stotal
            for i=1:SIndexTentacle
                Stotal=[Stotal,CurrentStotal+i*deltaL];
            end
            
%             TotalPlanningPath=[TotalPlanningPath;xgnv(2:SIndexTentacle+1,IndexMin),ygnv(2:SIndexTentacle+1,IndexMin)];

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

%     figure(1)
%     plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('局部路径规划图','FontSize',16);
%     xlabel('x/m','FontSize',14)
%     ylabel('y/m','FontSize',14)
%     set(gcf,'color','w')
%     axis equal
%     hold on
%     grid on
% 
%     plot(newobstacle(:,1),newobstacle(:,2),'.k','LineWidth',5,'MarkerSize',10,'HandleVisibility','off') 
% 
%     alpha=0:pi/40:2*pi;
%     for i=1:size(newobstacle,1)
%         x=newobstacle(i,1)+2*cos(alpha);
%         y=newobstacle(i,2)+2*sin(alpha);
%         hold on
%         plot(x,y,'k','LineWidth',1,'HandleVisibility','off')   
%     end
%     hold on
%     alpha=0:pi/40:2*pi;
%     for i=1:size(newobstacle,1)
%         x=newobstacle(i,1)+4.5*cos(alpha);
%         y=newobstacle(i,2)+4.5*sin(alpha);
%         hold on
%         plot(x,y,'k','LineWidth',1,'HandleVisibility','off')   
%     end
%     hold on
% 
%     %最终规划路径评价指标
% 
%     %1.准确性
% 
%     Error=[];%存放各点误差
%     for i=1:size(TotalPlanningPath,1)
%         IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
%         Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
%     end
% % 
%     figure(2)
%     plot(Stotal,Error,'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('侧向偏差距离随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('侧向偏差距离/m','FontSize',14)
%     set(gcf,'color','w')
%     hold on
%     grid on

%     %(1)总路径各点与参考路径横向距离最大值
%     maxError=max(Error);
%     %(2)总路径各点与参考路径横向距离平均值
%     meanError=mean(Error);


    % 2.平顺性
%     figure(3)
%     plot(Stotal,CurvatureOfTotalTentacle,'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('曲率随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('曲率/m^-^1','FontSize',14)
%     set(gcf,'color','w')
%     hold on
%     grid on
    %(1)总路径各点曲率绝对值的平均值
%     meanCurvature=mean(abs(CurvatureOfTotalTentacle));
%     %(2)总路径各点曲率导数绝对值的平均值
%     meanC=sum(abs(COfTotalTentacle))/count;
% 
%     maxCurvature=max(abs(CurvatureOfTotalTentacle));
%     maxC=max(abs(COfTotalTentacle));

%     COfTotalTentaclebydot=0;
%     for i=1:count
%         for j=1:SIndexTentacle
%             COfTotalTentaclebydot=[COfTotalTentaclebydot,COfTotalTentacle(i)];
%         end
%     end

%     figure(4)
%     plot(Stotal,COfTotalTentaclebydot,'Color',colormatrix(colorset,:),'LineWidth',2)
%     title('曲率导数随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('曲率导数/m^-^2','FontSize',14)
%     set(gcf,'color','w')
%     hold on
%     grid on

    tg=[tg,mean(t_generate)];
    ts=[ts,mean(t_select)];
end

% 
save('timedata5000_anew', 'ts', 'tg')





