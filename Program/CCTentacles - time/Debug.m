% clear,clc
% 
% tg=[];%统计触须生成时间
% ts=[];%统计触须选择时间
% 
% % 参考路径
% % path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%转弯情形
% path=[50,70;40,70;30,70;20,70;10,70;10,60;10,50];%转弯new
% % path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%避障/减速情形
% % path=[75,20;65,20;55,20;45,20;35,20;25,20;15,20];%避障new
% [ReferencePath,deltaP]  = interplotation(path);
% 
% % 绘出ReferencePath
% % figure(1)
% % plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5)
% % hold on
% 
% 
% % newobstacle=[30,21.5];%避障情形
% newobstacle=[0,10];%转弯情形
% 
% for expnum=1:5000;
% 
%     %局部路径规划
%     Vx=3; L=2.7; amax=4; x0=10; y0=50; fai=pi/2; rou0=0; C0=0; CurrentStotal=0; 
%     deltamax=35/180*pi;
% 
%     m=0.5;%曲率导数C表达式为：C=as^n+C0
% 
%     deltaT=0.1; 
% 
%     count=0;%当前处于第count个规划周期
%     
% %     TotalPlanningPath=[x0,y0];%存放整个规划路径各点坐标,先存入起点坐标
% 
%     t_generate=[];%存放各周期生成触须所需时间
%     t_select=[];%存放各周期选择触须所需时间
% 
%     while(1)
% 
%         if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
%             %已到终点区域
%             break;
%         else
%             %开始规划
% 
%             t0=tic;
%             %生成触须
%             [xg,yg,deltaL,a]=CCTentacles(deltamax,Vx,L,amax,x0,y0,rou0,C0,fai,m); 
%             t_generate=[t_generate,toc(t0)];
% 
%             t1=tic;
%             %选出可行触须，并计算其Vclearance
%             [xgnv,ygnv,anv,L0,Lc] = ClearanceValue(xg,yg,a,deltaL,newobstacle,Vx);
% 
%             %计算可行触须的Vpath和Vcurvature
%             %Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
%             [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
%             Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
%             Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   
%             Vcurvature = CurvatureValue(anv,C0,Vx,deltaT,m);
% 
%             %选出最优触须作为轨迹
% %             a1=1;   a2=0;  a3=0; 
%             a1=0.5632;   a2=0.3436;   a3=0.0932;
%             Vcombined=a1*Vdist+a2*Valpha+a3*Vcurvature;
%             [VcombinedMin,IndexMin]=min(Vcombined);
%             t_select=[t_select,toc(t1)];
% 
%             %执行该触须，更新初始状态x0,y0,fai,rou0,CurrentC
% 
%             %计算每一周期向前行驶序号数
%             S=Vx*deltaT;%规划周期内沿轨迹行驶路程
%             SIndexTentacle=floor(S/deltaL);
%             if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
%                 SIndexTentacle=SIndexTentacle+1;
%             end
% 
%             %最优触须a值
%             aofBestTentacle=anv(IndexMin);
% 
% %             %最优触须各点曲率导数C
% %             for i=1:SIndexTentacle
% %     %             COfTotalTentacle=[COfTotalTentacle,C0+i*deltaL*dCdlofBestTentacle];
% %                 COfTotalTentacle=[COfTotalTentacle,C0+(i*deltaL)^m*aofBestTentacle];
% %             end
% 
% %             %最优触须各点曲率Curvature
% %             for i=1:SIndexTentacle
% %     %             CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*C0+1/2*dCdlofBestTentacle*(i*deltaL)^2];
% %                 CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*C0+(aofBestTentacle*(i*deltaL)^(m+1))/(m+1)];
% %             end 
% 
% %             %最优触须各点对应的总路程Stotal
% %             for i=1:SIndexTentacle
% %                 Stotal=[Stotal,CurrentStotal+i*deltaL];
% %             end
%     % 
%     %         %绘制沿最优触须行驶S的过程中各点曲率随总路程的变化图
%     %         plot(Stotal,CurvatureOfTotalTentacle*100,'Color',[0,0.45,0.74])
% 
%             %绘制触须执行结果
%             %plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[1,0.6,0],'LineWidth',2)
%     %         plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[0,0.45,0.74],'LineWidth',2)
% 
%     %         figure(1)
%     %         plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'m','LineWidth',2)
%     %         title('局部路径规划图','FontSize',16);
%     %         xlabel('x/m','FontSize',14)
%     %         ylabel('y/m','FontSize',14)
%     %         set(gcf,'color','w')
%     %         plot(xgnv(:,31),ygnv(:,31),'Color',[0,0.45,0.74],'LineWidth',2)
% 
% 
% %             TotalPlanningPath=[TotalPlanningPath;xgnv(2:SIndexTentacle+1,IndexMin),ygnv(2:SIndexTentacle+1,IndexMin)];
%           
%             
%             %更新x0,y0
%             x0=xgnv(SIndexTentacle+1,IndexMin);
%             y0=ygnv(SIndexTentacle+1,IndexMin);
% 
%             %更新fai
%             if xgnv(SIndexTentacle+2,IndexMin)==xgnv(SIndexTentacle,IndexMin)
%                 if ygnv(SIndexTentacle+2,IndexMin)>ygnv(SIndexTentacle,IndexMin)
%                     fai=pi/2;
%                 else
%                     fai=-pi/2;
%                 end
%             elseif xgnv(SIndexTentacle+2,IndexMin)>xgnv(SIndexTentacle,IndexMin)
%                 SlopeOfTentacle=((ygnv(SIndexTentacle+2,IndexMin)-ygnv(SIndexTentacle,IndexMin)))/((xgnv(SIndexTentacle+2,IndexMin)-xgnv(SIndexTentacle,IndexMin)));
%                 fai=atan(SlopeOfTentacle); 
%             else
%                 SlopeOfTentacle=((ygnv(SIndexTentacle+2,IndexMin)-ygnv(SIndexTentacle,IndexMin)))/((xgnv(SIndexTentacle+2,IndexMin)-xgnv(SIndexTentacle,IndexMin)));
%                 if SlopeOfTentacle>0
%                     fai=atan(SlopeOfTentacle)-pi;
%                 else
%                     fai=atan(SlopeOfTentacle)+pi;
%                 end
%             end
% 
%             %更新rou0
%     %         rou0=rou0+CofBestTentacle*SIndexTentacle*deltaL;
%     %         rou0=rou0+C0*(SIndexTentacle*deltaL)+1/2*dCdlofBestTentacle*(SIndexTentacle*deltaL)^2;
%             rou0=rou0+C0*(SIndexTentacle*deltaL)+(aofBestTentacle*(SIndexTentacle*deltaL)^(m+1))/(m+1);
% 
%             %更新C0
%     %         C0=C0+dCdlofBestTentacle*(SIndexTentacle*deltaL);
%             C0=C0+aofBestTentacle*(SIndexTentacle*deltaL)^m;
% 
% %             %更新当前位置总路径长度CurrentStotal
% %             CurrentStotal=CurrentStotal+SIndexTentacle*deltaL;
% 
%             count=count+1;%第count个规划周期结束
%         end
%     end
% 
% %     figure(1)
% %     plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),'b','LineWidth',2)
% %     title('局部路径规划图','FontSize',16);
% %     xlabel('x/m','FontSize',14)
% %     ylabel('y/m','FontSize',14)
% %     set(gcf,'color','w')
% %     axis equal
% 
% 
%     %最终规划路径评价指标
% 
%     %1.准确性
% 
% %     Error=[];%存放各点误差
% %     for i=1:size(TotalPlanningPath,1)
% %         IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
% %         Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
% %     end
% 
% %     figure(2)
% %     plot(Stotal,Error,Farbe,'LineWidth',2)
% %     title('侧向偏差距离随路程变化图','FontSize',16);
% %     xlabel('s/m','FontSize',14)
% %     ylabel('侧向偏差距离/m','FontSize',14)
% %     set(gcf,'color','w')
% %     hold on
% 
% %     %(1)总路径各点与参考路径横向距离最大值
% %     maxError=max(Error);
% %     %(2)总路径各点与参考路径横向距离平均值
% %     meanError=mean(Error);
% 
% 
%     % 2.平顺性
% %     figure(3)
% %     % subplot(3,1,3)
% %     plot(Stotal,CurvatureOfTotalTentacle,Farbe,'LineWidth',2)
% %     title('曲率随路程变化图','FontSize',16);
% %     xlabel('s/m','FontSize',14)
% %     ylabel('曲率/m^-^1','FontSize',14)
% %     set(gcf,'color','w')
% %     hold on
% %     %(1)总路径各点曲率绝对值的平均值
% %     meanAbsCurvature=mean(abs(CurvatureOfTotalTentacle));
% %     %(2)总路径各点曲率导数绝对值的平均值
% %     meanAbsC=mean(abs(COfTotalTentacle));
% 
% %     maxCurvature=max(abs(CurvatureOfTotalTentacle));
% %     maxC=max(abs(COfTotalTentacle));
% 
% 
% %     figure(4)
%     % subplot(3,1,3)
% %     plot(Stotal,COfTotalTentacle,Farbe,'LineWidth',2)
% %     title('曲率导数随路程变化图','FontSize',16);
% %     xlabel('s/m','FontSize',14)
% %     ylabel('曲率/m^-^1','FontSize',14)
% %     set(gcf,'color','w')
% %     hold on
% 
%     tg=[tg,mean(t_generate)];
%     ts=[ts,mean(t_select)];
% 
% end
% 
% save('timedata5000CC_tnew', 'ts', 'tg')


clear,clc

tg=[];%统计触须生成时间
ts=[];%统计触须选择时间

% 参考路径
% path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%转弯情形
% path=[50,70;40,70;30,70;20,70;10,70;10,60;10,50];%转弯new
% path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%避障/减速情形
path=[75,20;65,20;55,20;45,20;35,20;25,20;15,20];%避障new
[ReferencePath,deltaP]  = interplotation(path);

% 绘出ReferencePath
% figure(1)
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5)
% hold on


newobstacle=[30,21.5];%避障情形
% newobstacle=[0,10];%转弯情形

for expnum=1:5000;

    %局部路径规划
    Vx=3; L=2.7; amax=4; x0=15; y0=20; fai=0; rou0=0; C0=0; CurrentStotal=0; 
    deltamax=35/180*pi;

    m=0.5;%曲率导数C表达式为：C=as^n+C0

    deltaT=0.1; 

    count=0;%当前处于第count个规划周期
    
%     TotalPlanningPath=[x0,y0];%存放整个规划路径各点坐标,先存入起点坐标

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
            [xg,yg,deltaL,a]=CCTentacles(deltamax,Vx,L,amax,x0,y0,rou0,C0,fai,m); 
            t_generate=[t_generate,toc(t0)];

            t1=tic;
            %选出可行触须，并计算其Vclearance
            [xgnv,ygnv,anv,L0,Lc] = ClearanceValue(xg,yg,a,deltaL,newobstacle,Vx);

            %计算可行触须的Vpath和Vcurvature
            %Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
            [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
            Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
            Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   
            Vcurvature = CurvatureValue(anv,C0,Vx,deltaT,m);

            %选出最优触须作为轨迹
%             a1=1;   a2=0;  a3=0; 
            a1=0.5632;   a2=0.3436;   a3=0.0932;
            Vcombined=a1*Vdist+a2*Valpha+a3*Vcurvature;
            [VcombinedMin,IndexMin]=min(Vcombined);
            t_select=[t_select,toc(t1)];

            %执行该触须，更新初始状态x0,y0,fai,rou0,CurrentC

            %计算每一周期向前行驶序号数
            S=Vx*deltaT;%规划周期内沿轨迹行驶路程
            SIndexTentacle=floor(S/deltaL);
            if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
                SIndexTentacle=SIndexTentacle+1;
            end

            %最优触须a值
            aofBestTentacle=anv(IndexMin);

%             %最优触须各点曲率导数C
%             for i=1:SIndexTentacle
%     %             COfTotalTentacle=[COfTotalTentacle,C0+i*deltaL*dCdlofBestTentacle];
%                 COfTotalTentacle=[COfTotalTentacle,C0+(i*deltaL)^m*aofBestTentacle];
%             end

%             %最优触须各点曲率Curvature
%             for i=1:SIndexTentacle
%     %             CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*C0+1/2*dCdlofBestTentacle*(i*deltaL)^2];
%                 CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*C0+(aofBestTentacle*(i*deltaL)^(m+1))/(m+1)];
%             end 

%             %最优触须各点对应的总路程Stotal
%             for i=1:SIndexTentacle
%                 Stotal=[Stotal,CurrentStotal+i*deltaL];
%             end
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
    %         rou0=rou0+CofBestTentacle*SIndexTentacle*deltaL;
    %         rou0=rou0+C0*(SIndexTentacle*deltaL)+1/2*dCdlofBestTentacle*(SIndexTentacle*deltaL)^2;
            rou0=rou0+C0*(SIndexTentacle*deltaL)+(aofBestTentacle*(SIndexTentacle*deltaL)^(m+1))/(m+1);

            %更新C0
    %         C0=C0+dCdlofBestTentacle*(SIndexTentacle*deltaL);
            C0=C0+aofBestTentacle*(SIndexTentacle*deltaL)^m;

%             %更新当前位置总路径长度CurrentStotal
%             CurrentStotal=CurrentStotal+SIndexTentacle*deltaL;

            count=count+1;%第count个规划周期结束
        end
    end

%     figure(1)
%     plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),'b','LineWidth',2)
%     title('局部路径规划图','FontSize',16);
%     xlabel('x/m','FontSize',14)
%     ylabel('y/m','FontSize',14)
%     set(gcf,'color','w')
%     axis equal


    %最终规划路径评价指标

    %1.准确性

%     Error=[];%存放各点误差
%     for i=1:size(TotalPlanningPath,1)
%         IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
%         Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
%     end

%     figure(2)
%     plot(Stotal,Error,Farbe,'LineWidth',2)
%     title('侧向偏差距离随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('侧向偏差距离/m','FontSize',14)
%     set(gcf,'color','w')
%     hold on

%     %(1)总路径各点与参考路径横向距离最大值
%     maxError=max(Error);
%     %(2)总路径各点与参考路径横向距离平均值
%     meanError=mean(Error);


    % 2.平顺性
%     figure(3)
%     % subplot(3,1,3)
%     plot(Stotal,CurvatureOfTotalTentacle,Farbe,'LineWidth',2)
%     title('曲率随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('曲率/m^-^1','FontSize',14)
%     set(gcf,'color','w')
%     hold on
%     %(1)总路径各点曲率绝对值的平均值
%     meanAbsCurvature=mean(abs(CurvatureOfTotalTentacle));
%     %(2)总路径各点曲率导数绝对值的平均值
%     meanAbsC=mean(abs(COfTotalTentacle));

%     maxCurvature=max(abs(CurvatureOfTotalTentacle));
%     maxC=max(abs(COfTotalTentacle));


%     figure(4)
    % subplot(3,1,3)
%     plot(Stotal,COfTotalTentacle,Farbe,'LineWidth',2)
%     title('曲率导数随路程变化图','FontSize',16);
%     xlabel('s/m','FontSize',14)
%     ylabel('曲率/m^-^1','FontSize',14)
%     set(gcf,'color','w')
%     hold on

    tg=[tg,mean(t_generate)];
    ts=[ts,mean(t_select)];

end

save('timedata5000CC_anewnew', 'ts', 'tg')


