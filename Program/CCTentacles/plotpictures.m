% clear,clc
% figure

% %参考路径

path=[60,20;50,20;40,20;30,20;20,20];%用于出图
path1=[30,20;20,20];
[ReferencePath,deltaP]  = interplotation(path);

%绘出ReferencePath
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',2)
% hold on
plot(path1(:,1),path1(:,2),'r','LineWidth',2)
hold on


%局部路径规划
% Vx=6; L=3; amax=4; x0=20; y0=10; fai=pi/2; rou0=0; %初始车辆状态
% deltamax=pi/6;%前轮最大转角
deltamax=30/180*pi;

Vx=3;x0=20; y0=19;fai=0.2;%

deltaT=1; 
% deltaT=0.1;%规划周期 单位为s

% obstacle=[10,45;22,33];%若有移动障碍物，每一规划周期obstacle可能会变，所以应放入while循环中
% obstacle=[20,65];%减速情形
% obstacle=[20,60];%避障情形
obstacle=[0,100];%转弯情形
% obstacle=[20,50;30,50;40,30;50,50;50,60];%A*模拟

while(1)
    
    %生成触须
    [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0); 
    
    %选出可行触须，并计算其Vclearance
    [xgnv,ygnv,Cnv,L0,Vclearance,Lcknv] = ClearanceValue(xg,yg,C,deltaL,obstacle,Vx);
    
    %计算可行触须的Vpath和Vcurvature
%     Lc=25;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Lc=18;
%     Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
    Vpath=NewPathValue(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
    Vcurvature = CurvatureValue(Cnv);
    
    %选出最优触须作为轨迹
    a1=0;   a2=1;    a3=0;
    Vcombined=a1*Vclearance+a2*Vpath+a3*Vcurvature;
    [VcombinedMin,IndexMin]=min(Vcombined);
    
    
    %执行该触须，更新初始状态x0,y0,fai,rou0
    
    %更新x0,y0,
    S=Vx*deltaT;%规划周期内沿轨迹行驶路程
    SIndexTentacle=floor(S/deltaL);
    if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
        SIndexTentacle=SIndexTentacle+1;
    end
    x0=xgnv(SIndexTentacle+1,IndexMin);
    y0=ygnv(SIndexTentacle+1,IndexMin);
    
    %绘制触须执行结果
    hold on
%     plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[1,0.6,0],'LineWidth',2)
    plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[0,0.45,0.74],'LineWidth',2)
    
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
    rou0=rou0+Cnv(IndexMin)*SIndexTentacle*deltaL;

%     close
end

















