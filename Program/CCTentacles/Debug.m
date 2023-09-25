% clear,clc
% figure

% �ο�·��
path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%ת������
% path=[10,50;10,40;10,30];%��ͼʾ��
% path=[40,130;40,120;40,110;40,100;40,90;40,80;40,70;40,60;40,50;40,40;40,30;30,30;20,30;20,20];%����ת������
% path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%����/��������
% path=[130,100;120,100;110,100;100,100;90,100;80,100;70,100;60,100;50,100;40,100;40,90;40,80;40,70;40,60;40,50;40,40;30,40;20,40;20,30;20,20];%A*ģ��
% path=[40,100;40,90;40,80;40,70;40,60;40,50;40,40;30,40;20,40;20,30];
[ReferencePath,deltaP]  = interplotation(path);

% ���ReferencePath
% figure(1)
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5)
% hold on


% obstacle=[10,45;22,33];%�����ƶ��ϰ��ÿһ�滮����obstacle���ܻ�䣬����Ӧ����whileѭ����
% obstacle=[20,65];%��������
% obstacle=[30,20.5];%��������
obstacle=[0,100];%ת������
% obstacle=[20,50;30,50;40,30;50,50;50,60;90,100.5];%A*ģ��
newobstacle=[52,70;110,119.5];

%�ֲ�·���滮
Vx=3; L=2.7; amax=4; x0=10; y0=30; fai=pi/2; rou0=0; C0=0; CurrentStotal=0; 
%��ʼ����״̬�� CofBestTentacle�ǵ�ǰ�������Ŵ�������ʱ仯�ʣ���ֵ��Ϊ0
% deltamax=pi/6;%ǰ�����ת��
deltamax=35/180*pi;

m=0.5;%���ʵ���C���ʽΪ��C=as^n+C0

deltaT=0.1; 
% deltaT=0.1;%�滮���� ��λΪs


CurvatureOfTotalTentacle=rou0;%����·����������,�����ֵrou0
COfTotalTentacle=C0;%����·���������ʱ仯��,�����ֵC0
Stotal=0;%����·��������·�̣������ֵ0
TotalPlanningPath=[x0,y0];%��������滮·����������,�ȴ����������
count=0;%��ǰ���ڵ�count���滮����


while(1)
    
    if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
        %�ѵ��յ�����
        break;
    else
        %��ʼ�滮
        
        %���ɴ���
        [xg,yg,deltaL,a]=CCTentacles(deltamax,Vx,L,amax,x0,y0,rou0,C0,fai,m); 

        %ѡ�����д��룬��������Vclearance
        [xgnv,ygnv,anv,L0,Vclearance,Lcknv] = ClearanceValue(xg,yg,a,deltaL,newobstacle,obstacle,Vx);

        %������д����Vpath��Vcurvature
        %Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
        [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
        Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   
%         Vpath=NewPathValue(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vcurvature = CurvatureDotValue(anv,C0,Vx,deltaT,m,rou0);

        %ѡ�����Ŵ�����Ϊ�켣
        a1=1;   a2=0;  a3=0; a4=0;
        Farbe='m';
        Vcombined=a1*Vdist+a2*Valpha+a3*Vcurvature+a4*Vclearance;
        [VcombinedMin,IndexMin]=min(Vcombined);

        %ִ�иô��룬���³�ʼ״̬x0,y0,fai,rou0,CurrentC

        %����ÿһ������ǰ��ʻ�����
        S=Vx*deltaT;%�滮�������ع켣��ʻ·��
        SIndexTentacle=floor(S/deltaL);
        if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
            SIndexTentacle=SIndexTentacle+1;
        end

        %���Ŵ���aֵ
        aofBestTentacle=anv(IndexMin);
        
        %���Ŵ���������ʵ���C
        for i=1:SIndexTentacle
%             COfTotalTentacle=[COfTotalTentacle,C0+i*deltaL*dCdlofBestTentacle];
            COfTotalTentacle=[COfTotalTentacle,C0+(i*deltaL)^m*aofBestTentacle];
        end
        
        %���Ŵ����������Curvature
        for i=1:SIndexTentacle
%             CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*C0+1/2*dCdlofBestTentacle*(i*deltaL)^2];
            CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*C0+(aofBestTentacle*(i*deltaL)^(m+1))/(m+1)];
        end 
        
        %���Ŵ�������Ӧ����·��Stotal
        for i=1:SIndexTentacle
            Stotal=[Stotal,CurrentStotal+i*deltaL];
        end
% 
%         %���������Ŵ�����ʻS�Ĺ����и�����������·�̵ı仯ͼ
%         plot(Stotal,CurvatureOfTotalTentacle*100,'Color',[0,0.45,0.74])

        %���ƴ���ִ�н��
        hold on
        %plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[1,0.6,0],'LineWidth',2)
%         plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[0,0.45,0.74],'LineWidth',2)

%         figure(1)
%         plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'m','LineWidth',2)
%         title('�ֲ�·���滮ͼ','FontSize',16);
%         xlabel('x/m','FontSize',14)
%         ylabel('y/m','FontSize',14)
%         set(gcf,'color','w')
%         plot(xgnv(:,31),ygnv(:,31),'Color',[0,0.45,0.74],'LineWidth',2)


        TotalPlanningPath=[TotalPlanningPath;xgnv(2:SIndexTentacle+1,IndexMin),ygnv(2:SIndexTentacle+1,IndexMin)];

        %����x0,y0
        x0=xgnv(SIndexTentacle+1,IndexMin);
        y0=ygnv(SIndexTentacle+1,IndexMin);

        %����fai
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

        %����rou0
%         rou0=rou0+CofBestTentacle*SIndexTentacle*deltaL;
%         rou0=rou0+C0*(SIndexTentacle*deltaL)+1/2*dCdlofBestTentacle*(SIndexTentacle*deltaL)^2;
        rou0=rou0+C0*(SIndexTentacle*deltaL)+(aofBestTentacle*(SIndexTentacle*deltaL)^(m+1))/(m+1);
        
        %����C0
%         C0=C0+dCdlofBestTentacle*(SIndexTentacle*deltaL);
        C0=C0+aofBestTentacle*(SIndexTentacle*deltaL)^m;

        %���µ�ǰλ����·������CurrentStotal
        CurrentStotal=CurrentStotal+SIndexTentacle*deltaL;

        count=count+1;%��count���滮���ڽ���
    end
end

%'Color',[1,0.6,0]
%'Color',[0,0.45,0.74]
figure(1)
hold on
plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),'Color',[0,0.75,0.75],'LineWidth',2)
title('�ֲ�·���滮ͼ','FontSize',16);
xlabel('x/m','FontSize',14)
ylabel('y/m','FontSize',14)
set(gcf,'color','w')
axis equal


%���չ滮·������ָ��

%1.׼ȷ��

Error=[];%��Ÿ������
for i=1:size(TotalPlanningPath,1)
    IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
    Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
end

figure(2)
hold on
plot(Stotal,Error,'Color',[0,0.75,0.75],'LineWidth',2)
title('����ƫ�������·�̱仯ͼ','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('����ƫ�����/m','FontSize',14)
set(gcf,'color','w')
hold on

%(1)��·��������ο�·������������ֵ
maxError=max(Error);
%(2)��·��������ο�·���������ƽ��ֵ
meanError=mean(Error);


% 2.ƽ˳��
figure(3)
% subplot(3,1,3)
hold on
plot(Stotal,CurvatureOfTotalTentacle,'Color',[0,0.75,0.75],'LineWidth',2)
title('������·�̱仯ͼ','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('����/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on
%(1)��·���������ʾ���ֵ��ƽ��ֵ
meanAbsCurvature=mean(abs(CurvatureOfTotalTentacle));
%(2)��·���������ʵ�������ֵ��ƽ��ֵ
meanAbsC=mean(abs(COfTotalTentacle));

maxCurvature=max(abs(CurvatureOfTotalTentacle));
maxC=max(abs(COfTotalTentacle));


figure(4)
% subplot(3,1,3)
hold on
plot(Stotal,COfTotalTentacle,'Color',[0,0.75,0.75],'LineWidth',2)
title('���ʵ�����·�̱仯ͼ','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('����/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on


% save('Astar_CC_data', 'TotalPlanningPath', 'Stotal', 'Error','CurvatureOfTotalTentacle','COfTotalTentacle');







