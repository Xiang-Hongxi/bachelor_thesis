% clear,clc
% figure

% �ο�·��
% path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%ת������
% path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%����/��������
% path=[130,100;120,100;110,100;100,100;90,100;80,100;70,100;60,100;50,100;40,100;40,90;40,80;40,70;40,60;40,50;40,40;30,40;20,40;20,30;20,20];%A*ģ��
[ReferencePath,deltaP]  = interplotation(path);

% ���ReferencePath
% figure(1)
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5)
% hold on

% obstacle=[30,20.5];%��������
% obstacle=[0,100];%ת������
% obstacle=[20,50;30,50;40,30;50,50;50,60;90,100.5];%A*ģ��
newobstacle=[52,70;110,119.5];

%�ֲ�·���滮
Vx=3; L=2.7; amax=4; x0=10; y0=10; fai=pi/2; rou0=0; CofBestTentacle=0; CurrentStotal=0; 
%��ʼ����״̬�� CofBestTentacle�ǵ�ǰ�������Ŵ�������ʱ仯�ʣ���ֵ��Ϊ0
% deltamax=pi/6;%ǰ�����ת��
deltamax=35/180*pi;

deltaT=0.1; 
% deltaT=0.1;%�滮���� ��λΪs


CurvatureOfTotalTentacle=rou0;%����·����������,�����ֵrou0
Stotal=0;%����·��������·�̣������ֵ0
COfTotalTentacle=[];%����·���������ʱ仯��
TotalPlanningPath=[x0,y0];%��������滮·����������,�ȴ����������
count=0;%��ǰ���ڵ�count���滮����


while(1)
    
    if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
        %�ѵ��յ�����
        break;
    else
        %��ʼ�滮
        
        %���ɴ���
        [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0); 

        %ѡ�����д��룬��������Vclearance
        [xgnv,ygnv,Cnv,L0,Vclearance,Lcknv] = ClearanceValue(xg,yg,C,deltaL,newobstacle,obstacle,Vx);

        %������д����Vpath��Vcurvature
        %Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
        [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
        Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   
%         Vpath=NewPathValue(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vcurvature = CurvatureValue(Cnv);
        Vcurvaturedot = CurvatureDotValue(Cnv,CofBestTentacle);
        Vsumcurvature = SumCurvatureValue(rou0,Cnv,Vx,deltaT,deltaL);

        %ѡ�����Ŵ�����Ϊ�켣
%         a1=0;   a2=0.5622;   a3=0.2180;  a4=0.2197; a5=0; a6=0;
%         a1=0;   a2=0.5622;   a3=0.2180;  a4=0; a5=0; a6=0;
%         a1=0;   a2=0.6289;   a3=0.2832;  a4=0.0879; a5=0; a6=0;
          a1=0;   a2=0.5632;   a3=0.3436;  a4=0.0932; a5=0; a6=0;
%         a1=0;   a2=1;   a3=0;  a4=0; a5=0; a6=0;
%         Farbe='b';
%         Vcombined=a1*Vclearance+a2*Vpath+a3*Vcurvature+a4*Vcurvaturedot+a5*Vsumcurvature;
        Vcombined=a1*Vclearance+a2*Vdist+a3*Valpha+a4*Vcurvature+a5*Vcurvaturedot+a6*Vsumcurvature;
        [VcombinedMin,IndexMin]=min(Vcombined);

        %ִ�иô��룬���³�ʼ״̬x0,y0,fai,rou0,CurrentC

        %����ÿһ������ǰ��ʻ�����
        S=Vx*deltaT;%�滮�������ع켣��ʻ·��
        SIndexTentacle=floor(S/deltaL);
        if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
            SIndexTentacle=SIndexTentacle+1;
        end

        %���Ŵ������ʱ仯��
        CofBestTentacle=Cnv(IndexMin);

        %������·���������ʱ仯����
        COfTotalTentacle=[COfTotalTentacle,CofBestTentacle];

        %���Ŵ����������Curvature
        for i=1:SIndexTentacle
            CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rou0+i*deltaL*CofBestTentacle];
        end

        %���Ŵ�������Ӧ����·��Stotal
        for i=1:SIndexTentacle
            Stotal=[Stotal,CurrentStotal+i*deltaL];
        end
% 
%         %���������Ŵ�����ʻS�Ĺ����и�����������·�̵ı仯ͼ
%         plot(Stotal,CurvatureOfTotalTentacle*100,'Color',[0,0.45,0.74])

        %���ƴ���ִ�н��
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
        rou0=rou0+CofBestTentacle*SIndexTentacle*deltaL;

        %���µ�ǰλ����·������CurrentStotal
        CurrentStotal=CurrentStotal+SIndexTentacle*deltaL;

        count=count+1;%��count���滮���ڽ���
    end
end

%'Color',[1,0.6,0]
%'Color',[0,0.45,0.74]
figure(1)
hold on
% plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),Farbe,'LineWidth',2)
plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),'Color',[0,0.45,0.74],'LineWidth',2)
title('�ֲ�·���滮ͼ','FontSize',16);
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



%���չ滮·������ָ��

%1.׼ȷ��

Error=[];%��Ÿ������
for i=1:size(TotalPlanningPath,1)
    IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
    Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
end

figure(2)
% plot(Stotal,Error,Farbe,'LineWidth',2)
plot(Stotal,Error,'Color',[0,0.45,0.74],'LineWidth',2)
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
% plot(Stotal,CurvatureOfTotalTentacle,Farbe,'LineWidth',2)
plot(Stotal,CurvatureOfTotalTentacle,'Color',[0,0.45,0.74],'LineWidth',2)
title('������·�̱仯ͼ','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('����/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on
%(1)��·���������ʾ���ֵ��ƽ��ֵ
meanCurvature=mean(abs(CurvatureOfTotalTentacle));
%(2)��·���������ʵ�������ֵ��ƽ��ֵ
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
title('���ʵ�����·�̱仯ͼ','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('���ʵ���/m^-^2','FontSize',14)
set(gcf,'color','w')
hold on


%save('Astar_clothoid_data_w1=1', 'TotalPlanningPath', 'Stotal', 'Error','CurvatureOfTotalTentacle','COfTotalTentaclebydot')

%save('Astar_clothoid_data_w=0.5632_0.3436_0.0932', 'TotalPlanningPath', 'Stotal', 'Error','CurvatureOfTotalTentacle','COfTotalTentaclebydot')






