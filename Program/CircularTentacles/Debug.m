clear,clc
% figure

% �ο�·��
% path=[60,70;50,70;40,70;30,70;20,70;10,70;10,60;10,50;10,40;10,30];%ת������
path=[80,20;70,20;60,20;50,20;40,20;30,20;20,20;10,20;0,20];%����/��������
% path=[130,100;120,100;110,100;100,100;90,100;80,100;70,100;60,100;50,100;40,100;40,90;40,80;40,70;40,60;40,50;40,40;30,40;20,40;20,30;20,20];%A*ģ��
[ReferencePath,deltaP]  = interplotation(path);

% ���ReferencePath
figure(1)
plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',1.5)
hold on

obstacle=[30,20.5];%��������
% obstacle=[0,100];%ת������
% obstacle=[20,50;30,50;40,30;50,50;50,60;90,100.5];%A*ģ��
newobstacle=[50,40];

%�ֲ�·���滮
Vx=3; L=2.7; amax=4; x0=0; y0=20; fai=0; rou0=0; CofBestTentacle=0; CurrentStotal=0; 
%��ʼ����״̬�� CofBestTentacle�ǵ�ǰ�������Ŵ�������ʱ仯�ʣ���ֵ��Ϊ0
% deltamax=pi/6;%ǰ�����ת��
deltamax=35/180*pi;

deltaT=0.1; 
% deltaT=0.1;%�滮���� ��λΪs


CurvatureOfTotalTentacle=rou0;%����·����������,�����ֵrou0
Stotal=0;%����·��������·�̣������ֵ0
COfTotalTentacle=0;%����·���������ʱ仯��,�����ֵ0
TotalPlanningPath=[x0,y0];%��������滮·����������,�ȴ����������
count=0;%��ǰ���ڵ�count���滮����

load('circulartentaclesdata.mat')
%����滮����ϵ��Բ����������

while(1)
    
    if sqrt((x0-ReferencePath(size(ReferencePath,1),1))^2+(y0-ReferencePath(size(ReferencePath,1),2))^2)<2
        %�ѵ��յ�����
        break;
    else
        %��ʼ�滮
        
        %ȫ������ϵ�´�������
        xg=cos(fai)*xp-sin(fai)*yp+x0;
        yg=sin(fai)*xp+cos(fai)*yp+y0;

        %ѡ�����д��룬��������Vclearance
        [xgnv,ygnv,Lcknv,rounv] = ClearanceValue(xg,yg,rou,deltaL,newobstacle,obstacle,Vx,EffectiveNum);

        %������д����Vpath��Vcurvature
        [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
        Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);
        Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle);   

        %ѡ�����Ŵ�����Ϊ�켣
        a1=1;   a2=0;   
        Farbe='b';
        Vcombined=a1*Vdist+a2*Valpha;
        [VcombinedMin,IndexMin]=min(Vcombined);

        %ִ�иô��룬���³�ʼ״̬x0,y0,fai,rou0,CurrentC

        %����ÿһ������ǰ��ʻ�����
        S=Vx*deltaT;%�滮�������ع켣��ʻ·��
        SIndexTentacle=floor(S/deltaL);
        if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
            SIndexTentacle=SIndexTentacle+1;
        end
        
%         %������·���������ʱ仯����
%         COfTotalTentacle=[COfTotalTentacle,CofBestTentacle];

        %���Ŵ����������Curvature
        for i=1:SIndexTentacle
            CurvatureOfTotalTentacle=[CurvatureOfTotalTentacle,rounv(IndexMin)];
        end 
        
        if rounv(IndexMin)==rou0 %���ʲ�ͻ��
            for i=1:SIndexTentacle
                COfTotalTentacle=[COfTotalTentacle,0];
            end
        elseif rounv(IndexMin)>rou0 %����ͻ��
            for i=1:SIndexTentacle
                if i==1 %ͻ���
                    COfTotalTentacle=[COfTotalTentacle,1000000];
                else
                     COfTotalTentacle=[COfTotalTentacle,0];
                end
            end
        else %����ͻ��
            for i=1:SIndexTentacle
                if i==1 %ͻ���
                     COfTotalTentacle=[COfTotalTentacle,-1000000];
                else
                     COfTotalTentacle=[COfTotalTentacle,0];
                end
            end  
        end

        %���Ŵ�������Ӧ����·��Stotal
        for i=1:SIndexTentacle
            Stotal=[Stotal,CurrentStotal+i*deltaL];
        end

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
        rou0=rounv(IndexMin);
        
        %���µ�ǰλ����·������CurrentStotal
        CurrentStotal=CurrentStotal+SIndexTentacle*deltaL;

        count=count+1;%��count���滮���ڽ���
    end
end

%'Color',[1,0.6,0]
%'Color',[0,0.45,0.74]
figure(1)
plot(TotalPlanningPath(:,1),TotalPlanningPath(:,2),Farbe,'LineWidth',2)
title('�ֲ�·���滮ͼ','FontSize',16);
xlabel('x/m','FontSize',14)
ylabel('y/m','FontSize',14)
set(gcf,'color','w')
axis equal
hold on


%���չ滮·������ָ��

%1.׼ȷ��

Error=[];%��Ÿ������
for i=1:size(TotalPlanningPath,1)
    IndexofReferencePoint=dsearchn(ReferencePath,[TotalPlanningPath(i,1),TotalPlanningPath(i,2)]);
    Error=[Error,sqrt(((ReferencePath(IndexofReferencePoint,1))-(TotalPlanningPath(i,1)))^2+((ReferencePath(IndexofReferencePoint,2))-(TotalPlanningPath(i,2)))^2)]; 
end

figure(2)
plot(Stotal,Error,Farbe,'LineWidth',2)
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
plot(Stotal,CurvatureOfTotalTentacle,Farbe,'LineWidth',2)
title('������·�̱仯ͼ','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('����/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on

%(1)��·���������ʾ���ֵ�����ֵ
maxCurvature=max(abs(CurvatureOfTotalTentacle));
%(2)��·���������ʾ���ֵ��ƽ��ֵ
meanCurvature=mean(abs(CurvatureOfTotalTentacle));

% (3)��·���������ʵ�������ֵ�����ֵ
% maxC=max(abs(COfTotalTentacle));
% (4)��·���������ʵ�������ֵ��ƽ��ֵ
% meanC=sum(abs(COfTotalTentacle))/count;
 
figure(4)
% % subplot(3,1,3)
plot(Stotal,COfTotalTentacle,Farbe,'LineWidth',2)
title('���ʵ�����·�̱仯ͼ','FontSize',16);
xlabel('s/m','FontSize',14)
ylabel('����/m^-^1','FontSize',14)
set(gcf,'color','w')
hold on










