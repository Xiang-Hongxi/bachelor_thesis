% clear,clc
% figure

% %�ο�·��

path=[60,20;50,20;40,20;30,20;20,20];%���ڳ�ͼ
path1=[30,20;20,20];
[ReferencePath,deltaP]  = interplotation(path);

%���ReferencePath
% plot(ReferencePath(:,1),ReferencePath(:,2),'r','LineWidth',2)
% hold on
plot(path1(:,1),path1(:,2),'r','LineWidth',2)
hold on


%�ֲ�·���滮
% Vx=6; L=3; amax=4; x0=20; y0=10; fai=pi/2; rou0=0; %��ʼ����״̬
% deltamax=pi/6;%ǰ�����ת��
deltamax=30/180*pi;

Vx=3;x0=20; y0=19;fai=0.2;%

deltaT=1; 
% deltaT=0.1;%�滮���� ��λΪs

% obstacle=[10,45;22,33];%�����ƶ��ϰ��ÿһ�滮����obstacle���ܻ�䣬����Ӧ����whileѭ����
% obstacle=[20,65];%��������
% obstacle=[20,60];%��������
obstacle=[0,100];%ת������
% obstacle=[20,50;30,50;40,30;50,50;50,60];%A*ģ��

while(1)
    
    %���ɴ���
    [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0); 
    
    %ѡ�����д��룬��������Vclearance
    [xgnv,ygnv,Cnv,L0,Vclearance,Lcknv] = ClearanceValue(xg,yg,C,deltaL,obstacle,Vx);
    
    %������д����Vpath��Vcurvature
%     Lc=25;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Lc=18;
%     Vpath=PathValue(xgnv,ygnv,ReferencePath,Lc,deltaL,deltaP);
    Vpath=NewPathValue(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP);
    Vcurvature = CurvatureValue(Cnv);
    
    %ѡ�����Ŵ�����Ϊ�켣
    a1=0;   a2=1;    a3=0;
    Vcombined=a1*Vclearance+a2*Vpath+a3*Vcurvature;
    [VcombinedMin,IndexMin]=min(Vcombined);
    
    
    %ִ�иô��룬���³�ʼ״̬x0,y0,fai,rou0
    
    %����x0,y0,
    S=Vx*deltaT;%�滮�������ع켣��ʻ·��
    SIndexTentacle=floor(S/deltaL);
    if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
        SIndexTentacle=SIndexTentacle+1;
    end
    x0=xgnv(SIndexTentacle+1,IndexMin);
    y0=ygnv(SIndexTentacle+1,IndexMin);
    
    %���ƴ���ִ�н��
    hold on
%     plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[1,0.6,0],'LineWidth',2)
    plot(xgnv(1:SIndexTentacle+1,IndexMin),ygnv(1:SIndexTentacle+1,IndexMin),'Color',[0,0.45,0.74],'LineWidth',2)
    
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
    rou0=rou0+Cnv(IndexMin)*SIndexTentacle*deltaL;

%     close
end

















