function  Vpath=PathValue(circlexgnv,circleygnv,ReferencePath,Lc,deltaLcircle,deltaP)
%����������У��켣��PathValue
%���ҵ����복����ǰλ�������ReferencePath�ϵĵ���Ϊ�ο��켣�ϵ���㣬���ҵ������ײ����Lc�ϵĵ�

%Ŀǰֻ�����˼н�alpha����������


%if ReferencePath==[]�������ѵ��յ㣬�����������ж�

%-----------------------------Ѱ�Ҳο��켣���-----------------------------------

% distance=sqrt((xgnv(1,1)-ReferencePath(1,1))^2+(ygnv(1,1)-ReferencePath(1,2))^2);%�����һ�����distance
% 
% for i=2:size(ReferencePath,1)
%     distance=[distance,sqrt((xgnv(1,1)-ReferencePath(i,1))^2+(ygnv(1,1)-ReferencePath(i,2))^2)];
%     %xgnv(1,1),ygnv(1,1)�ֱ��ǳ�����ǰλ��
%     if distance(i)>distance(i-1)
%         break %distanceӦ�����ȼ�������һ����������˵��ǰһֵΪ��Сֵ
%     end
% end

CurrentPosition=[circlexgnv(1,1),circleygnv(1,1)];
StartIndexofRP= dsearchn(ReferencePath,CurrentPosition);
%StartIndexofRPΪ���뵱ǰλ�������ReferencePath�ϵĵ�����

%-----------------------------Ѱ����Ӧ��Lc���Ĺ켣�ʹ����յ�-----------------------------------

LIndexRP=floor(Lc/deltaP);
if abs(LIndexRP*deltaP-Lc)>abs((LIndexRP+1)*deltaP-Lc)
    LIndexRP=LIndexRP+1;
end
EndIndexofRP=StartIndexofRP+LIndexRP;
%EndIndexofRPΪ��Ӧ��ReferencePath�ϵ��յ�����

LcPath=(EndIndexofRP-StartIndexofRP)*deltaP;
%ʵ���ϵ�Lc���в�ֵ���

LIndexTentacle=floor(Lc/deltaLcircle);
if abs(LIndexTentacle*deltaLcircle-Lc)>abs((LIndexTentacle+1)*deltaLcircle-Lc)
    LIndexTentacle=LIndexTentacle+1;
end
StartIndexofTentacle=1;
EndIndexofTentacle=StartIndexofTentacle+LIndexTentacle;
%EndIndexofTentacleΪ��Ӧ�Ĵ����ϵ��յ�����

LcTentacle=(EndIndexofTentacle-StartIndexofTentacle)*deltaLcircle;
%ʵ���ϵ�Lc���в�ֵ���

%-----------------���³�������ڻ�ͼʾ������д���Ͳο�·���ϵĵ�--------------------------------
% hold on
% plot(ReferencePath(:,1),ReferencePath(:,2),'r')
% hold on
% %���ReferencePath
% plot(circlexgnv(1,1),circleygnv(1,1),'o') 
% hold on
% %�����ǰλ��
% plot(ReferencePath(StartIndexofRP,1),ReferencePath(StartIndexofRP,2),'o')
% hold on
% %���ReferencePath���뵱ǰλ������ĵ�
% plot(ReferencePath(EndIndexofRP,1),ReferencePath(EndIndexofRP,2),'x')
% hold on
% %���ReferencePath�Ͼ���Lc�ĵ�
% 
% for i=1:size(circlexgnv,2)
%     plot(circlexgnv(EndIndexofTentacle,i),circleygnv(EndIndexofTentacle,i),'x')
%     hold on
% end
%����������Ͼ���Lc�ĵ�

% hold on
% plot(ReferencePath(StartIndexofRP+1,1),ReferencePath(StartIndexofRP+1,2),'ro')
% axis equal 

%-----------------------------����Vpath-----------------------------------
%б����i-1,i+1������������

a=[];%��Ÿ����д����Ӧ��ľ���
alpha=[];%��Ÿ����д����Ӧ��ļн�
SlopeOfRP=(ReferencePath(EndIndexofRP+1,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP+1,1)-ReferencePath(EndIndexofRP-1,1));
% δ�����յ㸽�����ܻᳬ��������
% if SlopeOfRP<0
%     AngleOfRP=atan(SlopeOfRP)+pi;
% else
%     AngleOfRP=atan(SlopeOfRP);
% end

if ReferencePath(EndIndexofRP+1,1)==ReferencePath(EndIndexofRP-1,1)
    if ReferencePath(EndIndexofRP+1,2)>ReferencePath(EndIndexofRP-1,2)
        AngleOfRP=pi/2;
    else
        AngleOfRP=-pi/2;
    end
elseif ReferencePath(EndIndexofRP+1,1)>ReferencePath(EndIndexofRP-1,1)
    SlopeOfRP=(ReferencePath(EndIndexofRP+1,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP+1,1)-ReferencePath(EndIndexofRP-1,1));
    AngleOfRP=atan(SlopeOfRP);
else
    SlopeOfRP=(ReferencePath(EndIndexofRP+1,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP+1,1)-ReferencePath(EndIndexofRP-1,1));
    if SlopeOfRP>0
        AngleOfRP=atan(SlopeOfRP)-pi;            
    else
        AngleOfRP=atan(SlopeOfRP)+pi;
    end
end


for i=1:size(circlexgnv,2)
    hold on 
%     plot(xgnv(:,i),ygnv(:,i),'y')
    a=[a,sqrt((circlexgnv(EndIndexofTentacle,i)-ReferencePath(EndIndexofRP,1))^2+(circleygnv(EndIndexofTentacle,i)-ReferencePath(EndIndexofRP,2))^2)];
   
%     SlopeOfTentacle=(ygnv(EndIndexofTentacle+1,i)-ygnv(EndIndexofTentacle-1,i))/(xgnv(EndIndexofTentacle+1,i)-xgnv(EndIndexofTentacle-1,i));
%     if SlopeOfTentacle<0
%         AngleOfTentacle=atan(SlopeOfTentacle)+pi;
%     else
%         AngleOfTentacle=atan(SlopeOfTentacle); 
%     end
%     alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)];

    if circlexgnv(EndIndexofTentacle+1,i)==circlexgnv(EndIndexofTentacle-1,i)
        if circleygnv(EndIndexofTentacle+1,i)>circleygnv(EndIndexofTentacle-1,i)
            AngleOfTentacle=pi/2;
        else
            AngleOfTentacle=-pi/2;
        end
    elseif circlexgnv(EndIndexofTentacle+1,i)>circlexgnv(EndIndexofTentacle-1,i)
        SlopeOfTentacle=((circleygnv(EndIndexofTentacle+1,i)-circleygnv(EndIndexofTentacle-1,i)))/((circlexgnv(EndIndexofTentacle+1,i)-circlexgnv(EndIndexofTentacle-1,i)));
        AngleOfTentacle=atan(SlopeOfTentacle); 
    else
        SlopeOfTentacle=((circleygnv(EndIndexofTentacle+1,i)-circleygnv(EndIndexofTentacle-1,i)))/((circlexgnv(EndIndexofTentacle+1,i)-circlexgnv(EndIndexofTentacle-1,i)));
        if SlopeOfTentacle>0
            AngleOfTentacle=atan(SlopeOfTentacle)-pi;
        else
            AngleOfTentacle=atan(SlopeOfTentacle)+pi;
        end
    end
    alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)];
    
end

% Calpha=3;%�������ɵ���
Calpha=0;
vdist=a+Calpha*alpha;
% vdist=Calpha*alpha;
vmin=min(vdist);
vmax=max(vdist);

Vpath=(vdist-vmin)/(vmax-vmin);


end

