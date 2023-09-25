clear,clc

%δ��������ת��
%����������Ĵ���

n=16;%�ٶ�������
roucircle=1.35;%����
V_=(2.8589+3.497)/2;%////////////////////////////////////
% V_=(5.5319+6.2437)/2;
amax=4;
deltamax=35*pi/180;
L=2.7;


j=7;%////////////////////////////////////
% j=11;
q=(j-1)/(n-1);
l=10+35*q^2;%����ഥ�뻡��

% roumax=min(amax/(V_^2),(tan(deltamax)/L)/sqrt(1+0.25*(tan(deltamax))^2));
roumax=min(amax/(V_^2),(tan(deltamax)/L)/sqrt(1+0*(tan(deltamax))^2));

Rj=1/roumax;

deltaL=Rj*pi/2000; 
% pi/2000Ϊ��һ�������������нǣ�Ϊ��������������д���deltaL��һ��

x=[];
y=[];
%��Ÿ�����x,y���꣬ÿ�б�ʾһ������
NumOfRowsForCircleMatrix=floor((l+20)/deltaL)+1;
%x,y����������������루��21�����룩���������ݸ���
EffectiveNum=[];%��Ÿ�������Ч�������
rou=[];%���ֵ�k�����������

for k=1:41
    if k<=20
        Rk=roucircle^(k-1)*Rj;
        rouk=-1/Rk;
        lk=l+20*sqrt((k-1)/20);
        
        Fai1=pi;
        deltaFaik=lk/Rk;%��k�����뻡����Ӧ��Բ�Ľ�
        Fai2=Fai1-deltaFaik;
        Fai=(Fai1:-deltaL/Rk:Fai2)';%-deltaLcircle/Rk��Ӧ�Ļ�����ΪdeltaLcircle,ת��ʹ��Ϊһ��
        
        circlex0=Rk;
        circley0=0;%Բ������
        
        xk=circlex0+Rk*cos(Fai);
        yk=circley0+Rk*sin(Fai);%ʵ�ʴ�������
        
        EffectiveNum=[EffectiveNum,size(xk,1)];
        %circlex��circley��ÿ��ǰEffectiveNum(��)�����ִ�����ʵ�Ĵ�������
        
        for i=EffectiveNum(k)+1:NumOfRowsForCircleMatrix
            xk(i)=inf;
            yk(i)=inf;
        end
        %�������ݣ���֤����ά��һ��
        
        x=[x,xk];
        y=[y,yk];
        %�����k����������
        
    elseif k>21
        Rk=roucircle^(41-k)*Rj;
        rouk=1/Rk;
        lk=l+20*sqrt((41-k)/20);
        Fai1=0;
        deltaFaik=lk/Rk;%��k�����뻡����Ӧ��Բ�Ľ�
        Fai2=Fai1+deltaFaik;
        Fai=(Fai1:deltaL/Rk:Fai2)';%-deltaLcircle/Rk��Ӧ�Ļ�����ΪdeltaLcircle��ת��ʹ��Ϊһ��
        
        circlex0=-Rk;
        circley0=0;%Բ������ 
        
        xk=circlex0+Rk*cos(Fai);
        yk=circley0+Rk*sin(Fai);%ʵ�ʴ�������
        
        EffectiveNum=[EffectiveNum,size(xk,1)];
        %circlex��circley��ÿ��ǰEffectiveNum(��)�����ִ�����ʵ�Ĵ�������
        
        for i=EffectiveNum(k)+1:NumOfRowsForCircleMatrix
            xk(i)=inf;
            yk(i)=inf;
        end
        %�������ݣ���֤����ά��һ��
        
        x=[x,xk];
        y=[y,yk];
        %�����k����������
        
    else %k=21
        rouk=0;
        lk=l+20;%�м�ֱ�ߴ���ĳ���
        xk=zeros(NumOfRowsForCircleMatrix,1);
        for i=1:NumOfRowsForCircleMatrix
            yk(i,1)=(i-1)*deltaL;
        end
        x=[x,xk];
        y=[y,yk];
        %�����k����������
        EffectiveNum=[EffectiveNum,size(xk,1)];
    end 
    rou=[rou,rouk];
end

xp=cos(-pi/2)*x-sin(-pi/2)*y;
yp=sin(-pi/2)*x+cos(-pi/2)*y;
%˳ʱ����ת90�ȣ���֤����ǰ�������ǳ�������,�±�p��ʾ�滮����ϵ

for i=1:size(xp,2)
    xi=xp(1:EffectiveNum(i),i);
    yi=yp(1:EffectiveNum(i),i);
%     plot(circlexi,circleyi)
    plot(xi,yi,'Color',[0 0.45 0.74])  
    hold on
end
% 

save('circulartentaclesdata', 'xp', 'yp', 'EffectiveNum','deltaL','rou')

grid on
axis equal
