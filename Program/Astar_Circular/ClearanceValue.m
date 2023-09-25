function [xgnv,ygnv,Lcknv,rounv] = ClearanceValue(xg,yg,rou,deltaL,newobstacle,obstacle,Vx,EffectiveNum)
%����clearancevalue ��û�п���û�п��д�����L0�������ٵ����Σ���������

%------------------------------------�����ϰ���----------------------------------
% 
% plot(obstacle(:,1),obstacle(:,2),'.k','LineWidth',5,'MarkerSize',10) 
% 
% alpha=0:pi/40:2*pi;
% for i=1:size(obstacle,1)
%     x=obstacle(i,1)+2*cos(alpha);
%     y=obstacle(i,2)+2*sin(alpha);
%     hold on
%     plot(x,y,'k','LineWidth',1)   
% end
% 
% hold on

%-----------------------------------------������ײ����Lc------------------------------------
% Ls=10;%�ƶ���ȫ����
Ls=6;
a=1.5;%�ƶ����ٶ�
Lc=Ls+Vx^2/(2*a);%�ɷŵ���������,�����Ͳ���Ҫ���Lc
%Lc=Ls+Vx^2/a

%-------------------------------------���������Ԥ�����Lck---------------------------------
Lck=[];%��Ÿ�����Ԥ�����
for k=1:size(xg,2)
    Lck=[Lck,Lc];
end

%----------------------�жϸ��������Ƿ�����ײ�㣬����У�����ش��뵽��ײ��ľ���-----------
L0=[];%��Ÿ��������ش������һ����ײ��ľ���

hold on

for k=1:size(xg,2) %�жϵ�k������ 
    flag=0;%��־����flagΪ0��ʾ������û����ײ�㣬flagΪ1��ʾ����������ײ�㣬��ʼ��Ϊ0
    for i=1:EffectiveNum(k)%�жϵ�k�������ϵ�i�����Ƿ���ײ
        for j=1:size(newobstacle,1)%�ж����j���ϰ����Ƿ���ײ
            if (sqrt((xg(i,k)-newobstacle(j,1))^2+(yg(i,k)-newobstacle(j,2))^2)<4.5)%���ΰ�ȫ������Ϊ4.5����ΪС�ڴ˾��뼴������ײ
                flag=1;%����ײ��
                break
            end
        end
        if flag==1;
            break;
        end
        for j=1:size(obstacle,1)%�ж����j���ϰ����Ƿ���ײ
            if (sqrt((xg(i,k)-obstacle(j,1))^2+(yg(i,k)-obstacle(j,2))^2)<8)%���ΰ�ȫ������Ϊ8����ΪС�ڴ˾��뼴������ײ
                flag=1;%����ײ��
                break
            end
        end
        if flag==1;
            break;
        end
    end
    
    if flag==1
        L0=[L0,deltaL*(i-1)];
    else
        L0=[L0,deltaL*i];%��ʱi=EffectiveNum(k)
    end
%     hold on
%     plot(circlexg(:,k),circleyg(:,k),'k')
end


%---------------------------�������-------------------------------------
%xgnv,ygnv��ſ��еĴ�������
xgnv=xg;
ygnv=yg;
Lcknv=Lck;
rounv=rou;

IndexOfUnnaviagable=[];%��Ų����еĴ������

for i=1:length(L0)
    if L0(i)<Lc
        IndexOfUnnaviagable=[IndexOfUnnaviagable,i];
    end
end

%��ȥ�����еĴ���
xgnv(:, IndexOfUnnaviagable)=[];
ygnv(:, IndexOfUnnaviagable)=[];
Lcknv(:, IndexOfUnnaviagable)=[];
rounv(:, IndexOfUnnaviagable)=[];
%////////////////////////////////////////////////////////////////////////////////////////////////////////////////

end






