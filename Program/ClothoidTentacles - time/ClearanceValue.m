function [xgnv,ygnv,Cnv,L0,Lc] = ClearanceValue(xg,yg,C,deltaL,newobstacle,Vx)
%����clearancevalue ��û�п���û�п��д�����L0�������ٵ����Σ���������

%------------------------------------�����ϰ���----------------------------------
% 
% plot(newobstacle(:,1),newobstacle(:,2),'.k','LineWidth',5,'MarkerSize',10) 
% 
% alpha=0:pi/40:2*pi;
% for i=1:size(newobstacle,1)
%     x=newobstacle(i,1)+2*cos(alpha);
%     y=newobstacle(i,2)+2*sin(alpha);
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

% Lck=Lc;%��Ÿ�����Ԥ�����


%----------------------�жϸ��������Ƿ�����ײ�㣬����У�����ش��뵽��ײ��ľ���-----------

L0=[];%��Ÿ��������ش������һ����ײ��ľ���


for k=1:size(xg,2) %�жϵ�k������
    
    flag=0;%��־����flagΪ0��ʾ������û����ײ�㣬flagΪ1��ʾ����������ײ�㣬��ʼ��Ϊ0
    
    for i=1:size(xg,1)%�жϵ�k�������ϵ�i�����Ƿ���ײ
        
        for j=1:size(newobstacle,1)%�ж����j���ϰ����Ƿ���ײ
            if (sqrt((xg(i,k)-newobstacle(j,1))^2+(yg(i,k)-newobstacle(j,2))^2)<4.5)%���ΰ�ȫ������Ϊ2����ΪС�ڴ˾��뼴������ײ
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
        L0=[L0,deltaL*i];%��ʱi=size(xg,1)
    end
%     hold on
%     plot(xg(:,k),yg(:,k),'k')%�������д���
end


%----------------------------�������,���һ���ϰ�����ײ����С��Lc�Ĵ����ǲ����д���-------------------------------------
%xgnv,ygnv��ſ��еĴ�������
xgnv=xg;
ygnv=yg;
Cnv=C;

IndexOfUnnaviagable=[];%��Ų����еĴ������

for i=1:length(L0)
    if L0(i)<Lc
        IndexOfUnnaviagable=[IndexOfUnnaviagable,i];
    end
end

%��ȥ�����еĴ���
xgnv(:, IndexOfUnnaviagable)=[];
ygnv(:, IndexOfUnnaviagable)=[];
Cnv(:, IndexOfUnnaviagable)=[];
L0(:, IndexOfUnnaviagable)=[];

% %���ƿ��д���
% for k=1:size(xgnv,2)
%     hold on
%     plot(xgnv(:,k),ygnv(:,k),'color',[0.69,0.88,0.90])
% end

end






