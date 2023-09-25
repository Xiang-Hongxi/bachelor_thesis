function  Vpath=NewPathValue(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP)

%------------------------Ѱ�Ҵ�����㼰���ӦReferencePath�ϵĵ�------------------------------

CurrentPosition=[xgnv(1,1),ygnv(1,1)];
StartIndexofRP= dsearchn(ReferencePath,CurrentPosition);
%StartIndexofRPΪ���뵱ǰλ�������ReferencePath�ϵĵ�����

%-----------------------------Ѱ����Ӧ��Lcknv���Ĺ켣�ʹ����յ�-----------------------------------
LIndexRP=[];
for k=1:size(xgnv,2)
    LIndexRPk=floor(Lcknv(k)/deltaP);
    if abs(LIndexRPk*deltaP-Lcknv(k))>abs((LIndexRPk+1)*deltaP-Lcknv(k))
        LIndexRPk=LIndexRPk+1;
    end
    LIndexRP=[LIndexRP,LIndexRPk];
end
EndIndexofRP=StartIndexofRP+LIndexRP;
%EndIndexofRPΪ��Ӧ��ReferencePath�ϵ��յ�����
LcPath=(EndIndexofRP-StartIndexofRP)*deltaP;
%ʵ���ϵ�Lc���в�ֵ���

LIndexTentacle=[];
for k=1:size(xgnv,2)
    LIndexTentaclek=floor(Lcknv(k)/deltaL);
    if abs(LIndexTentaclek*deltaL-Lcknv(k))>abs((LIndexTentaclek+1)*deltaL-Lcknv(k))
        LIndexTentaclek=LIndexTentaclek+1;
    end
    LIndexTentacle=[LIndexTentacle,LIndexTentaclek];
end
StartIndexofTentacle=1;
EndIndexofTentacle=StartIndexofTentacle+LIndexTentacle;
%EndIndexofTentacleΪ��Ӧ�Ĵ����ϵ��յ�����
LcTentacle=(EndIndexofTentacle-StartIndexofTentacle)*deltaL;
%ʵ���ϵ�Lc���в�ֵ���


%-----------------���³�������ڻ�ͼʾ������д���Ͳο�·���ϵĵ�--------------------------------
% plot(ReferencePath(:,1),ReferencePath(:,2),'r')
% hold on
%���ReferencePath
% plot(xgnv(1,1),ygnv(1,1),'o','color',[0,0.45,0.74],'LineWidth',2) 
% hold on
% % �����ǰλ��
% plot(ReferencePath(StartIndexofRP,1),ReferencePath(StartIndexofRP,2),'o')
% hold on
% % ���ReferencePath���뵱ǰλ������ĵ�
% % 
% % %
% plot(ReferencePath(EndIndexofRP,1),ReferencePath(EndIndexofRP,2),'x')
% hold on
%���ReferencePath�Ͼ���Lck�ĵ�
% % 
% for k=1:size(xgnv,2)
%     plot(xgnv(EndIndexofTentacle(k),k),ygnv(EndIndexofTentacle(k),k),'x')
%     hold on
% end
%����������Ͼ���Lck�ĵ�


%-----------------------------����Vpath-----------------------------------
%б����i-1,i+1������������

a=[];%��Ÿ����д����Ӧ��ľ���
alpha=[];%��Ÿ����д����Ӧ��ļн�

for k=1:size(xgnv,2)
    %���ReferencePath��Lcknv����x��������н�
    if ReferencePath(EndIndexofRP(k),1)==ReferencePath(EndIndexofRP(k)-1,1)
        if ReferencePath(EndIndexofRP(k),2)>ReferencePath(EndIndexofRP(k)-1,2)
            AngleOfRP=pi/2;
        else
            AngleOfRP=-pi/2;
        end
    elseif ReferencePath(EndIndexofRP(k),1)>ReferencePath(EndIndexofRP(k)-1,1)
        SlopeOfRP=(ReferencePath(EndIndexofRP(k),2)-ReferencePath(EndIndexofRP(k)-1,2))/(ReferencePath(EndIndexofRP(k),1)-ReferencePath(EndIndexofRP(k)-1,1));
        AngleOfRP=atan(SlopeOfRP);
    else
        SlopeOfRP=(ReferencePath(EndIndexofRP(k),2)-ReferencePath(EndIndexofRP(k)-1,2))/(ReferencePath(EndIndexofRP(k),1)-ReferencePath(EndIndexofRP(k)-1,1));
        if SlopeOfRP>0
            AngleOfRP=atan(SlopeOfRP)-pi;
        else
            AngleOfRP=atan(SlopeOfRP)+pi;
        end
    end
    
    %���������Lcknv����x��������н�
    if xgnv(EndIndexofTentacle(k),k)==xgnv(EndIndexofTentacle(k)-1,k)
        if ygnv(EndIndexofTentacle(k),k)>ygnv(EndIndexofTentacle(k)-1,k)
            AngleOfTentacle=pi/2;
        else
            AngleOfTentacle=-pi/2;
        end
    elseif xgnv(EndIndexofTentacle(k),k)>xgnv(EndIndexofTentacle(k)-1,k)
        SlopeOfTentacle=((ygnv(EndIndexofTentacle(k),k)-ygnv(EndIndexofTentacle(k)-1,k)))/((xgnv(EndIndexofTentacle(k),k)-xgnv(EndIndexofTentacle(k)-1,k)));
        AngleOfTentacle=atan(SlopeOfTentacle); 
    else
        SlopeOfTentacle=((ygnv(EndIndexofTentacle(k),k)-ygnv(EndIndexofTentacle(k)-1,k)))/((xgnv(EndIndexofTentacle(k),k)-xgnv(EndIndexofTentacle(k)-1,k)));
        if SlopeOfTentacle>0
            AngleOfTentacle=atan(SlopeOfTentacle)-pi;
        else
            AngleOfTentacle=atan(SlopeOfTentacle)+pi;
        end
    end
    
    %������߼н�
    alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)];
    
    %������߾���
    a=[a,sqrt((xgnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),1))^2+(ygnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),2))^2)];
    
end


% Calpha=3;%�������ɵ���
Calpha=0;
vdist=a+Calpha*alpha;
% vdist=Calpha*alpha;
vmin=min(vdist);
vmax=max(vdist);

Vpath=(vdist-vmin)/(vmax-vmin);


end

