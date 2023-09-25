function  [EndIndexofRP,EndIndexofTentacle]=FindComparePoint(xgnv,ygnv,ReferencePath,Lcknv,deltaL,deltaP)

%------------------------Ѱ�Ҵ�����㼰���ӦReferencePath�ϵĵ�------------------------------

CurrentPosition=[xgnv(1,1),ygnv(1,1)];
StartIndexofRP= dsearchn(ReferencePath,CurrentPosition);
%StartIndexofRPΪ���뵱ǰλ�������ReferencePath�ϵĵ�����

%------------------------------------�ж�ʣ��RP�Ƿ񹻳�----------------------------------------

for k=1:size(xgnv,2)
    if (size(ReferencePath,1)-StartIndexofRP)*deltaP<Lcknv(k)
        Lcknv(k)=(size(ReferencePath,1)-StartIndexofRP)*deltaP;  
    end
end

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
% ���ReferencePath
% plot(xgnv(1,1),ygnv(1,1),'o','color',[0,0.45,0.74],'LineWidth',2) 
% hold on
% % �����ǰλ��
% plot(ReferencePath(StartIndexofRP,1),ReferencePath(StartIndexofRP,2),'o','color',[0,0.45,0.74],'LineWidth',2)
% hold on
% % ���ReferencePath���뵱ǰλ������ĵ�

% plot(ReferencePath(EndIndexofRP,1),ReferencePath(EndIndexofRP,2),'xr','LineWidth',2,'MarkerSize',8)
% hold on
% ���ReferencePath�Ͼ���Lck�ĵ�

% plot(ReferencePath(EndIndexofRP(31),1),ReferencePath(EndIndexofRP(31),2),'+','color',[0,0.45,0.74],'LineWidth',2)
% hold on
% ���ReferencePath�Ͼ���Lck�ĵ�

% % 
% for k=1:size(xgnv,2)
%     plot(xgnv(EndIndexofTentacle(k),k),ygnv(EndIndexofTentacle(k),k),'x','color',[0,0.45,0.74],'LineWidth',2,'MarkerSize',8)
%     hold on
% end
% %����������Ͼ���Lck�ĵ�


% plot(xgnv(EndIndexofTentacle(31),31),ygnv(EndIndexofTentacle(31),31),'+','color',[0,0.45,0.74],'LineWidth',2)

%����������Ͼ���Lck�ĵ�

end

