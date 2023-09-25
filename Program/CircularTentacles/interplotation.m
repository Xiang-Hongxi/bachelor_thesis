% function accuratepath = interplotation(path)
% %��path���в�ֵ��������˳��Ϊ����㵽�յ�
% 
% %����˳��Ϊ��㵽�յ�
% sizeofpath=size(path);
% n=sizeofpath(1);
% pathfromstartpoint=[];
% for i=n:-1:1
%     pathfromstartpoint=[pathfromstartpoint;path(i,:)];
% end
% 
% 
% %��ֵ
% accu=7; %����������դ�����ļ����accu���㣨��ֵ���ȣ�
% accuratepath=pathfromstartpoint(1,:); %��ֵ���·��,�ȷ������
% iaccu=1; %��ʾaccuratepath��ǰ�к�
% 
% for i=1:n-1;
%     
%     if pathfromstartpoint(i,1)==pathfromstartpoint(i+1,1) %������
%         
%         deltay=(pathfromstartpoint(i+1,2)-pathfromstartpoint(i,2))/(accu+1);
%         while accuratepath(iaccu,2)~=pathfromstartpoint(i+1,2)
%             accuratepath=[accuratepath;accuratepath(iaccu,1),accuratepath(iaccu,2)+deltay];
%             iaccu=iaccu+1;
%         end
%         
%     else %������
%         
%         deltax=(pathfromstartpoint(i+1,1)-pathfromstartpoint(i,1))/(accu+1);
%         while accuratepath(iaccu,1)~=pathfromstartpoint(i+1,1)
%             accuratepath=[accuratepath;accuratepath(iaccu,1)+deltax,accuratepath(iaccu,2)];
%             iaccu=iaccu+1;
%         end
%         
%     end
%     
% end
% 
% 
% end


function [ReferencePath,deltaP] = interplotation(path)
%����˳��Ϊ����㵽�յ㲢���в�ֵ

%����˳��Ϊ��㵽�յ�
sizeofpath=size(path);
n=sizeofpath(1);
pathfromstartpoint=[];
for i=n:-1:1
    pathfromstartpoint=[pathfromstartpoint;path(i,:)];
end

%��ֵ
insertnum=1000; %����������դ�����ļ����insertnum����,������ֵ����
deltaP=(sqrt((path(1,1)-path(2,1))^2+(path(1,2)-path(2,2))^2))/(insertnum+1);%��ֵ�����ڵ�ľ���
ReferencePath=[pathfromstartpoint(1,1), pathfromstartpoint(1,2)]; %��ֵ���·��
irefer=1; %��ʾReferencepath��ǰ�к�

for i=1:n-1;
    
    if pathfromstartpoint(i,1)==pathfromstartpoint(i+1,1) %������
   
         for j=2:insertnum+2
             ReferencePath=[ReferencePath;pathfromstartpoint(i,1),interp1([1,insertnum+2],[pathfromstartpoint(i,2),pathfromstartpoint(i+1,2)],j,'linear')];
             irefer=irefer+1;
         end
          
        
    elseif pathfromstartpoint(i,2)==pathfromstartpoint(i+1,2) %������
        
        for j=2:insertnum+2
             ReferencePath=[ReferencePath;interp1([1,insertnum+2],[pathfromstartpoint(i,1),pathfromstartpoint(i+1,1)],j,'linear'),pathfromstartpoint(i,2)];
             irefer=irefer+1;
        end
         
    else %б����
        
        insertnumslop=(insertnum+1)*sqrt(2)-1;
        for j=2:insertnumslop+2
             ReferencePath=[ReferencePath;interp1([1,insertnumslop+2],[pathfromstartpoint(i,1),pathfromstartpoint(i+1,1)],j,'linear'),interp1([1,insertnumslop+2],[pathfromstartpoint(i,2),pathfromstartpoint(i+1,2)],j,'linear')];
             irefer=irefer+1;
        end
        
    end
    
end


end


