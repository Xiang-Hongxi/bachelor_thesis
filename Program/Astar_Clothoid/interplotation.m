% function accuratepath = interplotation(path)
% %对path进行插值，并交换顺序为从起点到终点
% 
% %交换顺序为起点到终点
% sizeofpath=size(path);
% n=sizeofpath(1);
% pathfromstartpoint=[];
% for i=n:-1:1
%     pathfromstartpoint=[pathfromstartpoint;path(i,:)];
% end
% 
% 
% %插值
% accu=7; %在相邻两个栅格中心间插入accu个点（插值精度）
% accuratepath=pathfromstartpoint(1,:); %插值后的路径,先放入起点
% iaccu=1; %表示accuratepath当前行号
% 
% for i=1:n-1;
%     
%     if pathfromstartpoint(i,1)==pathfromstartpoint(i+1,1) %竖着走
%         
%         deltay=(pathfromstartpoint(i+1,2)-pathfromstartpoint(i,2))/(accu+1);
%         while accuratepath(iaccu,2)~=pathfromstartpoint(i+1,2)
%             accuratepath=[accuratepath;accuratepath(iaccu,1),accuratepath(iaccu,2)+deltay];
%             iaccu=iaccu+1;
%         end
%         
%     else %横着走
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
%交换顺序为从起点到终点并进行插值

%交换顺序为起点到终点
sizeofpath=size(path);
n=sizeofpath(1);
pathfromstartpoint=[];
for i=n:-1:1
    pathfromstartpoint=[pathfromstartpoint;path(i,:)];
end

%插值
insertnum=1000; %在相邻两个栅格中心间插入insertnum个点,表征插值精度
deltaP=(sqrt((path(1,1)-path(2,1))^2+(path(1,2)-path(2,2))^2))/(insertnum+1);%插值后相邻点的距离
ReferencePath=[pathfromstartpoint(1,1), pathfromstartpoint(1,2)]; %插值后的路径
irefer=1; %表示Referencepath当前行号

for i=1:n-1;
    
    if pathfromstartpoint(i,1)==pathfromstartpoint(i+1,1) %竖着走
   
         for j=2:insertnum+2
             ReferencePath=[ReferencePath;pathfromstartpoint(i,1),interp1([1,insertnum+2],[pathfromstartpoint(i,2),pathfromstartpoint(i+1,2)],j,'linear')];
             irefer=irefer+1;
         end
          
        
    elseif pathfromstartpoint(i,2)==pathfromstartpoint(i+1,2) %横着走
        
        for j=2:insertnum+2
             ReferencePath=[ReferencePath;interp1([1,insertnum+2],[pathfromstartpoint(i,1),pathfromstartpoint(i+1,1)],j,'linear'),pathfromstartpoint(i,2)];
             irefer=irefer+1;
        end
         
    else %斜着走
        
        insertnumslop=(insertnum+1)*sqrt(2)-1;
        for j=2:insertnumslop+2
             ReferencePath=[ReferencePath;interp1([1,insertnumslop+2],[pathfromstartpoint(i,1),pathfromstartpoint(i+1,1)],j,'linear'),interp1([1,insertnumslop+2],[pathfromstartpoint(i,2),pathfromstartpoint(i+1,2)],j,'linear')];
             irefer=irefer+1;
        end
        
    end
    
end


end


