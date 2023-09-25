function Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle)
%计算各可行触须vdist

dist=[];%存放各可行触须对应点的距离

for k=1:size(xgnv,2)
    %求出两者距离
    dist=[dist,sqrt((xgnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),1))^2+(ygnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),2))^2)];
end

mindist=min(dist);
maxdist=max(dist);

Vdist=(dist-mindist)/(maxdist-mindist);