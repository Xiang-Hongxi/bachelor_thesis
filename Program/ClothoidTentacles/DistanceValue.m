function Vdist = DistanceValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle)
%��������д���vdist

dist=[];%��Ÿ����д����Ӧ��ľ���

for k=1:size(xgnv,2)
    %������߾���
    dist=[dist,sqrt((xgnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),1))^2+(ygnv(EndIndexofTentacle(k),k)-ReferencePath(EndIndexofRP(k),2))^2)];
end

mindist=min(dist);
maxdist=max(dist);

Vdist=(dist-mindist)/(maxdist-mindist);