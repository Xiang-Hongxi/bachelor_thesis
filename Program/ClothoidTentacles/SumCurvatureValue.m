function Vsumcurvature = SumCurvatureValue(rou0,Cnv,Vx,deltaT,deltaL)
%��ÿ������ǰVx��deltaT�θ������ʾ���ֵ�ĺ�

 %����ÿһ������ǰ��ʻ�����
 S=Vx*deltaT;%�滮�������ع켣��ʻ·��
 SIndexTentacle=floor(S/deltaL);
 if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
    SIndexTentacle=SIndexTentacle+1;
 end

%�����������ǰS�θ������ʾ���ֵ�ĺ�

SumOfAbsOfCurvature=[];%��Ÿ��������ʾ���ֵ�ĺ�
for k=1:size(Cnv,2)
    AbsOfCurvatureAtPoint=[];%��ŵ�k������������ʾ���ֵ
    for i=1:SIndexTentacle
        AbsOfCurvatureAtPoint=[AbsOfCurvatureAtPoint,abs(rou0+i*deltaL*Cnv(k))];
    end
    SumOfAbsOfCurvature=[SumOfAbsOfCurvature,sum(AbsOfCurvatureAtPoint)];
end

maxSum=max(SumOfAbsOfCurvature);
minSum=min(SumOfAbsOfCurvature);
Vsumcurvature=(SumOfAbsOfCurvature-minSum)/(maxSum-minSum);

end