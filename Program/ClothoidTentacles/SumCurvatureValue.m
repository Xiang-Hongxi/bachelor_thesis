function Vsumcurvature = SumCurvatureValue(rou0,Cnv,Vx,deltaT,deltaL)
%求每条触须前Vx·deltaT段各点曲率绝对值的和

 %计算每一周期向前行驶序号数
 S=Vx*deltaT;%规划周期内沿轨迹行驶路程
 SIndexTentacle=floor(S/deltaL);
 if abs(SIndexTentacle*deltaL-S)>abs((SIndexTentacle+1)*deltaL-S)
    SIndexTentacle=SIndexTentacle+1;
 end

%计算各条触须前S段各点曲率绝对值的和

SumOfAbsOfCurvature=[];%存放各触须曲率绝对值的和
for k=1:size(Cnv,2)
    AbsOfCurvatureAtPoint=[];%存放第k条触须各点曲率绝对值
    for i=1:SIndexTentacle
        AbsOfCurvatureAtPoint=[AbsOfCurvatureAtPoint,abs(rou0+i*deltaL*Cnv(k))];
    end
    SumOfAbsOfCurvature=[SumOfAbsOfCurvature,sum(AbsOfCurvatureAtPoint)];
end

maxSum=max(SumOfAbsOfCurvature);
minSum=min(SumOfAbsOfCurvature);
Vsumcurvature=(SumOfAbsOfCurvature-minSum)/(maxSum-minSum);

end