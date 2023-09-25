function Vcurvature = CurvatureValue(Cnv)
%计算各可行轨迹的CurvatureValue
% 把Vcurvature标准化到[0,1]的方法可以创新一下，可以借鉴Vpath的方法，在size(Cnv,2)条触须间而不是所有可能值间划分为[0,1]

% roumax=0.244768648248987;
% LT=3;
% 
% Vcurvature=[];
% for i=1:size(Cnv,2)
%     Vcurvature=[Vcurvature,(abs(Cnv(i)))/(2*roumax/LT)];
% end

AbsOfCnv=abs(Cnv);
Cnv_max=max(AbsOfCnv);
Cnv_min=min(AbsOfCnv);
Vcurvature=(AbsOfCnv-Cnv_min)/(Cnv_max-Cnv_min);

end

