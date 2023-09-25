function Vcurvature = CurvatureValue(anv,C0,Vx,deltaT,m)
%计算各可行轨迹的CurvatureValue
% 把Vcurvature标准化到[0,1]的方法可以创新一下，可以借鉴Vpath的方法，在size(Cnv,2)条触须间而不是所有可能值间划分为[0,1]

% roumax=0.11111;
% LT=6;

% Vcurvature=[];
% for i=1:size(Cnv,2)
%     Vcurvature=[Vcurvature,(abs(Cnv(i)))/(2*roumax/LT)];
% end

AbsOfanv=abs(anv*(Vx*deltaT*10)^(m)+C0);%待选
% AbsOfanv=abs((anv*(Vx*deltaT*10)^(n+1))/(n+1)+C0*(Vx*deltaT*10));%待选
% AbsOfanv=abs(anv);
anv_max=max(AbsOfanv);
anv_min=min(AbsOfanv);
Vcurvature=(AbsOfanv-anv_min)/(anv_max-anv_min);

end


