function Vcurvature = CurvatureValue(anv,C0,Vx,deltaT,m)
%��������й켣��CurvatureValue
% ��Vcurvature��׼����[0,1]�ķ������Դ���һ�£����Խ��Vpath�ķ�������size(Cnv,2)���������������п���ֵ�仮��Ϊ[0,1]

% roumax=0.11111;
% LT=6;

% Vcurvature=[];
% for i=1:size(Cnv,2)
%     Vcurvature=[Vcurvature,(abs(Cnv(i)))/(2*roumax/LT)];
% end

AbsOfanv=abs(anv*(Vx*deltaT*10)^(m)+C0);%��ѡ
% AbsOfanv=abs((anv*(Vx*deltaT*10)^(n+1))/(n+1)+C0*(Vx*deltaT*10));%��ѡ
% AbsOfanv=abs(anv);
anv_max=max(AbsOfanv);
anv_min=min(AbsOfanv);
Vcurvature=(AbsOfanv-anv_min)/(anv_max-anv_min);

end


