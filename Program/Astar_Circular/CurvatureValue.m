function Vcurvature = CurvatureValue(Cnv)
%��������й켣��CurvatureValue
% ��Vcurvature��׼����[0,1]�ķ������Դ���һ�£����Խ��Vpath�ķ�������size(Cnv,2)���������������п���ֵ�仮��Ϊ[0,1]

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

