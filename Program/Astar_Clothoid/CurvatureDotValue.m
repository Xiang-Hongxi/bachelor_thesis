function Vcurvaturedot = CurvatureDotValue(Cnv,CurrentC)
%��������й켣��CurvatureValue
% ��Vcurvature��׼����[0,1]�ķ������Դ���һ�£����Խ��Vpath�ķ�������size(Cnv,2)���������������п���ֵ�仮��Ϊ[0,1]

deltaC=abs(Cnv-CurrentC);
deltaCmax=max(deltaC);
deltaCmin=min(deltaC);
Vcurvaturedot=(deltaC-deltaCmin)/(deltaCmax-deltaCmin);


end
