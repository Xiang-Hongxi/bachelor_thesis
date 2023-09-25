function Vcurvaturedot = CurvatureDotValue(Cnv,CurrentC)
%计算各可行轨迹的CurvatureValue
% 把Vcurvature标准化到[0,1]的方法可以创新一下，可以借鉴Vpath的方法，在size(Cnv,2)条触须间而不是所有可能值间划分为[0,1]

deltaC=abs(Cnv-CurrentC);
deltaCmax=max(deltaC);
deltaCmin=min(deltaC);
Vcurvaturedot=(deltaC-deltaCmin)/(deltaCmax-deltaCmin);


end
