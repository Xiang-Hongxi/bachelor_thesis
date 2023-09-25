%测试用
A1=[1 1/7 1/5 1/3;7 1 3 5;5 1/3 1 3;3 1/5 1/3 1];
A2=[1 3 5 4;1/3 1 3 2;1/5 1/3 1 1/2;1/4 1/2 2 1];

%变量顺序为：Vdist，Valpha，Vc

%最大侧向偏差maxError矩阵B1
dist_alpha=3;
alpha_C=3;
dist_C=7;
B1=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B1,maxeigvalue_B1,CI_B1,RI_B1,CR_B1] = MatrixAnalyse(B1);

%平均侧向偏差meanError矩阵B2
dist_alpha=5;
alpha_C=2;
dist_C=9;
B2=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B2,maxeigvalue_B2,CI_B2,RI_B2,CR_B2] = MatrixAnalyse(B2);

%最大曲率maxCurvature矩阵B3
dist_alpha=1/3;
alpha_C=1/2;
dist_C=1/5;
B3=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B3,maxeigvalue_B3,CI_B3,RI_B3,CR_B3] = MatrixAnalyse(B3);

%平均曲率绝对值meanAbsCurvature矩阵B4
dist_alpha=1/3;
alpha_C=5;
dist_C=2;
B4=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B4,maxeigvalue_B4,CI_B4,RI_B4,CR_B4] = MatrixAnalyse(B4);

%最大曲率导数maxC矩阵B5
dist_alpha=1/2;
alpha_C=1/3;
dist_C=1/5;
B5=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B5,maxeigvalue_B5,CI_B5,RI_B5,CR_B5] = MatrixAnalyse(B5);

%平均曲率导数meanAbsC矩阵B6
dist_alpha=1/3;
alpha_C=1;
dist_C=1/3;
B6=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B6,maxeigvalue_B6,CI_B6,RI_B6,CR_B6] = MatrixAnalyse(B6);

%总排序与总一致检验

%准确性/平滑性
accu_smooth=7/3;
w_accu=accu_smooth/(1+accu_smooth);
w_smooth=1/(1+accu_smooth);

w_maxError=w_accu*0.6;
w_meanError=w_accu*0.4;
w_maxCurvature=w_smooth*0.3;
w_AverageAbsOfCurvature=w_smooth*0.2;
w_maxC=w_smooth*0.3;
w_AverageAbsofC=w_smooth*0.2;

CR=(w_maxError*CI_B1+w_meanError*CI_B2+w_maxCurvature*CI_B3+w_AverageAbsOfCurvature*CI_B4+w_maxC*CI_B5+w_AverageAbsofC*CI_B6)/...
    (w_maxError*RI_B1+w_meanError*RI_B2+w_maxCurvature*RI_B3+w_AverageAbsOfCurvature*RI_B4+w_maxC*RI_B5+w_AverageAbsofC*RI_B6);

w_dist=w_maxError*w_B1(1)+w_meanError*w_B2(1)+w_maxCurvature*w_B3(1)+w_AverageAbsOfCurvature*w_B4(1)+w_maxC*w_B5(1)+w_AverageAbsofC*w_B6(1);
w_alpha=w_maxError*w_B1(2)+w_meanError*w_B2(2)+w_maxCurvature*w_B3(2)+w_AverageAbsOfCurvature*w_B4(2)+w_maxC*w_B5(2)+w_AverageAbsofC*w_B6(2);
w_C=w_maxError*w_B1(3)+w_meanError*w_B2(3)+w_maxCurvature*w_B3(3)+w_AverageAbsOfCurvature*w_B4(3)+w_maxC*w_B5(3)+w_AverageAbsofC*w_B6(3);
w_total=[w_dist,w_alpha,w_C];