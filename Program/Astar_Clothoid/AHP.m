% %测试用
% A1=[1 1/7 1/5 1/3;7 1 3 5;5 1/3 1 3;3 1/5 1/3 1];
% A2=[1 3 5 4;1/3 1 3 2;1/5 1/3 1 1/2;1/4 1/2 2 1];
% 
% %变量顺序为：Vdist，Valpha，Vc
% 
% %最大侧向偏差maxError矩阵B1
% dist_alpha=3;
% alpha_C=3;
% dist_C=7;
% B1=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
% [w_B1,maxeigvalue_B1,CI_B1,RI_B1,CR_B1] = MatrixAnalyse(B1);
% 
% %平均侧向偏差meanError矩阵B2
% dist_alpha=5;
% alpha_C=2;
% dist_C=9;
% B2=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
% [w_B2,maxeigvalue_B2,CI_B2,RI_B2,CR_B2] = MatrixAnalyse(B2);
% 
% %最大曲率maxCurvature矩阵B3
% dist_alpha=1/3;
% alpha_C=1/2;
% dist_C=1/5;
% B3=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
% [w_B3,maxeigvalue_B3,CI_B3,RI_B3,CR_B3] = MatrixAnalyse(B3);
% 
% %平均曲率绝对值meanAbsCurvature矩阵B4
% dist_alpha=1/3;
% alpha_C=5;
% dist_C=2;
% B4=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
% [w_B4,maxeigvalue_B4,CI_B4,RI_B4,CR_B4] = MatrixAnalyse(B4);
% 
% %最大曲率导数maxC矩阵B5
% dist_alpha=1/2;
% alpha_C=1/3;
% dist_C=1/5;
% B5=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
% [w_B5,maxeigvalue_B5,CI_B5,RI_B5,CR_B5] = MatrixAnalyse(B5);
% 
% %平均曲率导数meanAbsC矩阵B6
% dist_alpha=1/3;
% alpha_C=1;
% dist_C=1/3;
% B6=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
% [w_B6,maxeigvalue_B6,CI_B6,RI_B6,CR_B6] = MatrixAnalyse(B6);
% 
% %总排序与总一致检验
% 
% %准确性/平滑性
% accu_smooth=7/3;
% w_accu=accu_smooth/(1+accu_smooth);
% w_smooth=1/(1+accu_smooth);
% 
% w_maxError=w_accu*0.6;
% w_meanError=w_accu*0.4;
% w_maxCurvature=w_smooth*0.3;
% w_AverageAbsOfCurvature=w_smooth*0.2;
% w_maxC=w_smooth*0.3;
% w_AverageAbsofC=w_smooth*0.2;
% 
% CR=(w_maxError*CI_B1+w_meanError*CI_B2+w_maxCurvature*CI_B3+w_AverageAbsOfCurvature*CI_B4+w_maxC*CI_B5+w_AverageAbsofC*CI_B6)/...
%     (w_maxError*RI_B1+w_meanError*RI_B2+w_maxCurvature*RI_B3+w_AverageAbsOfCurvature*RI_B4+w_maxC*RI_B5+w_AverageAbsofC*RI_B6);
% 
% w_dist=w_maxError*w_B1(1)+w_meanError*w_B2(1)+w_maxCurvature*w_B3(1)+w_AverageAbsOfCurvature*w_B4(1)+w_maxC*w_B5(1)+w_AverageAbsofC*w_B6(1);
% w_alpha=w_maxError*w_B1(2)+w_meanError*w_B2(2)+w_maxCurvature*w_B3(2)+w_AverageAbsOfCurvature*w_B4(2)+w_maxC*w_B5(2)+w_AverageAbsofC*w_B6(2);
% w_C=w_maxError*w_B1(3)+w_meanError*w_B2(3)+w_maxCurvature*w_B3(3)+w_AverageAbsOfCurvature*w_B4(3)+w_maxC*w_B5(3)+w_AverageAbsofC*w_B6(3);
% w_total=[w_dist,w_alpha,w_C];


% %准确性/平滑性
% accu_smooth=7/3;
% w_accu=accu_smooth/(1+accu_smooth);
% w_smooth=1/(1+accu_smooth);
% 
% w_maxError=w_accu*0.6;
% w_meanError=w_accu*0.4;
% w_maxCurvature=w_smooth*0.3;
% w_AverageAbsOfCurvature=w_smooth*0.15;
% w_maxC=w_smooth*0.4;
% w_AverageAbsofC=w_smooth*0.15;

% 计算各评价指标权重
% 顺序为maxError,meanError,maxCurvature,meanCurvature,maxC,meanC
maxE_meanE=2;
maxE_maxCurvature=3;
maxE_meanCurvature=9;
maxE_maxC=3;
maxE_meanC=9;
meanE_maxCurvature=1;
meanE_meanCurvature=3;
meanE_maxC=1;
meanE_meanC=5;
maxCurvature_meanCurvature=4;
maxCurvature_maxC=1;
maxCurvature_meanC=4;
meanCurvature_maxC=1/4;
meanCurvature_meanC=1;
maxC_meanC=4;

A=[1,maxE_meanE,maxE_maxCurvature,maxE_meanCurvature,maxE_maxC,maxE_meanC;...
   1/maxE_meanE,1,meanE_maxCurvature,meanE_meanCurvature,meanE_maxC,meanE_meanC;...
   1/maxE_maxCurvature,1/meanE_maxCurvature,1,maxCurvature_meanCurvature,maxCurvature_maxC,maxCurvature_meanC;...
   1/maxE_meanCurvature,1/meanE_meanCurvature,1/maxCurvature_meanCurvature,1,meanCurvature_maxC,meanCurvature_meanC;...
   1/maxE_maxC,1/meanE_maxC,1/maxCurvature_maxC,1/meanCurvature_maxC,1,maxC_meanC;...
   1/maxE_meanC,1/meanE_meanC,1/maxCurvature_meanC,1/meanCurvature_meanC,1/maxC_meanC,1];

[w_A,maxeigvalue_A,CI_A,RI_A,CR_A] = MatrixAnalyse(A);
w_maxError=w_A(1);
w_meanError=w_A(2);
w_maxCurvature=w_A(3);
w_meanCurvature=w_A(4);
w_maxC=w_A(5);
w_meanC=w_A(6);

%变量顺序为：Vdist，Valpha，Vc

%避障情形
%最大侧向偏差maxError矩阵B1
dist_alpha=9;
alpha_C=1/2;
dist_C=4;
B1_a=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B1_a,maxeigvalue_B1_a,CI_B1_a,RI_B1_a,CR_B1_a] = MatrixAnalyse(B1_a);

%平均侧向偏差meanError矩阵B2
dist_alpha=9;
alpha_C=1/3;
dist_C=4;
B2_a=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B2_a,maxeigvalue_B2_a,CI_B2_a,RI_B2_a,CR_B2_a] = MatrixAnalyse(B2_a);

%最大曲率maxCurvature矩阵B3
dist_alpha=1/4;
alpha_C=3;
dist_C=1/2;
B3_a=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B3_a,maxeigvalue_B3_a,CI_B3_a,RI_B3_a,CR_B3_a] = MatrixAnalyse(B3_a);

%平均曲率绝对值meanCurvature矩阵B4
dist_alpha=1/2;
alpha_C=4;
dist_C=3;
B4_a=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B4_a,maxeigvalue_B4_a,CI_B4_a,RI_B4_a,CR_B4_a] = MatrixAnalyse(B4_a);

%最大曲率导数maxC矩阵B5
dist_alpha=1/4;
alpha_C=1;
dist_C=1/4;
B5_a=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B5_a,maxeigvalue_B5_a,CI_B5_a,RI_B5_a,CR_B5_a] = MatrixAnalyse(B5_a);

%平均曲率导数meanC矩阵B6
dist_alpha=1/3;
alpha_C=3;
dist_C=1;
B6_a=[1,dist_alpha,dist_C;1/dist_alpha,1,alpha_C;1/dist_C,1/alpha_C,1];
[w_B6_a,maxeigvalue_B6_a,CI_B6_a,RI_B6_a,CR_B6_a] = MatrixAnalyse(B6_a);

%总排序与总一致检验

CR_a=(w_maxError*CI_B1_a+w_meanError*CI_B2_a+w_maxCurvature*CI_B3_a+w_meanCurvature*CI_B4_a+w_maxC*CI_B5_a+w_meanC*CI_B6_a)/...
    (w_maxError*RI_B1_a+w_meanError*RI_B2_a+w_maxCurvature*RI_B3_a+w_meanCurvature*RI_B4_a+w_maxC*RI_B5_a+w_meanC*RI_B6_a);
w_dist_a=w_maxError*w_B1_a(1)+w_meanError*w_B2_a(1)+w_maxCurvature*w_B3_a(1)+w_meanCurvature*w_B4_a(1)+w_maxC*w_B5_a(1)+w_meanC*w_B6_a(1);
w_alpha_a=w_maxError*w_B1_a(2)+w_meanError*w_B2_a(2)+w_maxCurvature*w_B3_a(2)+w_meanCurvature*w_B4_a(2)+w_maxC*w_B5_a(2)+w_meanC*w_B6_a(2);
w_C_a=w_maxError*w_B1_a(3)+w_meanError*w_B2_a(3)+w_maxCurvature*w_B3_a(3)+w_meanCurvature*w_B4_a(3)+w_maxC*w_B5_a(3)+w_meanC*w_B6_a(3);
w_total_a=[w_dist_a,w_alpha_a,w_C_a];


%转弯情形
%最大侧向偏差maxError矩阵B1
dist_alpha=5;
B1_t=[1,dist_alpha;1/dist_alpha,1];
w_B1_t=[dist_alpha/(1+dist_alpha),1/(1+dist_alpha),0];

%平均侧向偏差meanError矩阵B2
dist_alpha=5;
B2_t=[1,dist_alpha;1/dist_alpha,1];
w_B2_t=[dist_alpha/(1+dist_alpha),1/(1+dist_alpha),0];

%最大曲率maxCurvature矩阵B3
dist_alpha=1/3;
B3_t=[1,dist_alpha;1/dist_alpha,1];
w_B3_t=[dist_alpha/(1+dist_alpha),1/(1+dist_alpha),0];

%平均曲率绝对值meanCurvature矩阵B4
dist_alpha=1;
B4_t=[1,dist_alpha;1/dist_alpha,1];
w_B4_t=[dist_alpha/(1+dist_alpha),1/(1+dist_alpha),0];

%最大曲率导数maxC矩阵B5
dist_alpha=1/3;
B5_t=[1,dist_alpha;1/dist_alpha,1];
w_B5_t=[dist_alpha/(1+dist_alpha),1/(1+dist_alpha),0];

%平均曲率导数meanC矩阵B6
dist_alpha=1/2;
B6_t=[1,dist_alpha;1/dist_alpha,1];
w_B6_t=[dist_alpha/(1+dist_alpha),1/(1+dist_alpha),0];

%总排序
w_dist_t=w_maxError*w_B1_t(1)+w_meanError*w_B2_t(1)+w_maxCurvature*w_B3_t(1)+w_meanCurvature*w_B4_t(1)+w_maxC*w_B5_t(1)+w_meanC*w_B6_t(1);
w_alpha_t=w_maxError*w_B1_t(2)+w_meanError*w_B2_t(2)+w_maxCurvature*w_B3_t(2)+w_meanCurvature*w_B4_t(2)+w_maxC*w_B5_t(2)+w_meanC*w_B6_t(2);
w_C_t=w_maxError*w_B1_t(3)+w_meanError*w_B2_t(3)+w_maxCurvature*w_B3_t(3)+w_meanCurvature*w_B4_t(3)+w_maxC*w_B5_t(3)+w_meanC*w_B6_t(3);
w_total_t=[w_dist_t,w_alpha_t,w_C_t];

w_a=0.4;%避障情形权重
w_t=0.6;%转弯情形权重
w_total=[];
for i=1:size(w_total_a,2)
    w_total=[w_total,w_total_a(i)*w_a+w_total_t(i)*w_t];
end

