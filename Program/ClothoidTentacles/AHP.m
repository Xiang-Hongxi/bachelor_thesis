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

