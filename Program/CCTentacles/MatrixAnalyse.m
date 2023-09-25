function [w,maxeigvalue,CI,RI,CR] = MatrixAnalyse(M)
%求输入矩阵M最大特征值，最大特征值对应的特征向量，并进行一致性分析

[V,D]=eig(M);%V是特征向量，D是特征值
[maxeigvalue,col]=max(max(D));%maxeigvalue是最大特征值，col是最大特征值所在列
w=1/sum(V(:,col))*V(:,col);%w为对应权重

%一致性检验
N=size(M,1);
CI=(maxeigvalue-N)/(N-1);
if N==3
    RI=0.58;
elseif N==4
    RI=0.9;
end
CR=CI/RI;





end

