function [w,maxeigvalue,CI,RI,CR] = MatrixAnalyse(M)
%���������M�������ֵ���������ֵ��Ӧ������������������һ���Է���

[V,D]=eig(M);%V������������D������ֵ
[maxeigvalue,col]=max(max(D));%maxeigvalue���������ֵ��col���������ֵ������
w=1/sum(V(:,col))*V(:,col);%wΪ��ӦȨ��

%һ���Լ���
N=size(M,1);
CI=(maxeigvalue-N)/(N-1);
if N==3
    RI=0.58;
elseif N==4
    RI=0.90;
elseif N==5
    RI=1.12;
elseif N==6
    RI=1.24;
end
CR=CI/RI;





end

