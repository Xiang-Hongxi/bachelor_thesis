function  [xg,yg,deltaL,a]=CCTentacles(deltamax,Vx,L,amax,x0,y0,rou0,C0,fai,m)
%���������������������������������������ɴ��롪����������������������������������

%��ʼ��������
% �������
roumax=min(amax/(Vx^2),(tan(deltamax)/L)/sqrt(1+0.25*(tan(deltamax))^2));
LT=1*Vx;%1ΪdeltaT�ʹ����̴򵽵�����ʱ�䣨1s���Ľϴ�ֵ

%���㴥���ܳ���Ltentacle
%����ϵ��
t0=7; L0=5; 
%���㴥�볤��
if Vx>1
    Ltentacle=t0*Vx-L0;
else
    Ltentacle=2;
end


%�Ѵ���ֳ�N�Σ���������N+1���㣩
N=1000;
%���ڵ��Ļ���
deltaL=Ltentacle/N;
%������Ŀ
Ntentacle=41;
%��1�������a;
a1=2*(-roumax-rou0-C0*LT)/(LT^2);
%��Ntentacle�������dC/dl
aCNtentacle=2*(roumax-rou0-C0*LT)/(LT^2);
%���Բ�ֵʱa������
deltaa=(aCNtentacle-a1)/(Ntentacle-1);

x=[];y=[];%x,yÿһ�зֱ���һ�������ϸ���ĺ�������


% �ۼӷ����
a=[];%�洢����������ʵ����仯��

for k=1:Ntentacle
    ai=a1+deltaa*(k-1);%�����Բ�ֵ�õ�һϵ��C
    
    if(k<=(1+Ntentacle)/2)
        ai=ai*0.9^(k-1);
    elseif(k>(1+Ntentacle)/2)
        ai=ai*0.9^(Ntentacle-k);
    end
    %�����Բ�ֵ�Ļ����Ͻ���a��һ��ϵ����ʹ�ô����м��ܣ�������
    a=[a,ai];
    
    xn=0;yn=0;%�������
    x(1,k)=xn; y(1,k)=yn;    
    
    for i=1:N
        Ln=i*deltaL;
        Ln_h=Ln^(m+2);
        Ln_l=Ln^2;
%         xn=xn+deltaL*cos((ai*Ln^(m+2))/((m+1)*(m+2))+(C0*Ln^2)/2+rou0*Ln);
%         yn=yn+deltaL*sin((ai*Ln^(m+2))/((m+1)*(m+2))+(C0*Ln^2)/2+rou0*Ln);
        xn=xn+deltaL*cos((ai*Ln_h)/((m+1)*(m+2))+(C0*Ln_l)/2+rou0*Ln);
        yn=yn+deltaL*sin((ai*Ln_h)/((m+1)*(m+2))+(C0*Ln_l)/2+rou0*Ln);
        x(i+1,k)=xn;
        y(i+1,k)=yn;
    end

end

%����������������������������������������ת������������������������������������
xg=cos(fai)*x-sin(fai)*y+x0;
yg=sin(fai)*x+cos(fai)*y+y0;

end

