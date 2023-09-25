function  [xg,yg,deltaL,a]=CCTentacles(deltamax,Vx,L,amax,x0,y0,rou0,C0,fai,m)
% ��������״̬��λ�á�����ǡ�ת��ǡ����٣����������ȡ���������ٶȣ��õ�һϵ�д���
% x0,y0Ϊ��ǰ����������ȫ������ϵ�µ����꣬ faiΪ������ǰ��ڽǣ�������������ȫ������ϵx��ļнǣ�
% delta0��fai�Ի���Ϊ��λ
% x,yΪ��������ϵ�´������꣬xg,ygΪȫ������ϵ�¸����������

%���������������������������������������ɴ��롪����������������������������������

%��ʼ��������
%��ʼ����
% rou0=tan(delta0)/L;
% �������
% roumax=min(amax/(Vx^2),tan(deltamax)/L);
roumax=min(amax/(Vx^2),(tan(deltamax)/L)/sqrt(1+0.25*(tan(deltamax))^2));
roumaxsafe=roumax/1;
%pi/6Ϊǰ�����ת�ǣ�3Ϊǰ����֮�䳵�� 
%���������ٶ�
% acmax=1.5;
%��ײ����
% LT=(Vx^2)/acmax;
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

% Ltentacle=80;

%�Ѵ���ֳ�N�Σ���������N+1���㣩
N=1000;
%���ڵ��Ļ���
deltaL=Ltentacle/N;
%������Ŀ
Ntentacle=41;
% Ntentacle=81;
%��1�������a;
a1=2*(-roumaxsafe-rou0-C0*LT)/(LT^2);
%��Ntentacle�������dC/dl
aCNtentacle=2*(roumaxsafe-rou0-C0*LT)/(LT^2);
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
%         xn=xn+deltaL*cos((dCdli*Ln^3)/6+(C0*Ln^2)/2+rou0*Ln);
%         yn=yn+deltaL*sin((dCdli*Ln^3)/6+(C0*Ln^2)/2+rou0*Ln);
        xn=xn+deltaL*cos((ai*Ln^(m+2))/((m+1)*(m+2))+(C0*Ln^2)/2+rou0*Ln);
        yn=yn+deltaL*sin((ai*Ln^(m+2))/((m+1)*(m+2))+(C0*Ln^2)/2+rou0*Ln);
        x(i+1,k)=xn;
        y(i+1,k)=yn;
    end
%     plot(x(:,k),y(:,k),'color',[0,0.45,0.74])  
%     hold on
end
axis equal
% for k=1:size(x,2)
%     xtentaclek=x(:,k);
%     ytentaclek=y(:,k);
%     plot(xtentaclek,ytentaclek,'color',[0,0.45,0.74]);
%     hold on
% end
% grid on
% axis equal

%����������������������������������������ת������������������������������������

xg=cos(fai)*x-sin(fai)*y+x0;
yg=sin(fai)*x+cos(fai)*y+y0;

% for k=1:size(xg,2)
%     xtentaclek=xg(:,k);
%     ytentaclek=yg(:,k);
%     plot(xtentaclek,ytentaclek,'y');
%     hold on
% end

% grid on
% axis equal

end

