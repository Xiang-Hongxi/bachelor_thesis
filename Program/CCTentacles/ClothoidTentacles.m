function  [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0)
% function  ClothoidTentacles(deltamax,L,Vx,rou0,amax)
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
%��1�������d(rou)/dl
C1=(-roumax-rou0)/LT;
%��Ntentacle�������d(rou)/dl
CNtentacle=(roumax-rou0)/LT;
%���Բ�ֵʱd(rou)/dl������
deltaCi=(CNtentacle-C1)/(Ntentacle-1);

x=[];y=[];%x,yÿһ�зֱ���һ�������ϸ���ĺ�������

% ���ַ����
% C=[];
% for i=1:Ntentacle 
%     Ci=C1+deltaCi*(i-1);%�����Բ�ֵ�õ�һϵ��C
%     
%     if(i<=(1+Ntentacle)/2)
%         Ci=Ci*0.9^(i-1);
%     elseif(i>(1+Ntentacle)/2)
%         Ci=Ci*0.9^(Ntentacle-i);
%     end
%     %�����Բ�ֵ�Ļ����Ͻ���C��һ��ϵ����ʹ�ô����м��ܣ�������
%     C=[C,Ci];

%     xn=0;yn=0;%�������
%     x(1,i)=xn; y(1,i)=yn;
%     for n=1:N
%         Ln=n*deltaL;
%         xn=quad(@(tao)cos((Ci*tao.^2)/2+rou0*tao),0,Ln)+0;
%         yn=quad(@(tao)sin((Ci*tao.^2)/2+rou0*tao),0,Ln)+0;
%         x(n+1,i)=xn;
%         y(n+1,i)=yn;
%     end
%     plot(x(:,i),y(:,i))  
%     hold on
% end 


% �ۼӷ����
C=[];%�洢�������б�ʱ仯
for i=1:Ntentacle
    Ci=C1+deltaCi*(i-1);%�����Բ�ֵ�õ�һϵ��C
    
    if(i<=(1+Ntentacle)/2)
        Ci=Ci*0.9^(i-1);
    elseif(i>(1+Ntentacle)/2)
        Ci=Ci*0.9^(Ntentacle-i);
    end
    %�����Բ�ֵ�Ļ����Ͻ���C��һ��ϵ����ʹ�ô����м��ܣ�������
    C=[C,Ci];
    
    xn=0;yn=0;%�������
    x(1,i)=xn; y(1,i)=yn;
    for n=1:N
        Ln=n*deltaL;
        xn=xn+deltaL*cos((Ci*Ln^2)/2+rou0*Ln);
        yn=yn+deltaL*sin((Ci*Ln^2)/2+rou0*Ln);
        x(n+1,i)=xn;
        y(n+1,i)=yn;
    end
%     plot(x(:,i),y(:,i))  
%     hold on
end

% for i=1:size(x,2)
%     xtentaclei=x(:,i);
%     ytentaclei=y(:,i);
%     plot(xtentaclei,ytentaclei,'color',[0,0.45,0.74]);
%     hold on
% end
% grid on
% axis equal

%����������������������������������������ת������������������������������������

xg=cos(fai)*x-sin(fai)*y+x0;
yg=sin(fai)*x+cos(fai)*y+y0;
% 
% % for i=1:size(xg,2)
% %     xtentaclei=xg(:,i);
% %     ytentaclei=yg(:,i);
% %     plot(xtentaclei,ytentaclei,'y');
% %     hold on
% % end

% grid on
% axis equal

end
