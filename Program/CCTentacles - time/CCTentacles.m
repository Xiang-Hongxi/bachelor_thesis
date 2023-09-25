function  [xg,yg,deltaL,a]=CCTentacles(deltamax,Vx,L,amax,x0,y0,rou0,C0,fai,m)
%——————————————————生成触须——————————————————

%初始化各参数
% 最大曲率
roumax=min(amax/(Vx^2),(tan(deltamax)/L)/sqrt(1+0.25*(tan(deltamax))^2));
LT=1*Vx;%1为deltaT和打方向盘打到底所需时间（1s）的较大值

%计算触须总长度Ltentacle
%所需系数
t0=7; L0=5; 
%计算触须长度
if Vx>1
    Ltentacle=t0*Vx-L0;
else
    Ltentacle=2;
end


%把触须分成N段（触须上有N+1个点）
N=1000;
%相邻点间的弧长
deltaL=Ltentacle/N;
%触须数目
Ntentacle=41;
%第1条触须的a;
a1=2*(-roumax-rou0-C0*LT)/(LT^2);
%第Ntentacle条触须的dC/dl
aCNtentacle=2*(roumax-rou0-C0*LT)/(LT^2);
%线性插值时a的增量
deltaa=(aCNtentacle-a1)/(Ntentacle-1);

x=[];y=[];%x,y每一列分别存放一条触须上各点的横纵坐标


% 累加法求解
a=[];%存储各触须的曲率导数变化率

for k=1:Ntentacle
    ai=a1+deltaa*(k-1);%先线性插值得到一系列C
    
    if(k<=(1+Ntentacle)/2)
        ai=ai*0.9^(k-1);
    elseif(k>(1+Ntentacle)/2)
        ai=ai*0.9^(Ntentacle-k);
    end
    %再线性插值的基础上将各a乘一定系数，使得触须中间密，两边疏
    a=[a,ai];
    
    xn=0;yn=0;%触须起点
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

%——————————————————坐标转换—————————————————
xg=cos(fai)*x-sin(fai)*y+x0;
yg=sin(fai)*x+cos(fai)*y+y0;

end

