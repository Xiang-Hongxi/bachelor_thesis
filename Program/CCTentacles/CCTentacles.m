function  [xg,yg,deltaL,a]=CCTentacles(deltamax,Vx,L,amax,x0,y0,rou0,C0,fai,m)
% 给定车辆状态（位置、航向角、转向角、车速）、车辆长度、最大侧向加速度，得到一系列触须
% x0,y0为当前车辆质心在全局坐标系下的坐标， fai为车辆当前横摆角（车辆纵轴线与全局坐标系x轴的夹角）
% delta0、fai以弧度为单位
% x,y为车辆坐标系下触须坐标，xg,yg为全局坐标系下各触须的坐标

%——————————————————生成触须——————————————————

%初始化各参数
%初始曲率
% rou0=tan(delta0)/L;
% 最大曲率
% roumax=min(amax/(Vx^2),tan(deltamax)/L);
roumax=min(amax/(Vx^2),(tan(deltamax)/L)/sqrt(1+0.25*(tan(deltamax))^2));
roumaxsafe=roumax/1;
%pi/6为前轮最大转角，3为前后轴之间车长 
%最大纵向减速度
% acmax=1.5;
%碰撞距离
% LT=(Vx^2)/acmax;
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

% Ltentacle=80;

%把触须分成N段（触须上有N+1个点）
N=1000;
%相邻点间的弧长
deltaL=Ltentacle/N;
%触须数目
Ntentacle=41;
% Ntentacle=81;
%第1条触须的a;
a1=2*(-roumaxsafe-rou0-C0*LT)/(LT^2);
%第Ntentacle条触须的dC/dl
aCNtentacle=2*(roumaxsafe-rou0-C0*LT)/(LT^2);
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

%——————————————————坐标转换—————————————————

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

