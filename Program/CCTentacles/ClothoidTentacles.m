function  [xg,yg,deltaL,Ltentacle,C,roumax]=ClothoidTentacles(deltamax,Vx,L,amax,x0,y0,fai,rou0)
% function  ClothoidTentacles(deltamax,L,Vx,rou0,amax)
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
%第1条触须的d(rou)/dl
C1=(-roumax-rou0)/LT;
%第Ntentacle条触须的d(rou)/dl
CNtentacle=(roumax-rou0)/LT;
%线性插值时d(rou)/dl的增量
deltaCi=(CNtentacle-C1)/(Ntentacle-1);

x=[];y=[];%x,y每一列分别存放一条触须上各点的横纵坐标

% 积分法求解
% C=[];
% for i=1:Ntentacle 
%     Ci=C1+deltaCi*(i-1);%先线性插值得到一系列C
%     
%     if(i<=(1+Ntentacle)/2)
%         Ci=Ci*0.9^(i-1);
%     elseif(i>(1+Ntentacle)/2)
%         Ci=Ci*0.9^(Ntentacle-i);
%     end
%     %再线性插值的基础上将各C乘一定系数，使得触须中间密，两边疏
%     C=[C,Ci];

%     xn=0;yn=0;%触须起点
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


% 累加法求解
C=[];%存储各触须的斜率变化
for i=1:Ntentacle
    Ci=C1+deltaCi*(i-1);%先线性插值得到一系列C
    
    if(i<=(1+Ntentacle)/2)
        Ci=Ci*0.9^(i-1);
    elseif(i>(1+Ntentacle)/2)
        Ci=Ci*0.9^(Ntentacle-i);
    end
    %再线性插值的基础上将各C乘一定系数，使得触须中间密，两边疏
    C=[C,Ci];
    
    xn=0;yn=0;%触须起点
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

%——————————————————坐标转换—————————————————

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
