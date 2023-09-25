clear,clc

%未考虑坐标转换
%绘制最外面的触须

n=16;%速度区间数
roucircle=1.35;%常数
V_=(2.8589+3.497)/2;%////////////////////////////////////
% V_=(5.5319+6.2437)/2;
amax=4;
deltamax=35*pi/180;
L=2.7;


j=7;%////////////////////////////////////
% j=11;
q=(j-1)/(n-1);
l=10+35*q^2;%最外侧触须弧长

% roumax=min(amax/(V_^2),(tan(deltamax)/L)/sqrt(1+0.25*(tan(deltamax))^2));
roumax=min(amax/(V_^2),(tan(deltamax)/L)/sqrt(1+0*(tan(deltamax))^2));

Rj=1/roumax;

deltaL=Rj*pi/2000; 
% pi/2000为第一条触须采样间隔夹角，为方便起见，设所有触须deltaL均一样

x=[];
y=[];
%存放各触须x,y坐标，每列表示一条触须
NumOfRowsForCircleMatrix=floor((l+20)/deltaL)+1;
%x,y矩阵行数等于最长触须（第21条触须）的坐标数据个数
EffectiveNum=[];%存放各触须有效坐标个数
rou=[];%保持第k条触须的曲率

for k=1:41
    if k<=20
        Rk=roucircle^(k-1)*Rj;
        rouk=-1/Rk;
        lk=l+20*sqrt((k-1)/20);
        
        Fai1=pi;
        deltaFaik=lk/Rk;%第k条触须弧长对应的圆心角
        Fai2=Fai1-deltaFaik;
        Fai=(Fai1:-deltaL/Rk:Fai2)';%-deltaLcircle/Rk对应的弧长即为deltaLcircle,转置使其为一列
        
        circlex0=Rk;
        circley0=0;%圆心坐标
        
        xk=circlex0+Rk*cos(Fai);
        yk=circley0+Rk*sin(Fai);%实际触须坐标
        
        EffectiveNum=[EffectiveNum,size(xk,1)];
        %circlex、circley中每列前EffectiveNum(含)个数字代表真实的触须坐标
        
        for i=EffectiveNum(k)+1:NumOfRowsForCircleMatrix
            xk(i)=inf;
            yk(i)=inf;
        end
        %补足数据，保证矩阵维度一致
        
        x=[x,xk];
        y=[y,yk];
        %存入第k条触须数据
        
    elseif k>21
        Rk=roucircle^(41-k)*Rj;
        rouk=1/Rk;
        lk=l+20*sqrt((41-k)/20);
        Fai1=0;
        deltaFaik=lk/Rk;%第k条触须弧长对应的圆心角
        Fai2=Fai1+deltaFaik;
        Fai=(Fai1:deltaL/Rk:Fai2)';%-deltaLcircle/Rk对应的弧长即为deltaLcircle，转置使其为一列
        
        circlex0=-Rk;
        circley0=0;%圆心坐标 
        
        xk=circlex0+Rk*cos(Fai);
        yk=circley0+Rk*sin(Fai);%实际触须坐标
        
        EffectiveNum=[EffectiveNum,size(xk,1)];
        %circlex、circley中每列前EffectiveNum(含)个数字代表真实的触须坐标
        
        for i=EffectiveNum(k)+1:NumOfRowsForCircleMatrix
            xk(i)=inf;
            yk(i)=inf;
        end
        %补足数据，保证矩阵维度一致
        
        x=[x,xk];
        y=[y,yk];
        %存入第k条触须数据
        
    else %k=21
        rouk=0;
        lk=l+20;%中间直线触须的长度
        xk=zeros(NumOfRowsForCircleMatrix,1);
        for i=1:NumOfRowsForCircleMatrix
            yk(i,1)=(i-1)*deltaL;
        end
        x=[x,xk];
        y=[y,yk];
        %存入第k条触须数据
        EffectiveNum=[EffectiveNum,size(xk,1)];
    end 
    rou=[rou,rouk];
end

xp=cos(-pi/2)*x-sin(-pi/2)*y;
yp=sin(-pi/2)*x+cos(-pi/2)*y;
%顺时针旋转90度，保证触须前进方向是车辆纵轴,下标p表示规划坐标系

for i=1:size(xp,2)
    xi=xp(1:EffectiveNum(i),i);
    yi=yp(1:EffectiveNum(i),i);
%     plot(circlexi,circleyi)
    plot(xi,yi,'Color',[0 0.45 0.74])  
    hold on
end
% 

save('circulartentaclesdata', 'xp', 'yp', 'EffectiveNum','deltaL','rou')

grid on
axis equal
