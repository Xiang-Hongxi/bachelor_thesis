%分析tg
maxtg=max(tg);
meantg=mean(tg);

P1g=0;%频数
for i=1:5000
    if tg(i)<0.005
        P1g=P1g+1;
    end
end
p1g=P1g/5000*100;

P2g=0;%频数
for i=1:5000
    if tg(i)<0.006
        P2g=P2g+1;
    end
end
p2g=P2g/5000*100;

%分析ts
maxts=max(ts);
meants=mean(ts);

P1s=0;%频数
for i=1:5000
    if ts(i)<0.005
        P1s=P1s+1;
    end
end
p1s=P1s/5000*100;

P2s=0;%频数
for i=1:5000
    if ts(i)<0.006
        P2s=P2s+1;
    end
end
p2s=P2s/5000*100;

%分析t总
maxt=max(ts+tg);
meant=mean(ts+tg);

P1t=0;%频数
for i=1:5000
    if ts(i)+tg(i)<0.01
        P1t=P1t+1;
    end
end
p1t=P1t/5000*100;

P2t=0;%频数
for i=1:5000
    if ts(i)+tg(i)<0.015
        P2t=P2t+1;
    end
end
p2t=P2t/5000*100;

