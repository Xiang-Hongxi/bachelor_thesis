function PlotGrid( map )
%PLOTGRID Summary of this function goes here
%   Detailed explanation goes here

%»æÖÆÍø¸ñ
% for i = 1:map.XYMAX+3
%  line([-0.5,map.XYMAX+1.5],[i-1.5,i-1.5]);
% end

% for i = 1:map.YMAX+3
%    line([-0.5,map.XMAX+1.5],[i-1.5,i-1.5]);
% end
for i = 10:10:map.YMAX+30
   line([-5,map.XMAX+15],[i-15,i-15],'color',[0.31,0.4,0.58],'HandleVisibility','off');
end

% for j = 1:map.XYMAX+3
%  line([j-1.5,j-1.5],[-0.5,map.XYMAX+1.5]);
% end

% for j = 1:map.XMAX+3
%    line([j-1.5,j-1.5],[-0.5,map.YMAX+1.5]);
% end
for j = 10:10:map.XMAX+30
   line([j-15,j-15],[-5,map.YMAX+15],'color',[0.31,0.4,0.58],'HandleVisibility','off');
end

hold on;
plot(map.start(1),map.start(2),'or','MarkerSize',10,'LineWidth',3,'HandleVisibility','off');
hold on;
plot(map.goal(1),map.goal(2),'or','MarkerSize',10,'LineWidth',3,'HandleVisibility','off');

% axis([-1.5,map.XYMAX+2.5,-1.5,map.XYMAX+2.5]);

% axis([-1.5,map.XMAX+2.5,-1.5,map.YMAX+2.5]);
axis([-15,map.XMAX+25,-15,map.YMAX+25]);
axis equal;


end