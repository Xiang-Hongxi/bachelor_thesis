function boundary=GetBoundary(map)
%获得地图的边界的坐标
% boundary=[];
% for i1=0:(map.XYMAX+1)
%     boundary=[boundary;[0 i1]];
% end
% for i2=0:(map.XYMAX+1)
%     boundary=[boundary;[i2 0]];
% end
% for i3=0:(map.XYMAX+1)
%     boundary=[boundary;[map.XYMAX+1 i3]];
% end
% for i4=0:(map.XYMAX+1)
%     boundary=[boundary;[i4 map.XYMAX+1]];
% end

% boundary=[];
% for i1=0:(map.YMAX+1)
%     boundary=[boundary;[0 i1]];
% end
% for i2=0:(map.XMAX+1)
%     boundary=[boundary;[i2 0]];
% end
% for i3=0:(map.YMAX+1)
%     boundary=[boundary;[map.XMAX+1 i3]];
% end
% for i4=0:(map.XMAX+1)
%     boundary=[boundary;[i4 map.YMAX+1]];
% end

boundary=[];
for i1=0:10:(map.YMAX+10)
    boundary=[boundary;[0 i1]];
end
for i2=0:10:(map.XMAX+10)
    boundary=[boundary;[i2 0]];
end
for i3=0:10:(map.YMAX+10)
    boundary=[boundary;[map.XMAX+10 i3]];
end
for i4=0:10:(map.XMAX+10)
    boundary=[boundary;[i4 map.YMAX+10]];
end

end