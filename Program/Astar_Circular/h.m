function hcost = h( m,goal )

%����������������ֵ ����������������㷨
hcost =10* abs(  m(1)-goal(1)  )+10*abs(  m(2)-goal(2)  );
% hcost =10* sqrt( (m(1)-goal(1))^2+(m(2)-goal(2))^2);
% hcost=0;

end