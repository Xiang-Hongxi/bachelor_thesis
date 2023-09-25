function Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle)
%��������д���valpha

alpha=[];%��Ÿ����д����Ӧ��ļн�

for k=1:size(xgnv,2)
    %���ReferencePath��Lcknv����x��������н�
    if ReferencePath(EndIndexofRP,1)==ReferencePath(EndIndexofRP-1,1)
        if ReferencePath(EndIndexofRP,2)>ReferencePath(EndIndexofRP-1,2)
            AngleOfRP=pi/2;
        else
            AngleOfRP=-pi/2;
        end
    elseif ReferencePath(EndIndexofRP,1)>ReferencePath(EndIndexofRP-1,1)
        SlopeOfRP=(ReferencePath(EndIndexofRP,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP,1)-ReferencePath(EndIndexofRP-1,1));
        AngleOfRP=atan(SlopeOfRP);
    else
        SlopeOfRP=(ReferencePath(EndIndexofRP,2)-ReferencePath(EndIndexofRP-1,2))/(ReferencePath(EndIndexofRP,1)-ReferencePath(EndIndexofRP-1,1));
        if SlopeOfRP>0
            AngleOfRP=atan(SlopeOfRP)-pi;
        else
            AngleOfRP=atan(SlopeOfRP)+pi;
        end
    end
    
    %���������Lcknv����x��������н�
    if xgnv(EndIndexofTentacle,k)==xgnv(EndIndexofTentacle-1,k)
        if ygnv(EndIndexofTentacle,k)>ygnv(EndIndexofTentacle-1,k)
            AngleOfTentacle=pi/2;
        else
            AngleOfTentacle=-pi/2;
        end
    elseif xgnv(EndIndexofTentacle,k)>xgnv(EndIndexofTentacle-1,k)
        SlopeOfTentacle=((ygnv(EndIndexofTentacle,k)-ygnv(EndIndexofTentacle-1,k)))/((xgnv(EndIndexofTentacle,k)-xgnv(EndIndexofTentacle-1,k)));
        AngleOfTentacle=atan(SlopeOfTentacle); 
    else
        SlopeOfTentacle=((ygnv(EndIndexofTentacle,k)-ygnv(EndIndexofTentacle-1,k)))/((xgnv(EndIndexofTentacle,k)-xgnv(EndIndexofTentacle-1,k)));
        if SlopeOfTentacle>0
            AngleOfTentacle=atan(SlopeOfTentacle)-pi;
        else
            AngleOfTentacle=atan(SlopeOfTentacle)+pi;
        end
    end
    
    %������߼н�
    alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)]; 
end


maxalpha=max(alpha);
minalpha=min(alpha);

Valpha=(alpha-minalpha)/(maxalpha-minalpha);

end




