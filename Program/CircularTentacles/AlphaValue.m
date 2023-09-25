function Valpha = AlphaValue(xgnv,ygnv,ReferencePath,EndIndexofRP,EndIndexofTentacle)
%计算各可行触须valpha

alpha=[];%存放各可行触须对应点的夹角

for k=1:size(xgnv,2)
    %求出ReferencePath上Lcknv处与x轴正方向夹角
    if ReferencePath(EndIndexofRP(k),1)==ReferencePath(EndIndexofRP(k)-1,1)
        if ReferencePath(EndIndexofRP(k),2)>ReferencePath(EndIndexofRP(k)-1,2)
            AngleOfRP=pi/2;
        else
            AngleOfRP=-pi/2;
        end
    elseif ReferencePath(EndIndexofRP(k),1)>ReferencePath(EndIndexofRP(k)-1,1)
        SlopeOfRP=(ReferencePath(EndIndexofRP(k),2)-ReferencePath(EndIndexofRP(k)-1,2))/(ReferencePath(EndIndexofRP(k),1)-ReferencePath(EndIndexofRP(k)-1,1));
        AngleOfRP=atan(SlopeOfRP);
    else
        SlopeOfRP=(ReferencePath(EndIndexofRP(k),2)-ReferencePath(EndIndexofRP(k)-1,2))/(ReferencePath(EndIndexofRP(k),1)-ReferencePath(EndIndexofRP(k)-1,1));
        if SlopeOfRP>0
            AngleOfRP=atan(SlopeOfRP)-pi;
        else
            AngleOfRP=atan(SlopeOfRP)+pi;
        end
    end
    
    %求出触须上Lcknv处与x轴正方向夹角
    if xgnv(EndIndexofTentacle(k),k)==xgnv(EndIndexofTentacle(k)-1,k)
        if ygnv(EndIndexofTentacle(k),k)>ygnv(EndIndexofTentacle(k)-1,k)
            AngleOfTentacle=pi/2;
        else
            AngleOfTentacle=-pi/2;
        end
    elseif xgnv(EndIndexofTentacle(k),k)>xgnv(EndIndexofTentacle(k)-1,k)
        SlopeOfTentacle=((ygnv(EndIndexofTentacle(k),k)-ygnv(EndIndexofTentacle(k)-1,k)))/((xgnv(EndIndexofTentacle(k),k)-xgnv(EndIndexofTentacle(k)-1,k)));
        AngleOfTentacle=atan(SlopeOfTentacle); 
    else
        SlopeOfTentacle=((ygnv(EndIndexofTentacle(k),k)-ygnv(EndIndexofTentacle(k)-1,k)))/((xgnv(EndIndexofTentacle(k),k)-xgnv(EndIndexofTentacle(k)-1,k)));
        if SlopeOfTentacle>0
            AngleOfTentacle=atan(SlopeOfTentacle)-pi;
        else
            AngleOfTentacle=atan(SlopeOfTentacle)+pi;
        end
    end
    
    %求出两者夹角
    alpha=[alpha,abs(AngleOfTentacle-AngleOfRP)]; 
end


maxalpha=max(alpha);
minalpha=min(alpha);

Valpha=(alpha-minalpha)/(maxalpha-minalpha);

end




