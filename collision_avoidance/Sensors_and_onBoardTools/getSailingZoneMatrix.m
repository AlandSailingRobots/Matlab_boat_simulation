function sailingZoneMatrix = getSailingZoneMatrix(sailingZone,P1,P2,dim)
    sailingZoneMatrix = ones(size(P1));
    %xmin = dim(1); xmax = dim(2); ymin = dim(3); ymax = dim(4);
    %P1 = P1 - (xmax - xmin);
    %P2 = P2 - (ymax - ymin);
    for i=1:size(sailingZone,2)
        j = mod(i,size(sailingZone,2))+1;
        x1 = sailingZone(1,i);
        x2 = sailingZone(1,j);
        y1 = sailingZone(2,i);
        y2 = sailingZone(2,j);
 
        a = [x1;y1];
        b = [x2;y2];
        m = [0;0];
        e = det([(b-a)/norm(b-a), m-a]);

        T = atan2(y2-y1,x2-x1); 
        Y = P2;
        X = P1;
        x = (X)*cos(T)+(Y)*sin(T);
        y = (X)*cos(T+pi/2)+(Y)*sin(T+pi/2);
        line = 1-heaviside(y+e);
                
%         figure(2)
%         contourf(P1,P2,line); hold on;
%         plot(x1,y1,'owhite','linewidth',3);
%         plot(x2,y2,'ogreen','linewidth',3);
%         hold off;
        
        sailingZoneMatrix = sailingZoneMatrix .* line;
        
%         figure(3)
%         contourf(P1,P2,sailingZoneMatrix); hold on;
%         plot(x1,y1,'owhite','linewidth',3);
%         plot(x2,y2,'ogreen','linewidth',3);hold off;
    end
    sailingZoneMatrix = 1-sailingZoneMatrix;
end