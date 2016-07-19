function [avoidCollisionPoint,followedLine,avoidMode] = avoid_obstacle(avoidMode,followedLine,collisionnedObstacle,avoidCollisionPoint,qhat,phat,x,P1a,P2a,psi,rq,r,dim)

    if(isempty(qhat)==0 && isempty(collisionnedObstacle)==0 && avoidMode==0)

        Z=calculate_potField(P1a,P2a,x,phat,qhat,psi);
        avoidCollisionPoint = calculate_avoidCollisionPoint(Z,dim);
        startCollLine = (norm(collisionnedObstacle-x(1:2))+rq+r)*[cos(x(3)+pi);sin(x(3)+pi)]+x(1:2);            

%             a1 = startCollLine;
%             b1 = avoidCollisionPoint;
        followedLine(:,1) = startCollLine;
        followedLine(:,2) = avoidCollisionPoint;
        avoidMode = 1;
    end

    startCollLine = followedLine(:,1);
    collLineAngle = atan2(startCollLine(2)-avoidCollisionPoint(2),startCollLine(1)-avoidCollisionPoint(1));
    if(     (sqrt((x(1)-avoidCollisionPoint(1))^2+(x(2)-avoidCollisionPoint(2))^2)<(r+r/10)...
            || ( ((x(1)-avoidCollisionPoint(1))*cos(collLineAngle)+(x(2)-avoidCollisionPoint(2))*sin(collLineAngle)>-40) ...
                && ((x(1)-avoidCollisionPoint(1))*cos(collLineAngle)+(x(2)-avoidCollisionPoint(2))*sin(collLineAngle)<0) ...
                && (abs(-(x(1)-avoidCollisionPoint(1))*sin(collLineAngle)+(x(2)-avoidCollisionPoint(2))*cos(collLineAngle))<r/2)) )...
        && avoidMode==1)

%             a1=avoidCollisionPoint;
%             b1=phat;
        followedLine(:,1) = avoidCollisionPoint;
        followedLine(:,2) = phat;
        avoidMode = 0;
%             qhat2=qhat;
%             qhat=[];
%             Z1=calculate_potField(P1a,P2a,x,phat,qhat,psi); %Just for drawing
%             qhat=qhat2;
    end
end