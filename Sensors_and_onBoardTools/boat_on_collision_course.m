function [collisionnedObstacle,avoidMode,is_obstacle_detected,haveToAvoidObstacle]...
         = boat_on_collision_course(x,qhat,rq,r,detectedObstacles,avoidMode,is_obstacle_detected,headingOnlyMode)
    collisionnedObstacle = [];
    for i=1:size(qhat,2)
        distBoatObstacle = norm(qhat(:,i)-[x(1);x(2)]);
        coneAngle = atan2(rq,distBoatObstacle);
        boatHeading = mod(x(3),2*pi);
        obstacleHeading = mod(atan2(qhat(2,i)-x(2),qhat(1,i)-x(1)),2*pi);
        bool = abs(boatHeading-obstacleHeading) < coneAngle;
        if((abs(boatHeading-obstacleHeading) < coneAngle) && (distBoatObstacle < (r*2)) && (x(4)>1)) % Trajectory will cross the security perimeter
            collisionnedObstacle = qhat(:,i);
        end   
    end
    
    if(headingOnlyMode==1)
        collisionnedObstacle = detectedObstacles;
    end
    
    if(isempty(collisionnedObstacle)==0)
        haveToAvoidObstacle=1;
    else
        haveToAvoidObstacle=0;
    end
        
    if(isempty(collisionnedObstacle)==1)
        is_obstacle_detected=0;
    elseif(isempty(collisionnedObstacle)==0 && is_obstacle_detected==0)
        is_obstacle_detected=1;
        avoidMode = 0;
    end

end
