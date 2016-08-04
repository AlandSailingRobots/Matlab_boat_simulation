%Mock detection return the detected obstacle when the boat is going toward the obstacle
function [detectedObstacles,bearingDetectedObstacle] = obstacle_detection(x,posObstacles,distDetect,angleDetect)
detectedObstacles = [];
bearingDetectedObstacle = [];
for i=1:size(posObstacles,2)
    if(sqrt((posObstacles(1,i)-x(1))^2+(posObstacles(2,i)-x(2))^2) < distDetect)
        theta0 = atan2((posObstacles(2,i)-x(2)),(posObstacles(1,i)-x(1))) - (mod(x(3)+pi,2*pi)-pi);%To check if the boat is heading toward the obstacle.
        if(abs(theta0)<=angleDetect)
            detectedObstacles = [detectedObstacles posObstacles(:,i)];
            bearingDetectedObstacle = theta0;
        end
    end
end
end