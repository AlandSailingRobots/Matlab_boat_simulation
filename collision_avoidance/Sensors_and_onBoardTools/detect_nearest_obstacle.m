%Detect nearest obstacle
function [distNearestObstacle,nearestObstacle] = detect_nearest_obstacle(x,qhat)
    distNearestObstacle = 1000;
    nearestObstacle=[1000;1000];
    for i=1:size(qhat,2)
        distNearestObstacle = min(norm(x(1:2)-qhat(:,i)),distNearestObstacle);
        if(min(norm(x(1:2)-qhat(:,i)),distNearestObstacle)==min(norm(x(1:2)-qhat(:,i))))
            nearestObstacle=qhat(:,i);
        end
    end
end