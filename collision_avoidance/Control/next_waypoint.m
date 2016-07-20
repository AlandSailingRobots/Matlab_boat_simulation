% Path following (waypoint system)
function [nWayP,phat,followedLine,avoidMode]=next_waypoint(x,phat,followedLine,avoidMode,collisionnedObstacle,posWaypoints,nWayP,r)
    % followedLine(:,1) = a1;
    % followedLine(:,2) = b1;
    if((sqrt((x(1)-phat(1))^2+(x(2)-phat(2))^2)<r)||((isempty(collisionnedObstacle)==0)&&(detect_nearest_obstacle(phat,collisionnedObstacle)<r)) )%update the waypoint to reach if the previous one has been reached
        % If the boat reach the objective or some obstacles are too
        % close, the next waypoint is called
        nWayP=nWayP+1;
        if(nWayP<=size(posWaypoints,2)) % If index hasn't overreached last waypoint index
            phat=posWaypoints(:,nWayP);
            followedLine(:,1) = x(1:2); followedLine(:,2) = phat;
            avoidMode = 0; % allowed to compute a new point.
%                 qhat2=qhat; qhat=[];
%                 Z1=calculate_potField(P1a,P2a,x,phat,qhat); % seulement pour l'affichage
%                 qhat=qhat2;
        end
    end
end