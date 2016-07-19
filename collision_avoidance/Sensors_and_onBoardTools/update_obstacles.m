%Add the obstacles that weren't detected before and clean any obstacles
%detected before that isn't at the same place.
function qhat = update_obstacles(qhat,detectedObstacles,x,distDetect,angleDetect)

    % Cleaning
    i = 1;
    while i<=size(qhat,2);
        T = atan2((qhat(2,i)-x(2)),(qhat(1,i)-x(1)))-x(3);

        % qhat(:,i) does not belong to detectedObstacles
        if(isempty(detectedObstacles)==0)
            qhatBelongs2DetObs = isempty(find(detectedObstacles(1)==qhat(1,i))==find(detectedObstacles(1)==qhat(1,i)));
        else
            qhatBelongs2DetObs = 1;
        end
        % same numbers and same indices (it's posible as well to use ismember matlab function)
        if( ((norm(qhat(:,i)-x(1:2))<=distDetect) && (abs(T)<=angleDetect)) & (qhatBelongs2DetObs) ) % If the obstacle should have been detected and is not
            qhat(:,i) = []; % Remove the undetected obstacle
        end
        i = i + 1;
    end

    % New obstacles
    for i=1:size(detectedObstacles,2);
        % test for each real obstacle if it has already been detected.
        % If not it add it to the list.
        ctr = 0;           
        for j=1:size(qhat,2);
            if(detectedObstacles(:,i)==qhat(:,j))
                ctr = ctr + 1;
            end        
        end
        if(ctr==0);
            qhat = [qhat detectedObstacles(:,i)];
        end
    end
end