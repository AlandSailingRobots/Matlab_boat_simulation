function posObstacles = moveObstacle(posObstacles,elemts,vObs,dt)   % elemt example : [i1 i2 i3]
                                                    % vObs example : [0 0 vxObs3 0 vxObs5 vxObs6;
                                                    %                 0 0 vyObs3 0 vyObs5 vyObs6]
    for i=elemts
        posObstacles(:,i) = posObstacles(:,i)+[vObs(1,i)*dt;vObs(2,i)*dt];
    end
end