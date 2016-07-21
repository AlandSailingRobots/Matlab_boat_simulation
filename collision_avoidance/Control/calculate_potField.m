    % Potential field computation
    function Z=calculate_potField(haveToAvoidObstacle,headingOnlyMode,P1,P2,...
                    x,phat,qhat,rq,bearingDetectedObstacle,sailingZone,psi) 
        %% Init
        windHeading = psi;
        % Obstacle and objective definition
        obj = phat;
        boat = x(1:2);
        boatHeading = x(3);

        %% Obstacle potential function
        if(headingOnlyMode==0)
            if(isempty(qhat)==1)
                ObsP=[];
            else
                ObsP=P1*0;

                scaleHole = 50;
                scalePike = 550;
                scale = 0.5;
                strengthHoles = 2;
                strengthPike = 4;
                strength = 5;
                offsetObstacle = 10;
                % Add the pikes
                for i=1:size(qhat,2)
                    xObs = P1-qhat(1,i);
                    yObs = P2-qhat(2,i);
                    tPike = ((xObs*1).^2+(yObs-offsetObstacle*scale).^2)/scalePike/scale;
                    ObsP = max(ObsP, strength*strengthPike*exp(-(tPike).^2));
                end
                % Then add the holes
                for i=1:size(qhat,2)
                    T = mod(boatHeading,2*pi)*0.3 + mod( atan2(phat(2)-qhat(2,i),phat(1)-qhat(1,i)) ,2*pi)*0.7 + pi/2;
                    xObs = P1-qhat(1,i);
                    yObs = P2-qhat(2,i);
                    xo =  xObs*cos(T) + yObs*sin(T);
                    yo = -xObs*sin(T) + yObs*cos(T);
                    tHoleR = ((xo-35*scale).^2+(yo-offsetObstacle*scale).^2)/scaleHole/scale;
                    tHoleL = ((xo+35*scale).^2+(yo-offsetObstacle*scale).^2)/scaleHole/scale;

                    ObsP = ObsP - strength*strengthHoles*exp(-(tHoleR).^2) - strength*strengthHoles*exp(-(tHoleL).^2);
                end
            end
        else
            if(haveToAvoidObstacle==0)
                ObsP=[];
            else
                eta = 3;
                
                bearingObstacle=x(3)+bearingDetectedObstacle-pi/2;
                lengthObstacle=rq*eta;
                x1 = P1-x(1);
                y1 = P2-x(2);

                xb =  x1*cos(bearingObstacle) + y1*sin(bearingObstacle);
                yb = -x1*sin(bearingObstacle) + y1*cos(bearingObstacle);
%                 intero=rectangularPulse(-lengthObstacle,lengthObstacle,xb);
                ObsP = (15*rectangularPulse(2*lengthObstacle,xb)-5*rectangularPulse(lengthObstacle*4,xb)).*heaviside(yb);
            end
        end

        %% Objective potential function
%         xObj = P1-obj(1);
%         yObj = P2-obj(2);
%         ObjP = 2*exp((-xObj.^2-yObj.^2)/25000);
        Ao = 1.6;
        ObjP = Ao*exp((-(P1-obj(1)).^2 - (P2-obj(2)).^2)/25000);

        %% Wind preference potential function
        x1 = P1-boat(1);
        y1 = P2-boat(2);
        T = windHeading+3*pi/4;
        xw =  x1*cos(T) + y1*sin(T);
        yw = -x1*sin(T) + y1*cos(T);

        WindP = 3*atan(xw).*heaviside(xw).*atan(yw).*heaviside(yw);
     
        %% Boat preference
        strengthBoat = 3;
        strengthHoleBoat = 1.5;
        strengthPikeBoat = 5;
        BoatP = strengthBoat*(-strengthHoleBoat*exp((-(P1-boat(1)).^2 - (P2-boat(2)).^2)/4000) + ...
                               strengthPikeBoat*exp((-((P1-boat(1))*1).^2 - ((P2-boat(2)).*1).^2)/200));

        %% Computation of the field
        if(isempty(ObsP)==1)
            Z = 10*(-ObjP);
        else
            Z = ObsP - ObjP + BoatP + WindP;
%             Z = ObsP - BoatP + WindP;
        end
        
        %% Adding Sailing Zone
        Z = Z + 10*sailingZone; 
%         Z = sailingZone; 
        
    end   