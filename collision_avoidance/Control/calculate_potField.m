    % Potential field computation
    function Z=calculate_potField(P1,P2,x,phat,qhat,psi) 
        %% Init
        windHeading = psi;
        % Obstacle and objective definition
        obj = phat;
        boat = x(1:2);
        boatHeading = x(3);

        %% Obstacle potential function
        if(isempty(qhat)==1)
            ObsP=[];
        else
            ObsP=P1*0;
            for i=1:size(qhat,2)
                xObs = P1-qhat(1,i);
                yObs = P2-qhat(2,i);
                T = (boatHeading+pi/2)*0.3+(atan2(phat(1)-qhat(1,i),phat(2)-qhat(2,i))+pi/2)*0.7;
%                 T = pi/2+atan2(phat(1)-qhat(1,i),phat(2)-qhat(2,i));
                xo =  xObs*cos(T) + yObs*sin(T);
                yo = -xObs*sin(T) + yObs*cos(T);
                scaleHole = 50;
                scalePike = 550;
                strengthHoles = 2;
                strengthPike = 4;
                strength = 5;

                tHoleR = ((xo*1-35).^2+(yo*1).^2)/scaleHole;
                tHoleL = ((xo*1+35).^2+(yo*1).^2)/scaleHole;
                tPike = ((xo*1).^2+(yo*1).^2)/scalePike;

%                 ObsP = ObsP + strength*(1*(-1./(sqrt((1-tHole.^2).^2+(2*0.3*tHole).^2)))+3*exp(-(tPike).^2));
                ObsP = ObsP + strength*(strengthHoles*(-(exp(-(tHoleR).^2))-(exp(-(tHoleL).^2)))+strengthPike*exp(-(tPike).^2));
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
    end   