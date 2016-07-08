% This function follow the line define with a and b
function [u,q,tacking] = follow_line(x,q, psi, followedLine,tacking)
        
        %controller_simpleLine Simple line following controller
        %   Controller based on the paper "A simple controller for line
        %   following of sailboats" by Luc Jaulin and Fabrice Le Bars
        %Constans/Parameters
        a = followedLine(:,1);
        b = followedLine(:,2);
        m = x(1:2); theta = x(3); v_boat = x(4);
        r = 5;              % m -   cutoff distance
        delta_rMax = pi/4;  % rad   maximum rudder angle
        gamma = pi/4;       % rad   incidence angle
        ngzAngle = pi/4;    % rad   close hauled angle
        ngzAngleBack = 0;    % rad   back wind angle
        % ngzAngleOUT = ngzAngle+pi/8;   % rad   out of the no-go zone angle
        % NGZHeading = mod(psi+pi,2*pi); % rad   no-go zone angle
        
        %Step 3
        phi = atan2((b(2)-a(2)),(b(1)-a(1)));
        
        %Step 1
        u = 1./hypot(b(1)-a(1),b(2)-a(2)).*[b(1)-a(1) b(2)-a(2)];
        v = [m(1)-a(1) m(2)-a(2)];
        e = u(1)*v(2)-v(1)*u(2);
        
        %Step 2
        if (abs(e) > r)
            q = sign(e);
        end
        
        %Step 4
        theta_star = phi-2*gamma/pi*atan(e/r); %take care of the incidence angle
        
        %Step 5-9
        %Boat left or right of the channel limit
        T = atan2(b(2)-a(2),b(1)-a(1));
        aB = a+[sign(e)*r*cos(T+pi/2);sign(e)*r*sin(T+pi/2)];
        bB = b+[sign(e)*r*cos(T+pi/2);sign(e)*r*sin(T+pi/2)];
        uB = 1./hypot(bB(1)-aB(1),bB(2)-aB(2)).*[bB(1)-aB(1) bB(2)-aB(2)];
        vB = [m(1)-aB(1) m(2)-aB(2)];
        eB = uB(1)*vB(2)-vB(1)*uB(2);
        
        % ( (sign(eB)*sign(e)<=0)||( (sign(eB)*sign(e)>0)&&(abs(e)<(r+sign(e)*50)) ) && ...
        if ( (cos(psi-theta_star)+cos(ngzAngle) < 0) || ...     
             (cos(psi-phi)+cos(ngzAngle)<0) ...  
           )
            if ~tacking
%                 v2 = [m(1)+2000*r*cos(theta+pi)-a(1) m(2)+2000*r*sin(theta+pi)-a(2)];
% %                 v2 = [m(1)+100*cos(theta+pi)-a(1) m(2)+100*sin(theta+pi)-a(2)];
%                 e2 = u(1)*v2(2)-v2(1)*u(2);
% %                 q = sign(e2);
                q = sign(theta-(mod(psi,pi)-pi));
                tacking = 1;
            end
            theta_bar = pi + psi - q*ngzAngle;
%         elseif( (cos(psi+pi-theta_star)+cos(ngzAngleBack) < 0) || ...
%               ( (abs(e)<r) && (cos(psi+pi-phi)+cos(ngzAngleBack)<0) ) )
%             if ~tacking
%                 v2 = [m(1)+100*cos(theta+pi)-a(1) m(2)+100*sin(theta+pi)-a(2)];
%                 e2 = u(1)*v2(2)-v2(1)*u(2);
%                 q = sign(e2);
%                 tacking = 1;
%             end
%             theta_bar = psi - q*ngzAngleBack;
        else
            tacking = 0;
            theta_bar = theta_star;
        end
        
        %Step 10-11
        if cos(theta-theta_bar) >= 0
            delta_r = delta_rMax*sin(theta-theta_bar);
        else
            delta_r = delta_rMax*sign(sin(theta-theta_bar));
        end
        
        %Step 12
        delta_sMax = pi/4*(cos(psi-theta_bar)+1);
        
        % Interfacing with the rest of the program
        u = [delta_r;delta_sMax];
    end