function collision_avoidance_sailing_boat_main
    %% File path init
    addpath('Control','Sensors_and_onBoardTools','Tools','Waypoint_matrix_and_associated_tools')
    
    %% Drawings
    function draw_circle(c,r,color1,linewidth1)
        for i=1:size(c,2)
            if (exist('linewidth1')==0), linewidth1=1; end;
            if (exist('color1')==0), color1='black'; end;
            s=0:0.01:2*pi;
            w=c(:,i)*ones(size(s))+r*[cos(s);sin(s)];
            plot(w(1,:),w(2,:),color1,'LineWidth',linewidth1);
        end
    end
    function draw_arrow(x,y,theta,L,color)
       e=0.2;
       M1=L*[  0  1  1-e 1 1-e ; 
             0  0   -e 0  e ];
       M = [M1  ; 1 1 1 1 1];  
       R=[cos(theta),-sin(theta),x;sin(theta),cos(theta),y;0 0 1];    
       M =R*M;
       plot(M(1,:),M(2,:),color);       
    end
    function draw_pot(Z)
%         surf(X,Y,Z);
%         contourf(X,Y,Z);        
        [C,h] = contour(X,Y,Z);
        set(h,'LineWidth',2);
        set(h,'LevelStep',2);
    end
    function draw(fig,personMode)
        %% Init
        a = followedLine(:,1);
        b = followedLine(:,2);
        delete(subplot(1,1,fig));
        subplot(1,1,fig);
        %Drawing of the vector field or of the potential function depending on the
        %value of fig
        if(fig==2)
            title('Potential Function');
        end
        if(fig==1)
            title('Potential guidance');
        end
        figure(1);
        if(personMode==3)
            viewRadius = 100;
            axis([x(1)-viewRadius,x(1)+viewRadius,x(2)-viewRadius,x(2)+viewRadius]); hold on; axis image;
        else
            axis([xmin,xmax,ymin,ymax]); hold on; axis image;
        end
        theta=x(3);
        hull=[-1    1    2    2    1   -1   -1   -1;
              -0.5 -0.5 -0.25 0.25 0.5  0.5 -0.5 -0.5 ;
               1    1    1    1    1    1    1    1] ;
        sail=[-2 0;
               0 0;
               1 1];
        rudder=[-0.5 0;
                 0 0;
                 1 1];
        R=[cos(theta),-sin(theta),x(1);sin(theta),cos(theta),x(2);0 0 1];
        hull=R*hull;
        Rdeltas=[cos(deltas),-sin(deltas),1;sin(deltas),cos(deltas),0;0 0 1];
        Rdeltar=[cos(u(1)),-sin(u(1)),-1;sin(u(1)),cos(u(1)),0;0 0 1];
        sail=R*Rdeltas*sail;
        rudder=R*Rdeltar*rudder;
        Mfs=[-1 -1;0 -fs/1000;1 1];
        Mfr=[0 0;0 fr/100;1 1];
        Mfs=R*Rdeltas*Mfs;
        Mfr=R*Rdeltar*Mfr;
        
        %% Background : Potential field and image
%         draw_field(x,phat,qhat,fig);
        draw_pot(Z);
        
        %% Boat
        draw_arrow(x(1),x(2),psi,5*awind,'red');
        plot(hull(1,:),hull(2,:),'black');       
        plot(sail(1,:),sail(2,:),'red');       
        plot(rudder(1,:),rudder(2,:),'red');
        plot(Mfs(1,:),Mfs(2,:),'blue');
        plot(Mfr(1,:),Mfr(2,:),'blue');
        
        %% Line and points
        plot(posWaypoints(1,:),posWaypoints(2,:),'-ogreen','LineWidth',2);
        plot(sailingZone(1,:),sailingZone(2,:),'-oblack','LineWidth',2);
        plot([sailingZone(1,1) sailingZone(1,size(sailingZone,2))],...
             [sailingZone(2,1) sailingZone(2,size(sailingZone,2))],'-oblack','LineWidth',3);
        plot([a(1);b(1)],[a(2);b(2)],'blue');
        plot(x0(1),x0(2),'ogreen','LineWidth',3);
        plot(avoidCollisionPoint(1),avoidCollisionPoint(2),'ocyan','LineWidth',3);
        
        %% Line and the limits of the channel
        plot(phat(1),phat(2),'oblack','LineWidth',3);
        lineAngle = atan2(b(2)-a(2),b(1)-a(1));
        plot( [a(1)-r*sin(lineAngle);
               b(1)-r*sin(lineAngle)] , ...
               ...
              [a(2)+r*cos(lineAngle);
               b(2)+r*cos(lineAngle)] , '-green')
           
        plot( [a(1)+r*sin(lineAngle);
               b(1)+r*sin(lineAngle)] , ...
               ...
              [a(2)-r*cos(lineAngle);
               b(2)-r*cos(lineAngle)] , '-green')
        
        %% Obstacles
        if(isempty(posObstacles)==0)
            for i=1:size(posObstacles,2)
                plot(posObstacles(1,i),posObstacles(2,i),'oblack','LineWidth',1); 
                draw_circle(posObstacles(:,i),rq,'black',1);
            end
        end   
        if(isempty(qhat)==0)
            for i=1:size(qhat,2)
                plot(qhat(1,i),qhat(2,i),'ored','LineWidth',3); 
                draw_circle(qhat(:,i),rq,'red',1);
            end
        end
        if(isempty(collisionnedObstacle)==0)
            plot(collisionnedObstacle(1),collisionnedObstacle(2),'omagenta','LineWidth',3); 
            draw_circle(collisionnedObstacle,rq,'magenta',1);
        end
    
        %% Sensor
        xrange1=x(1:2)+distDetect*[cos(theta+angleDetect);sin(theta+angleDetect)];
        xrange2=x(1:2)+distDetect*[cos(theta-angleDetect);sin(theta-angleDetect)];
        plot([x(1);xrange1(1)],[x(2);xrange1(2)],'green','Linewidth',1);
        plot([x(1);xrange2(1)],[x(2);xrange2(2)],'green','Linewidth',1);

        %% no-go zone
        xngzIN1=x(1:2)+10*[cos(psi+pi+ngzAngle);sin(psi+pi+ngzAngle)];
        xngzIN2=x(1:2)+10*[cos(psi+pi-ngzAngle);sin(psi+pi-ngzAngle)];
        plot([x(1);xngzIN1(1)],[x(2);xngzIN1(2)],'red','Linewidth',1);
        plot([x(1);xngzIN2(1)],[x(2);xngzIN2(2)],'red','Linewidth',1);
        
%         xngzBACK1=x(1:2)+10*[cos(psi+ngzAngleBack);sin(psi+ngzAngleBack)];
%         xngzBACK2=x(1:2)+10*[cos(psi-ngzAngleBack);sin(psi-ngzAngleBack)];
%         plot([x(1)+10;xngzBACK1(1)],[x(2)+10;xngzBACK1(2)+10],'red','Linewidth',1);
%         plot([x(1)+10;xngzBACK2(1)],[x(2)+10;xngzBACK2(2)+10],'red','Linewidth',1);
        
        %NOGOZone OUT
        xngzOUT1=x(1:2)+10*[cos(psi+pi+ngzAngleOUT);sin(psi+pi+ngzAngleOUT)];
        xngzOUT2=x(1:2)+10*[cos(psi+pi-ngzAngleOUT);sin(psi+pi-ngzAngleOUT)];
        plot([x(1);xngzOUT1(1)],[x(2);xngzOUT1(2)],'black','Linewidth',2);
        plot([x(1);xngzOUT2(1)],[x(2);xngzOUT2(2)],'black','Linewidth',2);
        
        %% WantedHeading
        draw_arrow(x(1),x(2),theta_star,20,'black');
        
        %% Detectionsquare
        if(followedLine(:,1)~=avoidCollisionPoint)
            startCollLine = followedLine(:,1);
            collLineAngle = atan2(startCollLine(2)-avoidCollisionPoint(2),startCollLine(1)-avoidCollisionPoint(1));
            collDetectArea = [ -r*sin(r/2) -40  -40  -r*sin(r/2)  ;
                               r/2          r/2 -r/2 -r/2 ];
            collDetectArea = [cos(collLineAngle) -sin(collLineAngle);
                              sin(collLineAngle)  cos(collLineAngle)] * collDetectArea;
            collDetectArea = [collDetectArea(1,:) + avoidCollisionPoint(1); collDetectArea(2,:) + avoidCollisionPoint(2)];
            plot(collDetectArea(1,:),collDetectArea(2,:),'white-');
            draw_circle(avoidCollisionPoint,r,'white',1)
        end
        drawnow();
    end

    %% Simulation   
    %Evolution function
    function  xdot = f(x,u)
        theta=x(3); v=x(4); w=x(5); deltar=u(1); deltasmax=u(2);
        w_ap=[awind*cos(psi-theta)-v;awind*sin(psi-theta)]; %Apparent wind
        psi_ap=atan2(w_ap(2),w_ap(1));   
        a_ap=norm(w_ap);
        sigma=cos(psi_ap)+cos(deltasmax);
        if (sigma<0), deltas=pi+psi_ap;  
        elseif(sigma>=0), deltas=-sign(sin(psi_ap))*deltasmax;  
        end
        fr = p5*v*sin(deltar);  fs = p4*a_ap*sin(deltas-psi_ap);
        dx=v*cos(theta)+p1*awind*cos(psi);
        dy=v*sin(theta)+p1*awind*sin(psi);
        dtheta=w;
        dv=(1/p9)*(sin(deltas)*fs-sin(deltar)*fr-p2*v^2);
        dw=(1/p10)*((p6-p7*cos(deltas))*fs-p8*cos(deltar)*fr-p3*w*v);
        xdot=([dx;dy;dtheta;dv;dw]);       
    end
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
    function play = checkEndSimu(x,phat,posWaypoints,r)
        if( ( all(phat == posWaypoints(:,size(posWaypoints,2))) ) && ...
            ( sqrt( (x(1)-posWaypoints(1,size(posWaypoints,2)))^2 + ...
                    (x(2)-posWaypoints(2,size(posWaypoints,2)))^2) < r ...
            ) ...
          ) % if the boat is next to the objective or an obstacle is too 
            % close to the current objective
            play = 0;
        else
            play = 1;
        end
    end

%% ---------- Main -------------
    init;   
    %% ---- Boat variables ----

    % Init state
    boatSpeed = 5;
    posIniBoatX = 40;
    posIniBoatY = 0;
    headingIniBoat = (0)/180*pi;
    x0 = [posIniBoatX;posIniBoatY;headingIniBoat;boatSpeed;0]; x = x0; %x=(x,y,theta,v,w)
    u0 = [-0.1;0.02]; u = u0; % input
    
    % Global variables drawing
        % To modify this you need to go into the following line function
    ngzAngle = pi/4;
    ngzAngleBack = 0;
    ngzAngleOUT = ngzAngle+pi/8;

    r = 5;    % radius of the corridor in which the boat is supposed to stay when following a line
    thetabar = 0;
    avoidCollisionPoint = x(1:2); % Point created in order to avoid a collision
    
    % Global variables simulation 
    p1=0.1; p2=1; p3=6000; p4=1000; p5=2000;
    p6=1; p7=1; p8=2; p9=300;p10=6000;     
    deltas=0; fs=0; fr=0;
    
    % Init following_line
    q=-1; % side of the line
    phat = []; % Next point to reach
    qhat = []; % List of detected points to avoid
    %startCollLine = [x(1)+10000;x(2)+10000]; % Init collision line
    
    % Global mode variables
    avoidMode = 0;% Mode. When i=0 the boat is heading toward its target. 
                  % When i=1 the boat is following a trajectory in order to avoid an object.
    is_obstacle_detected = 1;
    headingOnlyMode = 1;% If =1 the boat is only detecting the direction of the obstacle
    haveToAvoidObstacle = 0;%In headingMode only, should the boat pass in avoiding mode?
    bearingDetectedObstacle = [];%Used in headingMode only,
    
    % Init follow line mode
    NGZmode = 0;  % No-go zone mode is active when the wanted heading in inside the no-go zone. 
                  % The boat will follow one side of the nogozone until it
                  % reached one side of the channel or the wanted heading is
                  % far from the no-go zone.
    
    % Init sensors
    distDetect = 20; angleDetect = pi/4;     
    
    %% ---- World variables ---- 
    
    % -- Global var simulation-- 
    awind = 3; % wind speed
%     psi = -3;  % angle of the wind
    psi = (-45)/180*pi;  % angle of the wind 
    
    % -- For real simulation in Aland bay --
    start_lat =  60.1074 ; start_lon =  19.9218 ;
    [gps_start_x,gps_start_y]= ll2utm(start_lat,start_lon);
    % Waypoints
    gps_absolute_waypoints = load('waypoints.mat');
    gps_waypoints = ...
        [gps_absolute_waypoints.utm_x - gps_start_x ;...
         gps_absolute_waypoints.utm_y - gps_start_y];
    % Sailing zone
    gps_absolute_sailingzone = load('sailing_zone.mat');
    gps_sailing_zone = ...
        [gps_absolute_sailingzone.utm_x - gps_start_x ;...
         gps_absolute_sailingzone.utm_y - gps_start_y];
    % Obstacles
    gps_absolute_obstacles = load('obstacles.mat');
    gps_obstacles = ...
        [gps_absolute_obstacles.utm_x - gps_start_x ;...
         gps_absolute_obstacles.utm_y - gps_start_y];
    
    % -- For other simulations --
    posWaypoints = gps_waypoints;
%     posWaypoints=[10 -20 ; %chain of waypoints
%                   0   0];
%     posWaypoints=[ 30 -40 -20; %chain of waypoints
%                    0   5 -15];

    
% Transposition is only used because 2 codes had to be mixed together.              
        
    % -- Obstacle --
    
    posObstacles = gps_obstacles;
%     posObstacles = [50 10 45 80;
%                     50 10 65 75];
%     posObstacles = create_wall([-10;-10],...
%                                [ 0; 0],0.5); % Real points to avoid
%     posObstacles = [0;0];


%     posObstacles = [ -30;
%                      -5];
%     vObs = [0 1 0 -1;
%             0 1 0 -1] ;
%     vObs = [0;
%             0];

    vObs = [];
    if(isempty(vObs)==0)
        movingObstacles = find(vObs(1,:)~=0 & vObs(2,:)~=0);
    end
    
    % -- Sailing zone --
    % turn clockwise for the points
    sailingZone = gps_sailing_zone;
%     sailingZone = [-50 -50 50  50 ;
%                    -50  50 50 -50];
          
    % -- Objective --
    phat=posWaypoints(:,1); % Current objective   
    
    % -- Global var drawing --
    xmin=min(sailingZone(1,:))-20;xmax=max(sailingZone(1,:))+20;
    ymin=min(sailingZone(2,:))-20;ymax=max(sailingZone(2,:))+20;   
    rq=5; % size of the object for detection (a security margin is advised)
    X=xmin:5:xmax;
    Y=ymin:5:ymax;   
    
    % -- Line --
    a0=x(1:2) ;b0=phat; % Initial line to follow
    followedLine = [a0 b0]; % Line to follow
    
    % -- Potential field --
    [P1,P2]=meshgrid(X,Y);
    
    dim = [xmin xmax ymin ymax];
    sailingZoneMatrix = getSailingZoneMatrix(sailingZone,P1,P2,dim);
    Z = calculate_potField(haveToAvoidObstacle,headingOnlyMode,P1,P2,x,...
            phat,qhat,rq,bearingDetectedObstacle,sailingZoneMatrix,psi);
    
    % -- Time --
    t=0;
    dt=0.1; 
    
    % -- Simulation variables --
    play=1;  % while play = 1, the simulation continues
    nWayP=1; % the index of the waypoint to reach next
    
    %% ---- Loop ----
    
    % There shouldn(t be any global variables apart from those used by the
    % drawing functions and the simulation functions.
    
    while(play==1)        
        % Obstacle detection
        [detectedObstacles,bearingDetectedObstacle] = ...
            obstacle_detection( x,...
                                posObstacles,...
                                distDetect,...
                                angleDetect);
                           
        qhat = update_obstacles( qhat,...
                                 detectedObstacles,...
                                 x,...
                                 distDetect,...
                                 angleDetect); % cleaning added
                             
        % The sensors check if a obstacle is detected, then add its coordinates
        % to the obstacle database (=qhat)
        [collisionnedObstacle,...
         avoidMode,...
         is_obstacle_detected,...
         haveToAvoidObstacle] = ...
            boat_on_collision_course(x,...
                                     qhat,...
                                     rq,...
                                     r,...
                                     detectedObstacles,...
                                     avoidMode,...
                                     is_obstacle_detected,...
                                     headingOnlyMode);
             
        % Path following (Waypoint system)
        [nWayP,phat,followedLine,avoidMode] = ...
            next_waypoint(x,...
                          phat,...
                          followedLine,...
                          avoidMode,...
                          collisionnedObstacle, ...
                          posWaypoints,...
                          nWayP,...
                          r);
                      
        play = checkEndSimu(x,phat,posWaypoints,r);
%         play = 1;
        
        % Avoid script
        [avoidCollisionPoint,followedLine,avoidMode,Z] = ...
            avoid_obstacle(haveToAvoidObstacle,...
                           headingOnlyMode,...
                           avoidMode,...
                           followedLine, ...
                           collisionnedObstacle,...
                           bearingDetectedObstacle,...
                           avoidCollisionPoint,...
                           sailingZoneMatrix,...
                           qhat,...
                           phat,...
                           x,...
                           P1,...
                           P2,...
                           psi,...
                           rq,...
                           r,...
                           dim,...
                           Z); % using global var
        
        % Command
        [u,q,NGZmode,theta_star] = follow_line(x,...
                                               q,...
                                               psi,...
                                               followedLine,...
                                               NGZmode);
        
        % Simulation
        if(isempty(vObs)==0)
            posObstacles = moveObstacle(posObstacles,movingObstacles,vObs,dt);
        end
        x=x+f(x,u)*dt;
        draw(1,3); % Most of the needed variables are global for the drawing
    end
end
