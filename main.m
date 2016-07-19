path(path,'C:\matlab_sailing\GitRepo\Matlab_boat_simulation\Matlab-files\2 Egen regulator');
path(path,'C:\matlab_sailing\GitRepo\Matlab_boat_simulation\new');
path(path,'C:\matlab_sailing\GitRepo\Matlab_boat_simulation\GPS_POS');
path(path,'C:\matlab_sailing\GitRepo\Matlab_boat_simulation\collision_avoidance/Tools');
path(path,'C:\matlab_sailing\GitRepo\Matlab_boat_simulation\collision_avoidance/Sensors_and_OnboardTools');
path(path,'C:\matlab_sailing\GitRepo\Matlab_boat_simulation\collision_avoidance/Control');
theta = wrapToPi(pi/2-theta/180*pi);
delay = 2; %smith delay in seconds 
size_buffer_delay = 500;

%% timer management
if ~exist('old_time','var')
   i_delay = 1;
   old_time = timer
   dt = 0;
   y_delay = zeros(5,size_buffer_delay);
   time_delay = zeros(1,size_buffer_delay);
   time_t = 0;
else
  if old_time>timer
       old_time = old_time-(2^32-1);
  end
  dt = (timer-old_time)/1000;
  old_time=timer;
end
time_t = time_t+dt;

%% Convert to distance relative start.  60.1074, 19.9218 
start_lat =  60.1074 ;
start_lon =  19.9218 ;
gps_lat = lat; gps_lon = lon; 
[start_x,start_y]= ll2utm(start_lat,start_lon);
[gps_x,gps_y]= ll2utm(gps_lat,gps_lon);

% x
m =[gps_x-start_x,gps_y-start_y] ; % boats position

X = [m(1) m(2) theta v]; % states used by controller
W = [psi_tw a_tw]; % True wind from filter

if ~exist('gps_waypoint','var')
   load('gps_point-moreclose_harbour.mat')
   gps_waypoint = [utm_x - start_x,utm_y - start_y]
   y_prediction = [X 0]';
   j=0;
end

if active_smith && time_t > delay
  pos_sum = y_prediction(1:3)-y_delay(1:3,1);
  m = m + pos_sum(1:2)';
  theta = theta+pos_sum(3);
  j = j+1;
end

while time_t >= time_delay(1) + delay && i_delay>1
    i_delay = i_delay-1;
    y_delay_temp = zeros(5,size_buffer_delay);
    y_delay_temp(:,1:size_buffer_delay-1)  =y_delay(:,2:size_buffer_delay) ;
    y_delay=y_delay_temp;
    time_delay_temp = zeros(1,size_buffer_delay);
    time_delay_temp(1:size_buffer_delay-1)  = time_delay(2:size_buffer_delay) ;
    time_delay = time_delay_temp;
end

%% command code

if obstacle_mode==1
    %% obstacle avoidance
    % [delta_r, delta_s, q, tacking] = controller_simpleLine_v_control(m, theta,v,q, psi_tw,a,b,tacking);
    % delta_r = delta_r/2;

    % Loopback values : tacking, avoidMode, is_obstacle_detected, NGZmode. 
    % Initialized values : 
        % x
        x = [m(1),m(2),theta,v,0] % w is not used in the script
        nWayP = k;
        % Initialization
        if(~exist('posObstacles','var'))
            posObstacles = [1.19   191;
                           -11.16 -284;
                           -1.47  -281;
                           -28.6  -726;
                            46.6   21.1]; % utm coordinates - start
            phat = posWaypoint(:,nWayP);
            qhat = [];
            distDetect = 10; 
            angleDetect = pi/4;
            followedLine = [x(1:2) phat];
            rq = 10;
            avoidCollisionPoint = x(1:2);
            % takes the furthest waypoints and use it as a box with a safety margin
            xmin = min(gps_waypoint(:,1))-50;
            xmax = max(gps_waypoint(:,1))+50;
            ymin = min(gps_waypoint(:,2))-50;
            ymax = max(gps_waypoint(:,2))+50;
            dim = [xmin xmax ymin ymax];
            X = linspace(xmin,xmax,100);
            Y = linspace(ymin,ymax,100);
            [P1,P2] = meshgrid(X,Y);
            
            % vObs = [0 1 0 -1 0
            %         0 1 0 -1 0] ;
            vObs = [];
            if(isempty(vObs)==0)
                movingObstacles = find(vObs(1,:)~=0 & vObs(2,:)~=0);
            end
            
            is_obstacle_detected = 0;
            NGZmode = 0;
            avoidMode = 0;
        end    
    % Obstacle detection
    detectedObstacles = obstacle_detection(x,posObstacles',distDetect,angleDetect);  
    qhat = update_obstacles(qhat,detectedObstacles,x,distDetect,angleDetect); % cleaning added
    % The sensors check if a obstacle is detected, then add its coordinates
    % to the obstacle database (=qhat)
    [collisionnedObstacle,avoidMode,is_obstacle_detected] = boat_on_collision_course(x,qhat,rq,r,avoidMode,is_obstacle_detected);

    % Path following (Waypoint system)
    [nWayP,phat,followedLine,avoidMode] = next_waypoint(x,phat,followedLine,avoidMode,collisionnedObstacle,gps_waypoint',nWayP,r);

    % Avoid script
    [avoidCollisionPoint,followedLine,avoidMode] = avoid_obstacle(avoidMode, followedLine, collisionnedObstacle,avoidCollisionPoint,qhat,phat,x,P1,P2,psi_tw,rq,r,dim); % using global var
    
    % Simulation
    if(isempty(vObs)==0)
        posObstacles = moveObstacle(posObstacles',movingObstacles,vObs,dt);
        posObstacles = posObstacles';
    end
    
    % Command
    [u,q,NGZmode]=follow_line(x,q,psi_tw,followedLine,NGZmode);

    % interface with old code
    delta_r = u(1);
    delta_s = u(2);
elseif(obstacle_mode==2)
    %% obstacle avoidance
    % [delta_r, delta_s, q, tacking] = controller_simpleLine_v_control(m, theta,v,q, psi_tw,a,b,tacking);
    % delta_r = delta_r/2;

    % Loopback values : tacking, avoidMode, is_obstacle_detected, NGZmode. 
    % Initialized values : 
        % x
        x = [m(1),m(2),theta,v,0] % w is not used in the script
        nWayP = k;
        % Initialization
        if(~exist('posObstacles','var'))
            posObstacles = [1.19   191;
                           -11.16 -284;
                           -1.47  -281;
                           -28.6  -726;
                            46.6   21.1]; % utm coordinates - start
            phat = posWaypoint(:,nWayP);
            qhat = [];
            distDetect = 10; 
            angleDetect = pi/4;
            followedLine = [x(1:2) phat];
            rq = 10;
            avoidCollisionPoint = x(1:2);
            % takes the furthest waypoints and use it as a box with a safety margin
            xmin = min(gps_waypoint(:,1))-50;
            xmax = max(gps_waypoint(:,1))+50;
            ymin = min(gps_waypoint(:,2))-50;
            ymax = max(gps_waypoint(:,2))+50;
            dim = [xmin xmax ymin ymax];
            X = linspace(xmin,xmax,100);
            Y = linspace(ymin,ymax,100);
            [P1,P2] = meshgrid(X,Y);
            
            % vObs = [0 1 0 -1 0
            %         0 1 0 -1 0] ;
            vObs = [];
            if(isempty(vObs)==0)
                movingObstacles = find(vObs(1,:)~=0 & vObs(2,:)~=0);
            end
            
            is_obstacle_detected = 0;
            NGZmode = 0;
            avoidMode = 0;
        end    
    % Obstacle detection
    detectedObstacles = obstacle_detection(x,posObstacles',distDetect,angleDetect);  
    qhat = update_obstacles(qhat,detectedObstacles,x,distDetect,angleDetect); % cleaning added
    % The sensors check if a obstacle is detected, then add its coordinates
    % to the obstacle database (=qhat)
    [collisionnedObstacle,avoidMode,is_obstacle_detected] = boat_on_collision_course(x,qhat,rq,r,avoidMode,is_obstacle_detected);

    % Path following (Waypoint system)
    [nWayP,phat,followedLine,avoidMode] = next_waypoint(x,phat,followedLine,avoidMode,collisionnedObstacle,gps_waypoint',nWayP,r);

    % Avoid script
    [avoidCollisionPoint,followedLine,avoidMode] = avoid_obstacle(avoidMode, followedLine, collisionnedObstacle,avoidCollisionPoint,qhat,phat,x,P1,P2,psi_tw,rq,r,dim); % using global var
    
    % Simulation
    if(isempty(vObs)==0)
        posObstacles = moveObstacle(posObstacles',movingObstacles,vObs,dt);
        posObstacles = posObstacles';
    end
    
    % Command
    [u,q,NGZmode]=follow_line(x,q,psi_tw,followedLine,NGZmode);

    % interface with old code
    delta_r = u(1);
    delta_s = u(2);
else
    %% old code
    [a,b,k] = path_planning(m(1) ,m(2),k,gps_waypoint(:,1)',gps_waypoint(:,2)')
    [delta_r,delta_s,q,tacking] = controller_simpleLine_v_control(m,theta,v,q,psi_tw,a,b,tacking);
    delta_r = delta_r/2;
end

%% smith predictor
y_prediction =y_prediction +dt* model_sailboat_jaulin(y_prediction,a_tw,psi_tw,delta_s,delta_r)

y_delay(:,i_delay) = y_prediction;
time_delay(i_delay) = time_t;
i_delay = i_delay+1;
if i_delay>size_buffer_delay
   i_delay = size_buffer_delay;
end
