%clear all

global k j;
j = 0;
%make plots ??
make_plots = 1; %if zero no plots will be presented


%Wind
a = 4;     %Values from Jaulin 4m/s and angle: pi
psi = -pi/2;
W = [psi a];

%Line to follow - [x1 y1 x2 y2 ...]
D = [0 0 100 0 150 25 100 220 0 150 -25 -25 150 -50 200 200];
k = 0;      % sail towards point k+1

% Specify a time vector
T_start = 0;
T_stop = 60; %220
T_steplength = .1;
T = [T_start:T_steplength:T_stop]';
steps = length(T);


%Controller set up
u = zeros(length(T),3);     %control signals will be saved in u for plots
q = 1;                      %init tack stat variable q


%Initial values Y0 = [x0 y0 theta0 v0 omega0];
Y0 = [0 0 pi/4 1 0]; % x,y,theta,v,omega

Y = zeros(steps,5);     %Matrix with all states through time
Y(1,:) = Y0;

for i=1:steps-1
    %Call controller function
    [delta_r, delta_s, q] = controller(Y(i,:), q, W, D);
    U = [delta_r delta_s];
    u(i,:) = [delta_r, delta_s, q];     %save control signals for plot
    
    %Solve differential equations using ode45 with default settings.
    [t,y] = ode45(@(t,X)model(t,X,W,U), [T(i) T(i+1)], Y(i,:));
    
    %save states. used as input next interation and later also for plots
    Y(i+1,:) = y(end,:);
    
end


%%

if make_plots
    
    %Plot of all five state variables as functions of time
    figure(1)
    plot(T,Y(:,1),T,Y(:,2),T,Y(:,3),T,Y(:,4),T,Y(:,5))
    legend('x','Y','theta','v','omega','location','best')
    
    
    %Plot control signals
    figure(3)
    plot(T,u(:,1),T,u(:,2),T,u(:,3),'*');
    legend('\delta_r','\delta_s','q')
    
    
    %set up spatial trajectory figure
    figure(2)
    clf                     %clear current figure
    hold on
    %draw lines to be followed
    for j = 0:length(D)/2-2
        plot([D(2*j+1) D(2*j+3)],[D(2*j+2) D(2*j+4)],'b--')
    end
    
    xlabel('x [m]')
    ylabel('y [m]')
    axis square
    axis_max_l = abs(max([-min(Y(:,1))+max(Y(:,1)) -min(Y(:,2))+max(Y(:,2))]));
    s = axis_max_l*.04;
    axis([min(Y(:,1))-s min(Y(:,1))+axis_max_l+s min(Y(:,2))-s min(Y(:,2)+axis_max_l)+s]);
    
    
    %draw true wind direction
    m_x = min(Y(:,1))+axis_max_l/2;
    m_y = min(Y(:,2))+axis_max_l/2;
    x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
    y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
    line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
    line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
    line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
    
    h_boat = draw_boat([],0,0,0,0,0,0); %call to get correct handle
    h_traj = [];        %handle for trajectory
    h_appWind = [];     %handel for apparent wind
    
    % M(length(Y(:,1))) = struct('cdata',[],'colormap',[]);
    % vidObj = VideoWriter('film.avi');
    % vidObj.Quality = 50;
    % open(vidObj);
    
    for i = 1:length(Y(:,1))
        delete([h_traj h_appWind])      %delete old trajectory. Saves memory...??
        h_traj = plot(gca(2),Y(1:i,1),Y(1:i,2),'r');      %draw spatial trajectory
        h_boat = draw_boat(h_boat,s,Y(i,1),Y(i,2),Y(i,3),u(i,1),u(i,2));  %draw sailboat
        
        %draw apparent wind direction
        m_x = min(Y(:,1))+axis_max_l/2+axis_max_l/6;
        m_y = min(Y(:,2))+axis_max_l/2;
        W_ap = [a*cos(psi-Y(i,3))-Y(i,4) a*sin(psi-Y(i,3))];
        psi_ap = atan2(W_ap(2),W_ap(1))+Y(i,3);    %apperent wind angle in b-frame
        x_w = [m_x m_x+3*s*cos(psi_ap) m_x+3*s*cos(psi_ap)-s*cos(psi_ap-pi/4) m_x+3*s*cos(psi_ap)-s*cos(psi_ap+pi/4)];
        y_w = [m_y m_y+3*s*sin(psi_ap) m_y+3*s*sin(psi_ap)-s*sin(psi_ap-pi/4) m_y+3*s*sin(psi_ap)-s*sin(psi_ap+pi/4)];
        h_appWind(1) = line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','k');
        h_appWind(2) = line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','k');
        h_appWind(3) = line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','k');
        
        title(['t= ' num2str(T(i)) ' s'])    %Current time in title
        drawnow
        
        
        % M(i) = getframe;
        % writeVideo(vidObj,M(i));
        
        % pause
    end
    
    % close(vidObj);
    
    hold off
end

%% true states saved for filter
X_true = [Y a*ones(length(Y),1) psi*ones(length(Y),1)];
dt = T_steplength;
save('trueStates', 'X_true', 'u', 'T', 'dt')