clear all

%Wind
a = 2;
psi = -3*pi/4;
W = [psi a];

%Line to follow - [xs ys xf yf]
D = [0 0 200 200];

% Specify a time vector 
T_start = 0;
T_stop = 30;
T_steplength = .5;
T = [T_start:T_steplength:T_stop]';
steps = length(T);


%Controller set up
u = zeros(length(T),3); %save control signals here
q = 1;     %init tack stat variable q


%Initial values Y0 = [x0 y0 theta0 v0 omega0];
Y0 = [0 0 0 1 0]; % x,y,theta,v,omega

Y = zeros(steps,5);     %Matrix with all states through time
Y(1,:) = Y0;

for i=1:steps-1

[delta_r, delta_s, q] = controller_LineFollower(Y(i,:), W, D, q);
U = [delta_r delta_s];
u(i,:) = [delta_r, delta_s, q];     %save control signals for plot

%Solve differential equations using ode45 with default settings. 
[t,y] = ode45(@(t,X)model_2(t,X,W,U), [T(i) T(i+1)], Y(i,:));

%save states. used as input next interation and later also for plots
Y(i+1,:) = y(end,:);

end



%Plot of all five state variables
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
plot([0 100],[0 100])   %draw line to be followed
hold on
xlabel('x [m]')
ylabel('y [m]')
axis square
axis_max_l = abs(max([-min(Y(:,1))+max(Y(:,1)) -min(Y(:,2))+max(Y(:,2))]));
s = axis_max_l*.04;
axis([min(Y(:,1))-s min(Y(:,1))+axis_max_l+s min(Y(:,2))-s min(Y(:,2)+axis_max_l)+s]);


%draw wind direction
m_x = min(Y(:,1))+axis_max_l/2;
m_y = min(Y(:,2))+axis_max_l/2;
x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');

h_boat = draw_boat([],0,0,0,0,0,0); %call to get correct handle
h_traj = [];        %handle for trajectory

% M(length(y(:,1))) = struct('cdata',[],'colormap',[]);
% vidObj = VideoWriter('film.avi');
% open(vidObj);

for i = 1:length(Y(:,1))
    delete(h_traj)      %delete old trajectory. Saves memory...??
    h_traj = plot(Y(1:i,1),Y(1:i,2),'r');      %draw spatial trajectory
    h_boat = draw_boat(h_boat,s,Y(i,1),Y(i,2),Y(i,3),u(i,1),u(i,2));  %draw sailboat
    title(['t= ' num2str(T(i)) ' s'])    %Current time in title
    
    drawnow
%     M(i) = getframe;
%     writeVideo(vidObj,M(i));
    %pause(T_steplength-.05)
end

% close(vidObj);
hold off

