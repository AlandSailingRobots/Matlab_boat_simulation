clear all
global q i u D W;

%Wind
a = 4;     %Values from Jaulin 4m/s and angle: pi
psi = pi/2;
W = [psi a];

%Destination
D = [0 0 50 50];    %line 

% Specify a time vector to decide at which time points to evaluate model
% else set start and stop value and let ode45 decide step length
T_start = 0;
T_stop = 10;
T_steplength = .1;
T = [T_start:T_steplength:T_stop]';
% T = [T_start T_stop];

%Control signals
u = zeros(length(T),3);
q = 1;     %init q;

%statistics
i = 1;

%Initial values X0 = [x0 y0 theta0 v0 omega0];
X0 = [0 0 0 1 0]; % x,y,theta,v,omega

%model is specified in a different .m-file as a function
model = @(t,X)model(t,X);

%Solve differential equations using ode45 with default settings. 
options = odeset('RelTol',1,'AbsTol',1,'MaxStep',.001);
[t,y] = ode45(model, T, X0);

%Plot of all five state variables
figure(1)
plot(t,y(:,1),t,y(:,2),t,y(:,3),t,y(:,4),t,y(:,5))
legend('x','y','theta','v','omega','location','best')


%set up spatial trajectory figure
figure(2)
clf         %clear current figure
plot([0 100],[0 100])
hold on
xlabel('x [m]')
ylabel('y [m]')
axis square
axis_max_l = abs(max([-min(y(:,1))+max(y(:,1)) -min(y(:,2))+max(y(:,2))]));
s = axis_max_l*.04;
axis([min(y(:,1))-s min(y(:,1))+axis_max_l+s min(y(:,2))-s min(y(:,2)+axis_max_l)+s]);


%draw wind direction
m_x = min(y(:,1))+axis_max_l/2;
m_y = min(y(:,2))+axis_max_l/2;
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

for i = 1:length(y(:,1))
    delete(h_traj)      %delete old trajectory. Saves memory...??
    h_traj = plot(y(1:i,1),y(1:i,2),'r');      %draw spatial trajectory
    h_boat = draw_boat(h_boat,s,y(i,1),y(i,2),y(i,3),u(i,1),u(i,2));  %draw sailboat
    title(['t= ' num2str(t(i)) ' s'])    %Current time in title
    
    drawnow
%     M(i) = getframe;
%     writeVideo(vidObj,M(i));
    %pause(T_steplength-.05)
end

% close(vidObj);
hold off

%[X0(4) y(1,4) y(end,4)]
