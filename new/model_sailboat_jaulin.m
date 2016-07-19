function dy = model_sailboat_jaulin(y,a,phi,delta_s,delta_r)
theta =y(3);
v=y(4);
omega=y(5);

%Internal constants
p1 = 0.05;          % -         drift coefficientv_norm
p2 = .2;            %kg s^-1    tangential friction
p3 = 6000;          %kg m       angular friction
p4 = 1000;          %kg s^-1    sail lift
p5 = 2000;          %kg s^-1    rudder lift
p6 = 1;             %m          distance to sail CoE
p7 = 1;             %m          distance to mast
p8 = 2;             %m          distance to rudder
p9 = 300;           %kg         mass of boat
p10 = 10000;        %kg m^2     mass moment of intertia


%link equations
W_ap = [a*cos(phi-theta)-v a*sin(phi-theta)];
%apperent wind speed vector in b-frame
phi_ap = atan2(W_ap(2),W_ap(1));    %apperent wind angle in b-frame
a_ap = hypot(W_ap(1),W_ap(2));      %apperent wind speed velocity in b-frame


sigma=cos(phi_ap)+cos(delta_s);
if (sigma<0),
    delta_s=pi+phi_ap;
else
    if sin(phi_ap)~=0
        delta_s=-sign(sin(phi_ap))*delta_s;
    end
end;

Fs = p4*a_ap*sin(delta_s-phi_ap);  %Force of wind on sail
Fr = p5*v*sin(delta_r);             %Force of water on rudder

x_dot =  v*cos(theta) + p1*a*cos(phi);     %x_dot
y_dot =    v*sin(theta) + p1*a*sin(phi);  %y_dot
v_dot =   ((Fs*sin(delta_s)-Fr*sin(delta_r))-sign(v)*(p2*(v)^2))/p9;
theta_dot = omega;                %theta_dot
omega_dot = (Fs*(p6-p7*cos(delta_s))-p8*Fr*cos(delta_r)...
    -p3*omega*norm(v)...
     )/p10; %omega_dot
%*cos(delta_r)
dy = [x_dot,y_dot,theta_dot,v_dot,omega_dot]';

end