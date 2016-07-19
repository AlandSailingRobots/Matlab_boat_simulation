%drag and lift coefficents for rudder. NACA 0015


a_r = [0 4 8 12 16 20 23 24]; %angle of attack
a_r = a_r./180.*pi; %angle of attack
C_L = [.01 .23 .44 .67 .90 1.10 1.25 .8];
C_D = [.01 .02 .03 .05 .1 .16 .20 .24];

plot(a_r,C_L,'r*',a_r,C_D,'b*')
legend('C_L','C_D','location','best')

P_cl = polyfit(a_r(1:end-1),C_L(1:end-1),1)
P_cd = polyfit(a_r(1:end-1),C_D(1:end-1),1)

hold on
plot([a_r(1) a_r(end-1)],[P_cl(2) P_cl(2)+P_cl(1)*a_r(end-1)],'r')
plot([a_r(1) a_r(end-1)],[P_cd(2) P_cd(2)+P_cd(1)*a_r(end-1)],'b')
hold off
