a = -pi:.1:pi;

C_D = 1-cos(2.*x);
C_L = sin(2.*x);

f = C_D.*sin(a) + C_L.*cos(a);

plot(x,C_D)
hold on
plot(x,C_L)
plot(x,f)
plot(x,sin(a))

hold off