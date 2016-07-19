%test draw

x = 10;
y = 10;

s = 2.5;

a = -pi*1/8;

p1 = [x+s*cos(a) y+s*sin(a)];
%p = [x-s*cos(a) x-s*sin(a)];
p2 = [x-s*cos(a)+s/2*cos(pi/2-a) x-s*sin(a)-s/2*sin(pi/2-a)];
p3 = [x-s*cos(a)-s/2*cos(pi/2-a) x-s*sin(a)+s/2*sin(pi/2-a)];


plot(x,y,'o')
hold on

plot(p1(1),p1(2),'*')
%plot(p(1),p(2),'*')
plot(p2(1),p2(2),'*')
plot(p3(1),p3(2),'*')

line([p2(1) p1(1)],[p2(2) p1(2)],'color','k')
line([p3(1) p1(1)],[p3(2) p1(2)],'color','k')
line([p2(1) p3(1)],[p2(2) p3(2)],'color','k')

axis([5 15 5 15])
legend('X','p2','location','best')
axis square
hold off
