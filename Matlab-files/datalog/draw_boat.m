function [h] = draw_boat(h,s,x,y,a_b,a_r,a_s)

%h - input: handle to last drawn boat
%s - scale of boat, equals width
%x - coordinate
%y - coordinate
%a_b - boats heading
%a_r - rudder angle in b-frame
%a_s - sail angle in b-frame


%Calculate corners 
p1 = [x+s*cos(a_b) y+s*sin(a_b)];
p2 = [x-s*cos(a_b)+s/2*cos(pi/2-a_b) y-s*sin(a_b)-s/2*sin(pi/2-a_b)];
p3 = [x-s*cos(a_b)-s/2*cos(pi/2-a_b) y-s*sin(a_b)+s/2*sin(pi/2-a_b)];
pr1 = [x-s*cos(a_b) y-s*sin(a_b)];
pr2 = [x-s*cos(a_b)-s*cos(a_b+a_r) y-s*sin(a_b)-s*sin(a_b+a_r)];
ps = [x-s*cos(a_b+a_s) y-s*sin(a_b+a_s)];

%delete old boat
delete(h)
%draw lines
h(1) = line([p2(1) p1(1)],[p2(2) p1(2)],'color','k');
h(2) = line([p3(1) p1(1)],[p3(2) p1(2)],'color','k');
h(3) = line([p2(1) p3(1)],[p2(2) p3(2)],'color','k');
h(4) = line([pr1(1) pr2(1)],[pr1(2) pr2(2)],'color','k');   %rudder
h(5) = line([x ps(1)],[y ps(2)],'color','k');           %sail

end
