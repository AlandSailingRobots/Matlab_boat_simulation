function wrap = wrapToPi_2(ang)
% written by Jon Melin 20150716 - returns angle in [-pi pi]
% while ang>pi || ang < -pi
%    if ang>pi
%        ang = ang - 2*pi;
%    elseif ang < -pi
%        ang = ang + 2*pi;
%    end
% end
% wrap = ang;

while ang>pi || ang < -pi
   if ang>pi
       ang = ang - 2*pi;
   elseif ang < -pi
       ang = ang + 2*pi;
   end
end
wrap = ang;