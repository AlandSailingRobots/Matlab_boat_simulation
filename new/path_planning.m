function [a,b,index_out] = path_planning(x,y,index_path,pointx,pointy)

b=[pointx(mod(index_path,length(pointx))+1),pointy(mod(index_path,length(pointx))+1)];
a=[pointx(index_path),pointy(index_path)];

  vOrthoAB = [-(b(2)-a(2)),(b(1)-a(1))];
  vMB = [b(1)-x,b(2)-y];
  % we do MB^BC BC is orthogonol to AB M the position of robot
  is_not_passed = vMB(1)*vOrthoAB(2)-vMB(2)*vOrthoAB(1)>0; %true if we didn't pass through B
if ~is_not_passed
  index_path = mod(index_path,length(pointx))+1;

  b=[pointx(mod(index_path,length(pointx))+1),pointy(mod(index_path,length(pointx))+1)];
  a=[pointx(index_path),pointy(index_path)];

end

index_out = index_path;

end