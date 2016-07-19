clear all
t = [1 1 2 2 3 4 5 6 7 8 10 10 11 11]

% i = 2;
% T(1) = t(1);
% for j = 2:length(t)
%     if t(j)>T(i-1)
%         T(i) = t(j);
%         i = i + 1;
%         
%     end
% end
index = diff(t)==0;
t(index) = []