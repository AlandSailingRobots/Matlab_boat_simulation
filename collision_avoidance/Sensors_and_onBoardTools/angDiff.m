function d = angDiff(th1, th2)
    d = mod((th1 - th2)+pi, 2*pi) - pi; % puts everything between [-pi;pi]
end