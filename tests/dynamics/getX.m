function X = getX(R,r)
% to transform a spatial quantity from one from to another
    X = [R zeros(3); skew(r)*R R];
end