function [ p ] = getP( T )
%   Get the position matrix from the transformation matrix
    p = T(1:3,end);
end

