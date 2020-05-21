function [ R ] = getR( T )
%   Return the rotation matrix from the transformation matrix
    R = T(1:3,1:3);
end

