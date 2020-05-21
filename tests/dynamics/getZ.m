function [ p ] = getZ( T )
%   Get the z axis vector matrix from the transformation matrix or rotation
%   matrix
    R = getR(T);
    p = R(1:3,3);
end



