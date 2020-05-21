function [ bijk ] = computeCristoffelSymbolSym(A,q,i,j,k)
% Compute christoffel symbols
bijk = sym(1)/sym(2)*(diff(A(i,j),q(k))+diff(A(i,k),q(j))-diff(A(j,k),q(i)));
end

