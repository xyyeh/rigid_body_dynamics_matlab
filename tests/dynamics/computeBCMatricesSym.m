function [B,C] = computeBCMatricesSym(A, q)
% Computes the symbolic B and C matrices
m = size(A,1);
n = m*(m-1)/2;
B = sym(zeros(m,n));
C = sym(zeros(m,m));

for i = 1:m
    j = 1;
    k = j+1;
    for p = 1:n
        B(i,p) = sym(2)*computeCristoffelSymbolSym(A,q,i,j,k);
        k = k+1;
        if(k == m+1)
            j = j+1;
            k = j+1;
        end
    end
end

for i = 1:m
   for j = 1:m
      C(i,j) = computeCristoffelSymbolSym(A,q,i,j,j); 
   end
end
end

