function [Ainv] = invSPD(A)
%Matrix inversion using cholesky factorization
    L = chol(A,'lower');
    Ainv = inv(L')*inv(L);
end

