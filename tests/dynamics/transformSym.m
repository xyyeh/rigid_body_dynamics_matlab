function T = transformSym(alp,a,d,t)
% DH transformation (alp,a,d,t)
% ct = cos(t);
% st = sin(t);
% ca = cos(alp);
% sa = sin(alp);

T = [cos(t), -sin(t), sym(0), a;
     sin(t)*cos(alp), cos(t)*cos(alp), -sin(alp), -sin(alp)*d;
     sin(t)*sin(alp), cos(t)*sin(alp), cos(alp), cos(alp)*d;
     sym(0), sym(0), sym(0), sym(1)];

% T = [ct, -st, 0, a;
%      st*ca, ct*ca, -sa, -sa*d;
%      st*sa, ct*sa, ca, ca*d;
%      0, 0, 0, 1];
end