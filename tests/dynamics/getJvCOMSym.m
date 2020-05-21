function Jv0cx = getJvCOMSym(p0c, q)
% Returns Jv at the center of mass location in the inertial frame {0}
% q is the joint coordinates
% p0c is the center of mass expressed in frame {0}

dof = numel(q);
Jv0cx = sym(zeros(3, dof));

for i = 1:dof
    Jv0cx(:,i) = diff(p0c,q(i));
end

end

