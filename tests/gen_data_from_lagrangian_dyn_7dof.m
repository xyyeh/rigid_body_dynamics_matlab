format short
addpath('./dynamics');

swapAngularWithLinear = 1;

% kinematics
syms q1 q2 q3 q4 q5 q6 q7
syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
pi = sym('pi');
q = [q1; q2; q3; q4; q5; q6; q7];
dq = [dq1; dq2; dq3; dq4; dq5; dq6; dq7];
dof = size(q,1);

% information
q_v = [1;2;3;4;5;6;7];%[1;2;3;4];
dq_v = [5;6;7;8;9;10];%[5;6;7;8];
disp('Running with the following parameters');
disp('q'); disp(q_v')
disp('dq'); disp(dq_v')

if (swapAngularWithLinear) 
  disp('WARNING: Task space vectors are defined as [linear; angular]')
  S = eye(6);
else
  S = [zeros(3) eye(3); eye(3) zeros(3)];
end

% forward kinematics
T01 = transformSym(sym(0),      sym(0), 0.333,  q1);
T12 = transformSym(-pi/sym(2),  sym(0),	sym(0),	q2);
T23 = transformSym(pi/sym(2),   sym(0),	0.316,  q3);
T34 = transformSym(pi/sym(2),   0.0825, sym(0),	q4);
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T04ofs = T04*[
  0.540302305868140  -0.841470984807897   0   1
  0.841470984807897   0.540302305868140   0   2
                  0                   0   1   3
                  0                   0   0   1];
         
% jacobian
p04 = getP(T04);
Jv4 = [diff(p04,q1) diff(p04,q2) diff(p04,q3) diff(p04,q4)];
Jw4 = [getZ(T01) getZ(T02) getZ(T03) getZ(T04)];
J = [Jv4; Jw4];

% jacobian at offset
p04ofs = getP(T04ofs);
Jv4ofs = [diff(p04ofs,q1) diff(p04ofs,q2) diff(p04ofs,q3) diff(p04ofs,q4)];
J_ofs = [Jv4ofs; Jw4];

% check task space pose
T04_v = double(subs(T04, [q; dq], [q_v; dq_v]));
T04ofs_v = double(subs(T04ofs, [q; dq], [q_v; dq_v]));

% check jacobian of last body at world frame
J04_v = S*double(subs(J, [q; dq], [q_v; dq_v]));
% check body velocity at world frame
v04_v = J04_v*dq_v;

% check jacobian of last body at world frame with offset
J05ofs_v = S*double(subs(J_ofs, [q; dq], [q_v; dq_v]));

% find bias acceleration
dJ = sym(zeros(size(J)));
for i=1:size(J,1)
  for j=1:size(J,2)
    dJ(i,j) = diff(J(i,j),q1)*dq1 + diff(J(i,j),q2)*dq2 + ...
              diff(J(i,j),q3)*dq3 + diff(J(i,j),q4)*dq4;
  end
end
h04 = dJ*dq;
% check dJ
dJ04_v = S*double(subs(dJ, [q; dq], [q_v; dq_v]));
% check bias acceleration
h04_v = S*double(subs(h04, [q; dq], [q_v; dq_v]));

% find bias acceleration with offset
dJ_ofs = sym(zeros(size(J_ofs)));
for i=1:size(J_ofs,1)
  for j=1:size(J_ofs,2)
    dJ_ofs(i,j) = diff(J_ofs(i,j),q1)*dq1 + diff(J_ofs(i,j),q2)*dq2 + ...
                  diff(J_ofs(i,j),q3)*dq3 + diff(J_ofs(i,j),q4)*dq4;
  end
end
h05 = dJ_ofs*dq;
% check dJ
dJofs_v = S*double(subs(dJ_ofs, [q; dq], [q_v; dq_v]));
% check bias acceleration
h05_v = S*double(subs(h05, [q; dq], [q_v; dq_v]));

disp('Kinematics:');
disp('T'); disp(T04_v)
disp('Tofs'); disp(T04ofs_v)
disp('J'); disp(J04_v)
disp('J_ofs'); disp(J05ofs_v);
disp('dJ'); disp(dJ04_v)
disp('dJ_ofs'); disp(dJofs_v);

% dynamics
% check mass matrix
m1 = 2.74;
c1 = [0; -0.0324958; -0.0675818];
I1 = [0.0180416958283 0               0
      0               0.0159136071891 0.0046758424612
      0               0.0046758424612 0.00620690827127];
m2 = 2.74;
c2 = [0; -0.06861; 0.0322285];
I2 = [0.0182856182281 0                 0
      0               0.00621358421175  -0.00472844221905
      0               -0.00472844221905 0.0161514346309];

m3 = 2.38;
c3 = [0.0469893; 0.0316374; -0.031704];
I3 = [0.00771376630908  -0.00248490625138 -0.00332147581033
      -0.00248490625138 0.00989108008727  -0.00217796151484
      -0.00332147581033 -0.00217796151484 0.00811723558464];

m4 = 2.38;
c4 = [-0.0360446; 0.0336853; 0.031882];
I4 = [0.00799663881132  0.00347095570217 -0.00241222942995
      0.00347095570217  0.00825390705278 0.00235774044121
      -0.00241222942995 0.00235774044121 0.0102515004345];

p0c1 = T01*[c1; 1]; p0c1 = p0c1(1:3);
p0c2 = T02*[c2; 1]; p0c2 = p0c2(1:3);
p0c3 = T03*[c3; 1]; p0c3 = p0c3(1:3);
p0c4 = T04*[c4; 1]; p0c4 = p0c4(1:3);

z = [0;0;0];
Jv1 = [diff(p0c1,q1) z z z];
Jv2 = [diff(p0c2,q1) diff(p0c2,q2) z z];
Jv3 = [diff(p0c3,q1) diff(p0c3,q2) diff(p0c3,q3) z];
Jv4 = [diff(p0c4,q1) diff(p0c4,q2) diff(p0c4,q3) diff(p0c4,q4)];
M_linear = m1*transpose(Jv1)*Jv1 + ...
           m2*transpose(Jv2)*Jv2 + ...
           m3*transpose(Jv3)*Jv3 + ...
           m4*transpose(Jv4)*Jv4;
J1w1 = transpose(getR(T01))*[getZ(T01) z z z];
J2w2 = transpose(getR(T02))*[getZ(T01) getZ(T02) z z];
J3w3 = transpose(getR(T03))*[getZ(T01) getZ(T02) getZ(T03) z];
J4w4 = transpose(getR(T04))*[getZ(T01) getZ(T02) getZ(T03) getZ(T04)];
M_angular = transpose(J1w1)*I1*J1w1 + ...
            transpose(J2w2)*I2*J2w2 + ...
            transpose(J3w3)*I3*J3w3 + ...
            transpose(J4w4)*I4*J4w4;
M = M_linear + M_angular;
M_v = double(subs(M, [q; dq], [q_v; dq_v]));

% check gravity compensation vector
g = [0;0;-9.81];
G = -transpose(Jv1)*(m1*g)-transpose(Jv2)*(m2*g)...
    -transpose(Jv3)*(m3*g)-transpose(Jv4)*(m4*g);
G_v = double(subs(G, q, q_v));

% compute C
[B,C] = computeBCMatricesSym(M,q);
dq_2 = sym(zeros(size(dq)));
for i = 1:numel(dq)
  dq_2(i) = dq(i)*dq(i);
end
dqdq = sym(zeros(size(B,2),1));
i = 1;
j = i+1;
for m = i:size(B,2)
  dqdq(m) = dq(i)*dq(j);
  j = j+1;
  if(j > numel(dq))
    i = i+1;
    j= i+1;
  end
end
C = B*dqdq+C*dq_2;
C_v = double(subs(C, [q; dq], [q_v; dq_v]));

disp('Dynamics:');
disp('M'); disp(M_v)
disp('C'); disp(C_v)
disp('G'); disp(G_v)

rmpath('./dynamics');

