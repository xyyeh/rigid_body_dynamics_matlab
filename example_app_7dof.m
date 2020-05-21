clear all
close all
clc

addpath('./RBDyn/');
addpath('./third_party/xml2struct') % from: https://www.mathworks.com/matlabcentral/fileexchange/28518-xml2struct
addpath('./third_party/Spatial') % from: http://royfeatherstone.org/spatial/v2/index.html
addpath('./third_party/utils')
addpath('./third_party/urdfs') 

% load robot using urdf
r = RBDyn('panda.urdf');

% set 'candle/home' position
% r.setJointPosition([0;0;0;-pi/2;0;pi/2;0]);
q_desired= [0;0.3;0;-1.57;0;1.87;0];
q_actual = [0.5;-0.2;0.4;-0.2;0.2;0.5;-0.2];
r.setJointPosition(q_actual); %([1;2;3;4;5;6;7]);
r.setJointVelocity([5;6;7;8;9;10;11]);
r.displaySystem

% calculate pose and jacobian at last link frame
r.calcPose
disp('R_0N');
disp(r.R_0N)
disp('P_0N');
disp(r.P_0N)
r.calcBasicJacobianN
disp('J_0N');
disp(r.J_0N)

% calculate pose and jacobian at tcp frame at 20 cm away in z7
S = [zeros(3) eye(3); eye(3) zeros(3)];
T = eye(4);
T(1:3,1:3) = [0.5403023 -0.841471 0; 0.841471 0.5403023 0; 0 0 1];
T(1:3,4) = [1;2;3];
r.calcPoseAt(T,1);
disp('R_0E');
disp(r.R_0E)
disp('P_0E');
disp(r.P_0E)
r.calcBasicJacobianE
disp('J_0E');
disp(S*r.J_0E)

% calculating mass matrix at a particular q
% r.setJointPosition([0;0;0;0;0;0;0]);
r.calcJointSpaceDyn
disp('r.mass_matrix');
disp(r.mass_matrix)
r.calcOperationalSpaceDynE
disp('r.lambda_matrix_invE');
disp(S*r.lambda_matrix_inv*S')
disp('r.lambda_matrixE');
disp(r.lambda_matrix)

rmpath('./RBDyn/');
rmpath('./third_party/xml2struct') % from: https://www.mathworks.com/matlabcentral/fileexchange/28518-xml2struct
rmpath('./third_party/Spatial') % from: http://royfeatherstone.org/spatial/v2/index.html
rmpath('./third_party/utils')
rmpath('./third_party/urdfs')