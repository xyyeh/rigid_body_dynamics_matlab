clear all
clc

addpath('./RBDyn/');
addpath('./third_party/xml2struct') % from: https://www.mathworks.com/matlabcentral/fileexchange/28518-xml2struct
addpath('./third_party/Spatial') % from: http://royfeatherstone.org/spatial/v2/index.html
addpath('./third_party/utils')
addpath('./third_party/urdfs') 

% load robot using urdf
r = RBDyn('panda4dof.urdf');

% set position
q_v = [1;2;3;4];
dq_v = [5;6;7;8];
r.setJointPosition(q_v);
r.setJointVelocity(dq_v);
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
T = eye(4);
T(1:3,4) = [0;0;0.2];
r.calcPoseAt(T,1);
disp('R_0E');
disp(r.R_0E)
disp('P_0E');
disp(r.P_0E)
r.calcBasicJacobianE
disp('J_0E');
disp(r.J_0E)

% calculating mass matrix at a particular q
r.calcJointSpaceDyn
disp('r.mass_matrix');
disp(r.mass_matrix)

% calculating gravity and coriolis torque at a particular q
r.calcJointSpaceDyn
disp('r.gravity_terms');
disp(r.gravity_terms)
disp('r.cor_cfg_terms');
disp(r.cor_cfg_terms)

% test bias acceleration
r.calcBiasAccelerationN
disp('r.bias_acc_terms');
disp(r.bias_acc_terms)

disp('basic J*dq');
r.calcBasicJacobianN;
r.J_0N*r.dq

disp('body J*dq');
r.calcBodyJacobian(r.dof)*r.dq



r.getGravity()

rmpath('./RBDyn/');
rmpath('./third_party/xml2struct') % from: https://www.mathworks.com/matlabcentral/fileexchange/28518-xml2struct
rmpath('./third_party/Spatial') % from: http://royfeatherstone.org/spatial/v2/index.html
rmpath('./third_party/utils')
rmpath('./third_party/urdfs')