%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run tests to verify functions within the RBDyn class. The test compares
% methods from RBDyn that is derived using Featherstone's spatial notation
% versus the lagrangian dynamics approach
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;
addpath('./tests');
testCase = RBDyn_test;
run(testCase)
rmpath('./tests');