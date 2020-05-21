classdef RBDyn_test < matlab.unittest.TestCase
  % RBDyn_test to test functions in RBDyn
  properties
    rb
    tol
    q
    dq
  end
  
  methods(TestMethodSetup)
    function createObject(testCase)
      import matlab.unittest.fixtures.PathFixture
      testCase.applyFixture(PathFixture('./RBDyn'));
      testCase.applyFixture(PathFixture('./third_party/','IncludingSubfolders',true));
      
      % instantiate object
      testCase.rb = RBDyn('panda4dof.urdf');
      
      % setup tolerance
      testCase.tol = 1e-5;
      
      % setup joint states
      testCase.q = [1;2;3;4];
      testCase.dq = [5;6;7;8];
    end
  end
  
  methods (Test)
    function testLoadURDF(testCase)
      expVal = 4;
      testCase.verifyEqual(testCase.rb.dof,expVal);
    end
    
    function testCalcPose(testCase)
      testCase.rb.setJointPosition(testCase.q);
      testCase.rb.calcPose();
      
      expVal = [
        -0.439692302494695  -0.242540827567088  -0.864780102737098
        -0.855503489096755  -0.180067887430341   0.485478460963668
        -0.273467473879787   0.953283537505734  -0.128320060202457];
           
      for i = 1:numel(expVal)
        testCase.verifyEqual(testCase.rb.R_0N(i),expVal(i),'AbsTol',testCase.tol);
      end
      
      expVal = [   
        0.163816718498119
        0.276677360958797
        0.275763904104283];
      for i = 1:numel(expVal)
        testCase.verifyEqual(testCase.rb.P_0N(i),expVal(i),'AbsTol',testCase.tol);
      end
    end
    
    function testCalcJacobian(testCase)
      testCase.rb.setJointPosition(testCase.q);
      testCase.rb.calcPose();
      testCase.rb.calcBasicJacobianN();
      
      expVal = [
                       0  -0.841470984807897   0.491295496433882  -0.864780102737098
                       0   0.540302305868140   0.765147401234293   0.485478460963668
       1.000000000000000                   0  -0.416146836547142  -0.128320060202457
      -0.276677360958797  -0.030924794591346   0.071344358475811                   0
       0.163816718498119  -0.048162513979928  -0.040051973029503                   0
                       0  -0.321326522144334   0.010586404966703                   0];
      
      for i = 1:numel(expVal)
          testCase.verifyEqual(testCase.rb.J_0N(i),expVal(i),'AbsTol',testCase.tol);
      end
    end
    
    function testCalcBiasAcceleration(testCase)
      testCase.rb.setJointPosition(testCase.q);
      testCase.rb.setJointVelocity(testCase.dq);
      testCase.rb.calcBiasAccelerationN();
      
      expVal = [
       -69.364370986669414
       -38.846846929261048
        15.039454295036155
        -4.591445184653463
       -14.473610134141838
        -1.985525115308438];
      
      for i = 1:numel(expVal)
        testCase.verifyEqual(testCase.rb.bias_acc_terms(i),expVal(i),'AbsTol',testCase.tol);
      end
    end
    
    function testMassMatrix(testCase)
      testCase.rb.setJointPosition(testCase.q);
      testCase.rb.calcJointSpaceDyn();
      expVal = [
           0.530514222631619   0.001976438195256  -0.152168185488270  -0.004079684299010
           0.001976438195256   0.532483370661369  -0.019742233100848   0.029284648485162
          -0.152168185488270  -0.019742233100848   0.070903231683112  -0.000114700166806
          -0.004079684299010   0.029284648485162  -0.000114700166806   0.016044212482595];
      
      for i = 1:numel(expVal)
        testCase.verifyEqual(testCase.rb.mass_matrix(i),expVal(i),'AbsTol',testCase.tol);
      end
    end
    
  end
end