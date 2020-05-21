classdef RBDyn < handle
  %RBDyn Generates the rigid body dynamics from a urdf file
  
  properties
    % system properties
    smds
    model
    
    % payload parameters
    payload
    payload_free
    
    % kinematics
    dof % total degrees of freedom
    q % joint position
    dq % joint velocities
    R_0N % rotation matrix expressing N in 0
    P_0N % position vector from origin of 0 to N
    X_N0 % plucker transform relating 0 and N
    R_0E % rotation matrix expressing E in 0
    P_0E % position vector from origin of 0 to E
    X_E0 % plucker transform relating 0 and E
    
    J_0N % basic/analytic jacobian expressing velocities at N in 0
    J_0E % basic/analytic jacobian expressing velocities at E in 0
    
    % joint space dynamics (\tau = M(q) ddq + H(dq,q) where H(dq,q) =
    % C(dq,q) + G(q))
    mass_matrix % M(q)
    mass_matrix_inv % M(q)^-1
    nonlinear_terms % H(dq,q) = C(dq,q) + G(q)
    cor_cfg_terms % C(dq,q)
    gravity_terms % G(q)
    bias_acc_terms % h(dq,q)
    
    % operational space dynamics (F = Lambda(x) ddx + mu(x,dx) + p(x)
    lambda_matrix % Lambda(x)
    lambda_matrix_inv % Lambda(x)^-1
    jacobian_bar % Jbar
    mu_terms % mu(x,dx)
    p_terms % p(x)
  end
  
  methods (Access = public)
    function obj = RBDyn(filename)
      %RBDyn Construct an instance of this class
      %   Takes in a urdf file from the urdfs folder, parses all arguments
      %   and sets up the robotic system. Only serial chains are supported.
      %   TODO: closed chains, floating base and multi-effectors
      
      % load URDF
      obj.model = xml2struct(filename);
      
      % number of moving bodies
      obj.smds.NB = size(obj.model.robot.joint,2);
      if obj.smds.NB == 1
        obj.model.robot.joint = num2cell(obj.model.robot.joint);
      end
      
      % initializations
      obj.smds.parent = zeros(1,obj.smds.NB);
      obj.smds.jtype = {};
      
      % generation loop
      for i = 1:obj.smds.NB
        % parents
        for j = 1:obj.smds.NB+1
          if strcmp(obj.model.robot.joint{i}.parent.Attributes.link,obj.model.robot.link{j}.Attributes.name)
            obj.smds.parent(i) = j-1;
          end
        end
        
        % jType
        if strcmp(obj.model.robot.joint{i}.Attributes.type,'revolute') || strcmp(obj.model.robot.joint{i}.Attributes.type,'continuous')
          if strcmp(obj.model.robot.joint{i}.axis.Attributes.xyz,'1 0 0') || strcmp(obj.model.robot.joint{i}.axis.Attributes.xyz,'-1 0 0')
            obj.smds.jtype{i} = 'Rx';
          elseif strcmp(obj.model.robot.joint{i}.axis.Attributes.xyz,'0 1 0') || strcmp(obj.model.robot.joint{i}.axis.Attributes.xyz,'0 -1 0')
            obj.smds.jtype{i} = 'Ry';
          elseif strcmp(obj.model.robot.joint{i}.axis.Attributes.xyz,'0 0 1') || strcmp(obj.model.robot.joint{i}.axis.Attributes.xyz,'0 0 -1')
            obj.smds.jtype{i} = 'Rz';
          end
        end
        
        % inertia
        com = str2num(obj.model.robot.link{i+1}.inertial.origin.Attributes.xyz);
        mass = str2double(obj.model.robot.link{i+1}.inertial.mass.Attributes.value);
        ixx = str2double(obj.model.robot.link{i+1}.inertial.inertia.Attributes.ixx);
        ixy = str2double(obj.model.robot.link{i+1}.inertial.inertia.Attributes.ixy);
        ixz = str2double(obj.model.robot.link{i+1}.inertial.inertia.Attributes.ixz);
        iyy = str2double(obj.model.robot.link{i+1}.inertial.inertia.Attributes.iyy);
        iyz = str2double(obj.model.robot.link{i+1}.inertial.inertia.Attributes.iyz);
        izz = str2double(obj.model.robot.link{i+1}.inertial.inertia.Attributes.izz);
        inertia = [ixx ixy ixz; ixy iyy iyz; ixz iyz izz];
        assert(mass > 0)
        assert((ixx + iyy) > izz)
        assert((ixx + izz) > iyy)
        assert((iyy + izz) > ixx)
        obj.smds.I{i} = mcI(mass,com,inertia);
        
        % transform tree
        angle = str2num(obj.model.robot.joint{i}.origin.Attributes.rpy);
        disp = str2num(obj.model.robot.joint{i}.origin.Attributes.xyz);
        obj.smds.Xtree{i} = rotx(angle(1))*roty(angle(2))*rotz(angle(3)) * xlt(disp);
        
        % meshes
        obj.smds.mesh{i} = obj.model.robot.link{i+1}.visual.geometry.mesh.Attributes.filename;
      end
      
      % base link mesh
      obj.smds.base_mesh = obj.model.robot.link{1}.visual.geometry.mesh.Attributes.filename;
      
      % gravity defaulted to pointing down
      obj.smds.gravity = [0; 0; -9.81];
      
      % payload
      obj.payload.mass = 0;
      obj.payload.com = zeros(3,1);
      obj.payload.inertia = zeros(3,3);
      
      % payload_free
      [m, c, i] = mcI(obj.smds.I{end});
      obj.payload_free.mass = m;
      obj.payload_free.com = c;
      obj.payload_free.inertia = i;
      
      % initialization
      obj.dof = obj.smds.NB;
      obj.q = zeros(obj.dof,1);
      obj.dq = zeros(obj.dof,1);
      obj.R_0N = eye(3);
      obj.P_0N = zeros(3,1);
      obj.X_N0 = eye(6);
      obj.R_0E = eye(3);
      obj.P_0E = zeros(3,1);
      obj.X_E0 = eye(6);
      obj.J_0N = zeros(6, obj.dof);
      obj.J_0E = zeros(6, obj.dof);
    end

    function setJointPosition(obj,q)
      assert((size(q,1) == obj.dof) && (size(q,2) == 1))
      obj.q = q;
    end
    
    function setJointVelocity(obj,dq)
      assert((size(dq,1) == obj.dof) && (size(dq,2) == 1))
      obj.dq = dq;
    end
    
    function calcPose(obj)
      %calcKinematics Calculates forward kinematics
      %   Calculates the mapping from joint space to task space
      %   pose of last link frame
      
      % plucker transform to the last link, i.e. ^N X_0
      b = obj.dof;
      obj.X_N0 = eye(6);
      while b > 0
        XJ = jcalc(obj.smds.jtype{b}, obj.q(b));
        obj.X_N0 = obj.X_N0*XJ*obj.smds.Xtree{b};
        b = obj.smds.parent(b);
      end
      
      % convert to homogeneous cordinates, i.e. ^N X_0 to (0_R_N, 0_p_N)
      T_N0 = pluho(obj.X_N0);
      obj.R_0N = transpose(getR(T_N0));
      obj.P_0N = -obj.R_0N*getP(T_N0);
    end
    
    function calcPoseAt(obj, T_NE, calcPoseFlag)
      %calcKinematics Calculates forward kinematics at a given point in the
      %last link frame
      %   Calculates the mapping from joint space to task space
      %   pose of a given frame at last link
      
      % calculate forward kinematics if flag is set
      if (calcPoseFlag)
        obj.calcPose();
      end
      
      % construct plucker transform
      R_EN = transpose(getR(T_NE));
      N_r_EN = getP(T_NE);
      X_EN = blkdiag(R_EN, R_EN)*xlt(N_r_EN);
      obj.X_E0 = X_EN*obj.X_N0;
      
      % convert to homogeneous cordinates, i.e. ^E X_0
      T_E0 = pluho(obj.X_E0);
      obj.R_0E = transpose(getR(T_E0));
      obj.P_0E = -obj.R_0E*getP(T_E0);
    end
    
    function Jb = calcBodyJacobian(obj, body)
      %calcBodyJacobian Calculates body jacobian expressed in frame 0
      %   Calculates the mapping from joint space to task space velocities
      %   using the body jacobian. The body jacobian is constructed based
      %   on the joints that contribute to the motion of the given body.
      e = zeros(1, obj.dof);
      while body ~= 0
        e(body) = 1;
        body = obj.smds.parent(body);
      end
      Jb = zeros(6, obj.dof);
      i_X_p = cell(1, obj.dof);
      for i = 1:obj.dof
        if e(i)
          [XJ, S] = jcalc(obj.smds.jtype{i}, obj.q(i));
          i_X_p{i} = XJ*obj.smds.Xtree{i}; % ^i X_{i-1}
          if obj.smds.parent(i) ~= 0
            i_X_p{i} = i_X_p{i} * i_X_p{obj.smds.parent(i)};% e.g. ^2 X_1 * ^1 X_0
          end
          Jb(:,i) = i_X_p{i} \ S; % inv(^b X_0) * S to express body J in frame 0
        end
      end
    end
    
    function calcBasicJacobianN(obj)
      %calcBasicJacobianN Calculates basic/analytical jacobian of frame N 
      %expressed in frame 0
      Jb = obj.calcBodyJacobian(obj.dof);
      
      % map twist to the commonly used 3D vectors (w, v).
      % we can view this set of velocities as the plucker coordinates of
      % the end effector's spatial velocity expressed in a coordinate
      % system that is parallel to absolute coordinates but has its origin
      % translated by r_0N to the particular point in the end effector to
      % which v in (w, v) refers to.
      % This is thus equivalent to translating the motion vector from 0 to
      % N, i.e. using:
      obj.J_0N = xlt(obj.P_0N)*Jb;
    end
    
    function calcBasicJacobianE(obj)
      %calcBasicJacobianE Calculates basic jacobian of frame E expressed in
      %frame 0
      Jb = obj.calcBodyJacobian(obj.dof);
      obj.J_0E = xlt(obj.P_0E)*Jb;
    end
    
    function calcBiasAccelerationN(obj)
      %calcBiasAccelerationN Calculates the bias acceleration given by h =
      %dJ*dq of frame N
      %   Calculates the bias acceleration by first finding all the spatial
      %   body velocities and accelerations. Finally, we make a spatial
      %   transform of the acceleration at the final body.
      i_X_p = cell(1, obj.dof);
      v = cell(1, obj.dof);
      a = cell(1, obj.dof);
      for i = 1:obj.dof
        [XJ, S{i}] = jcalc(obj.smds.jtype{i}, obj.q(i));
        vJ = S{i}*obj.dq(i);
        i_X_p{i} = XJ*obj.smds.Xtree{i}; % ^i X_{i-1}
        if obj.smds.parent(i) == 0
          v{i} = vJ;
          a{i} = zeros(6,1);
          N_X_0 = i_X_p{i};
        else
          v{i} = i_X_p{i}*v{obj.smds.parent(i)} + vJ;
          a{i} = i_X_p{i}*a{obj.smds.parent(i)} + crm(v{i})*vJ;
          N_X_0 = i_X_p{i}*N_X_0;
        end
      end

      % pXi
      irp = [0;0;0];
      E = transpose(N_X_0(1:3,1:3));
    	v_sp = v{obj.smds.NB};
      pXi = plux(E,irp);
      pVi = pXi*v_sp;

      % spatial accleration of last link, i.e. xbias = dJ*dq;
      v = pVi(4:6,1);
      w = pVi(1:3,1);
      a_dash = [zeros(3,1); cross(w,v)];
      obj.bias_acc_terms = pXi*a{obj.smds.NB}+a_dash;
    end
    
    function calcJointSpaceDyn(obj)
      %calcJointSpaceDyn Calculates joint space dynamic paramters
      
      % Mass matrix
      [obj.mass_matrix, obj.nonlinear_terms] = HandC(obj.smds, obj.q, obj.dq);
      obj.mass_matrix_inv = invSPD(obj.mass_matrix);
      % Gravity
      [~, obj.gravity_terms] = HandC(obj.smds, obj.q, zeros(obj.dof,1));
      % Coriolis/Centrifugal
      obj.cor_cfg_terms = obj.nonlinear_terms - obj.gravity_terms;
    end
    
    function calcOperationalSpaceDynE(obj)
      %calcOperationalSpaceDynE Calculates operational space dynamic
      %parameters at frame E expresed in frame 0
      
      % Pseudo kinetic energy matrix
      % TODO: this can be solved easily with AB algorithm
      J = obj.J_0E;
      Jt = transpose(J);
      obj.lambda_matrix_inv = J*obj.mass_matrix_inv*Jt;
      obj.lambda_matrix = invSPD(obj.lambda_matrix_inv);
      % Dynamically consistent jacobian
      obj.jacobian_bar = obj.mass_matrix_inv*Jt*obj.lambda_matrix;
      Jbart = transpose(obj.jacobian_bar);
      % Mu
      % TODO: -obj.lambda_matrix*dJ*dq can be found as the bias force
      % in the AB algorithm
      obj.mu_terms = Jbart*obj.cor_cfg_terms;
      % p
      obj.p_terms = Jbart*obj.gravity_terms;
    end
    
    function g = getGravity(obj)
      %getGravity Returns gravity vector expressed in frame 0
      g = obj.smds.gravity;
    end
    
    function setGravity(obj, g)
      %setGravity Sets gravity vector expressed in frame 0
      obj.smds.gravity = g;
    end
    
    function resetPayload(obj)
      %resetPayload Resets payload parameters and update inertial parameters
      obj.smds.I{obj.smds.NB} = mcI(obj.payload_free.mass, ...
        obj.payload_free.com, obj.payload_free.inertia);
    end
    
    function setPayload(obj, mass, inertia, T_NPcom)
      %setPayload Sets payload parameters and update inertial parameters
      assert(mass > 0)
      assert((size(T_NPcom, 1) == 4) && (size(T_NPcom, 2) == 4))
      assert((size(inertia,1) == 3) && (size(inertia,2) == 3))
      assert(max(max(abs(inertia-transpose(inertia)))) < eps)
      assert((inertia(1,1) + inertia(2,2)) > inertia(3,3))
      assert((inertia(1,1) + inertia(3,3)) > inertia(2,2))
      assert((inertia(2,2) + inertia(3,3)) > inertia(1,1))
      
      % set payload
      obj.payload.mass = mass;
      obj.payload.com = getP(T_NPcom);
      obj.payload.frame = getR(T_NPcom);
      obj.payload.inertia = inertia;
      
      % recompute inertial parameter for last link
      [m, c, I] = mcI(obj.smds.I{obj.smds.NB});
      
      % compute inertial properties of composite body
      [m, c, I] = obj.calcCompositeBodyInertia(m, obj.payload.mass, ...
        c, obj.payload.com, I, obj.payload.inertia, obj.payload.frame);
      obj.smds.I{end} = mcI(m, c, I);
    end
    
    function displaySystem(obj)
      %displaySystem Displays the robotic system at current configuration    
      figure;
      if ~isempty(obj.smds.base_mesh)
        [vb,fb,~,cb,~] = stlread(obj.smds.base_mesh);
        [vb,fb] = patchslim(vb,fb);
        patch('Faces',fb,'Vertices',transpose(eye(3)*transpose(vb)),...
          'FaceVertexCData',cb,'FaceColor',[1 1 1],'EdgeColor','none',...
          'HandleVisibility','off');
      end
      
      % find all frames for meshes
      i_X_p = cell(1, obj.dof);
      for j = 1:obj.dof
        [XJ, ~] = jcalc(obj.smds.jtype{j}, obj.q(j));
        i_X_p{j} = XJ*obj.smds.Xtree{j}; % ^i X_{i-1}
        if obj.smds.parent(j) ~= 0
          i_X_p{j} = i_X_p{j} * i_X_p{obj.smds.parent(j)};% e.g. ^2 X_1 * ^1 X_0
        end
      end
      
      % display meshes
      v = {}; f = {}; c = {};
      for i = 1:obj.dof
        if ~isempty(obj.smds.mesh{i})
          [v{i},f{i},~,c{i},~] = stlread(obj.smds.mesh{i});
          [v{i},f{i}] = patchslim(v{i},f{i});
          T_i0 = pluho(i_X_p{i});
          R_0i = transpose(getR(T_i0));
          P_0i = -R_0i*getP(T_i0);
          patch('Faces',f{i},'Vertices',transpose(R_0i*transpose(v{i})+...
            P_0i*ones(1,size(v{i},1))),'FaceVertexCData',c{i},'FaceColor',...
            [1 1 1],'EdgeColor','none','HandleVisibility','off');
        end
      end
      
      camlight('HEADLIGHT');
      material('METAL');
      axis manual
      axis('equal');
      axis([-0.4 0.9 -0.6 0.6 -0.2 1.4]);
      zoom(1.2);
      view([60 30]);
      rotate3d on
      hold off
      grid on
      xlabel('x');
      ylabel('y');
      zlabel('z');
    end
  end
  
  methods (Access = private)
    function [comb_mass, comb_com, comb_I] = calcCompositeBodyInertia(~, m1, m2, com1, com2, Ic1, Ic2, R12)
      %calcCompositeBodyInertia calculate inertia of a composite body. Note
      %that all the bodies passed in needs to be in the same reference
      %frame, i.e. frame 1
      
      % new mass
      comb_mass = m1+m2;
      % new com
      comb_com = (m1*com1 + m2*com2)/comb_mass; % in frame 1
      % inertia contribution of body 1 at new com
      r = -comb_com+com1; %r_comb_com_to_com1
      I1_comb_com = Ic1+m1*(r'*r-r*r');
      % for second body, similarity transform to move Ic2 in frame 2 to
      % frame 1
      Ic2_1 = R12*Ic2*transpose(R12);
      % inertia contribution of body 2 at new com
      r = -comb_com+com2; %r_comb_com_to_com2
      I2_comb_com = Ic2_1+m2*(r'*r-r*r');
      % total inertia
      comb_I = I1_comb_com+I2_comb_com;
    end
  end
end