%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% INPUTS:
%           sim             - A Simulation struct containing body1 and body2
%           body1           - A body struct 
%           body2           - A body struct
%           joint_origin    - The world-frame initial position of the joint
%           joint_axis      - The world-frame initial direction of the
%                               joint axis
%           jointType       - A string matching one of the following:
%                               'spherical'
%                               'prismatic'
%                               'revolute'
%                               'cylindrical'
%                               'rigid'
function sim = sim_addJoint( sim, body1_id, body2_id, joint_origin, joint_axis, jointType )
    
    body1 = sim.bodies(body1_id);
    body2 = sim.bodies(body2_id); 
    
    if ~body1.dynamic && ~body2.dynamic
       error('Cannot create a joint between two static bodies.'); 
    end
    
    %% Initialize joint values for bodies 1 and 2
    joint_axis = joint_axis / norm(joint_axis);  
    q = [ 1+dot([0;0;1],joint_axis); cross3([0;0;1], joint_axis) ];
    q = q/norm(q);                                        % Rotation to joint frame from world frame

    pos1 = qt2rot(qtinv(body1.quat)) * (joint_origin-body1.u); % Joint position calculated by body1 (world frame)
    pos2 = qt2rot(qtinv(body2.quat)) * (joint_origin-body2.u); % Joint position calculated by body2 (world frame)
    quat1 = qtmultiply(qtinv(body1.quat), q);         % Joint rotation calculated by body1 (world frame)
    quat2 = qtmultiply(qtinv(body2.quat), q);         % Joint rotation calculated by body2 (world frame)
    
    
    %% Create joint 
    
    joint = Joint(); 
    joint.type = jointType;         
    joint.body1id = body1.bodyID;
    joint.body2id = body2.bodyID;
           
    joint.jointPos1 = pos1;
    joint.quat1 = quat1;
    
    joint.jointPos2 = pos2;
    joint.quat2 = quat2; 
    
    % Init joint transformations()
    joint.T1 = zeros(4,4);
    joint.T1(4,4) = 1;
    joint.T1(1:3,1:3) = qt2rot(joint.quat1);
    joint.T1(1:3,4) = joint.jointPos1; 

    joint.T2 = zeros(4,4); 
    joint.T2(4,4) = 1;
    joint.T2(1:3,1:3) = qt2rot(joint.quat2);
    joint.T2(1:3,4) = joint.jointPos2;

    joint.pos = joint.P1;       % Since bodies have not yet moved, p1 = p2
    joint.Xdir = joint.X1 - joint.P1;
    joint.Ydir = joint.Y1 - joint.P1;
    joint.Zdir = joint.Z1 - joint.P1; 

    joint.linkedJoints = []; 
    joint.mu = body1.mu * body2.mu; 
    joint.drawn = false; 

    % Mask columns to obtain specific joint types
    if strcmp(joint.type, 'fixed')
       joint.mask = [1 2 3 4 5 6];    joint.jntCode = 1;
    elseif strcmp(joint.type, 'revolute')
       joint.mask = [1 2 3 4 5  ];    joint.jntCode = 2;
    elseif strcmp(joint.type, 'prismatic')
       joint.mask = [1 2   4 5 6];    joint.jntCode = 3;
    elseif strcmp(joint.type, 'cylindrical')
       joint.mask = [1 2   4 5  ];    joint.jntCode = 4;
    elseif strcmp(joint.type, 'spherical')
       joint.mask = [1 2 3      ];    joint.jntCode = 5;
    end
    joint.numPreviousConstraints = sim.num_jointConstraints; 
    joint.numConstraints = length(joint.mask); 

    joint.theta = 0;
    joint.theta_prev = 0;

    %% Initialize transformations
    joint.T1 = zeros(4,4);
    joint.T1(4,4) = 1;
    joint.T1(1:3,1:3) = qt2rot(joint.quat1);
    joint.T1(1:3,4) = joint.jointPos1; 

    joint.T2 = zeros(4,4); 
    joint.T2(4,4) = 1;
    joint.T2(1:3,1:3) = qt2rot(joint.quat2);
    joint.T2(1:3,4) = joint.jointPos2;
    
    %% Add joint to sim
sim.joints
joint
    sim.joints = [sim.joints joint]; 
    sim.joints(end).jointID = length(sim.joints); 
    sim = updateJoint( sim, length(sim.joints) );  % Necessary? 

    sim.joints(end).constraintIndex = ...
      sim.num_jointConstraints+1 : sim.num_jointConstraints + sim.joints(end).numConstraints;
    sim.num_jointConstraints = sim.num_jointConstraints + sim.joints(end).numConstraints; 

    if body1.dynamic && body1.numJoints == 0
      sim.num_activeJointBodies = sim.num_activeJointBodies + 1; 
    end
    if body2.dynamic && body2.numJoints == 0
     sim.num_activeJointBodies = sim.num_activeJointBodies + 1; 
    end
    
    
    % Tell the objects to not collide with one another
    sim.bodies(body1_id).doesNotCollideWith = [sim.bodies(body1_id).doesNotCollideWith body2.bodyID];
    sim.bodies(body2_id).doesNotCollideWith = [sim.bodies(body2_id).doesNotCollideWith body1.bodyID];
    sim.bodies(body1_id).numJoints = body1.numJoints+1;
    sim.bodies(body2_id).numJoints = body1.numJoints+1;

end











