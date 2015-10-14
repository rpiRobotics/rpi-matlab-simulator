%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function joint = Joint( )

    joint = struct; 
    
    joint.jointID = 0;          % A unique ID
    joint.type = '';
    joint.jointType = 0;        % Integer code representing joint type
    joint.mask = zeros(6,1);    % A column mask for selecting joint constraints based on type
    
    joint.body1id = 0;          % ID of first joined body
    joint.body2id = 0;          % ID of second joined body
    joint.constraintIndex = 0;  % Column position of this joint in the bilateral constraint matrix
    joint.numConstraints = 0;   % Number of constraints, dependent on joint type
    joint.numPreviousConstraints = 0; % Number of constraints from joints with lower constraintIndex 
    
    joint.pos = [0 0 0];        % Initial world position of joint when initialized
    joint.Xdir = [0 0 0];       % Initial x-direction of joint
    joint.Ydir = [0 0 0];       % Initial y-direction 
    joint.Zdir = [0 0 0];       % Initial z-direction (always considered the AXIS of the joint)
    joint.theta = 0;            % The current value of the joint "angle" 
    joint.theta_prev = 0;

    %joint.linkedJoints = [];    % List of jointIDs of joints that are in a chain with this joint

    joint.mu = 0.5;             % Joint friction

    joint.quat1 = [1 0 0 0];    % Quaternion orientation of joint frame from body1 to joint
    joint.jointPos1 = [0 0 0];  % Position of joint frame from body1 COM to joint
    joint.T1 = zeros(4);        % Transformation matrix from body1 COM to joint on body1
    joint.T1world = zeros(4);

    joint.quat2 = [1 0 0 0];    % Quaternion orientation of joint frame from body1 to joint
    joint.jointPos2 = [0 0 0];  % Position of joint frame from body1 COM to joint
    joint.T2 = zeros(4);        % Transformation matrix from body2 COM to joint on body2
    joint.T2world = zeros(4);

    joint.P1 = [0 0 0];         % Position of joint as body1 sees it, in WORLD coordinates
    joint.X1 = [0 0 0];         % Position of X=1 on body1's joint frame, in WORLD coordinates
    joint.Y1 = [0 0 0];         % Position of Y=1
    joint.Z1 = [0 0 0];         % Position of Z=1
    joint.V1 = [0 0 0];

    joint.P2 = [0 0 0];         % Position of joint as body2 sees it, in WORLD coordinates
    joint.X2 = [0 0 0];         % Position of X=1 on body2's joint frame, in WORLD coordinates
    joint.Y2 = [0 0 0];         % Position of Y=1
    joint.Z2 = [0 0 0];         % Position of Z=1
    joint.V2 = [0 0 0];
    
    % Graphics% Graphics data
    %joint.drawn = false; 
    joint.P1Handle = [];
    joint.P1_X_AxisHandle = [];
    joint.P1_Y_AxisHandle = [];
    joint.P1_Z_AxisHandle = [];
    joint.P1_Z_pointHandle = [];
    joint.P2Handle = [];
    joint.P2_X_AxisHandle = [];
    joint.P2_Y_AxisHandle = [];
    joint.P2_Z_AxisHandle = [];
    joint.P2_Z_pointHandle = [];
    joint.labelHandle = [];
    
end

