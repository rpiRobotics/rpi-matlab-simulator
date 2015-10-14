%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
%

function sim = powerball(  )

    %% Load the saved meshes (this takes a few seconds)
    load('powerball.mat');
    
    %% Create mesh bodies, and set positions and orientations
    B1 = Body_mesh( robo_base.verts, robo_base.faces); 
        B1.dynamic = false; 
        B1.color = [.7 .7 .4]; 
        B1.u = robo_base.u; 
        B1.edgealpha = 0.05; 
    
    B3 = Body_mesh( robo_arm1.verts, robo_arm1.faces);
        B3.u = robo_arm1.u;
        B3.color = [.2 .2 .7];
        B3.edgealpha = 0; 
        
    B6 = Body_mesh( robo_erb115.verts, robo_erb115.faces);
        B6.u = robo_erb115.u; 
        B6.color = [.7 .7 .7]; 
        B6.edgealpha = 0.1; 
        
    B2 = Body_mesh( robo_erb1451.verts, robo_erb1451.faces);
        B2.u = robo_erb1451.u; 
        B2.color = [.7 .7 .7]; 
        B2.edgealpha = 0.1; 
    
    B4 = Body_mesh( robo_erb1452.verts, robo_erb1452.faces); 
        B4.u = robo_erb1452.u; 
        B4.color = [.7 .7 .7];
        B4.edgealpha = 0.1; 
    
    B5 = Body_mesh( robo_larm.verts, robo_larm.faces); 
        B5.u = robo_larm.u; 
        B5.color = [.2 .2 .7];
        B5.edgealpha = 0; 
        
    %jointAngle = [90;10;45;45;25;45];
    jointAngle = [0;0;0;0;0;0];
    
    % Define the joints
%     jnt6_pos = [0; 0; .860];
%     jnt6_axis = [0;0 ; 1];

    jnt5_pos = [0; 0; .860];
    jnt5_axis = [1;0 ; 0]; 

    jnt4_pos = [0;0;.555];
    jnt4_axis = [0;0;1];

    jnt3_pos = [0;0;.555];
    jnt3_axis = [1;0;0];

    jnt2_pos = [0;0;.205];
    jnt2_axis = [1;0;0];
    
    jnt1_pos = [0;0;.205];
    jnt1_axis = [0;0;1];

    % Let's store all of our zero-position offsets!
    p1 = B1.u;
    p2 = B2.u-B1.u;
    p3 = B3.u-B2.u;
    p4 = B4.u-B3.u;
    p5 = B5.u-B4.u;
    p6 = B6.u-B5.u;
    %p7 = B7.u-B6.u;

    % Rotation matrices
    R12 = rot( jnt1_axis, jointAngle(1)*pi/180 ); 
    R23 = rot( jnt2_axis, jointAngle(2)*pi/180 ); 
    R34 = rot( jnt3_axis, jointAngle(3)*pi/180 ); 
    R45 = rot( jnt4_axis, jointAngle(4)*pi/180 ); 
    R56 = rot( jnt5_axis, jointAngle(5)*pi/180 ); 
    %R67 = rot( jnt6_axis, jointAngle(6)*pi/180 ); 

    % Update positions
    B1.u = p1;                % Zero offset
    B2.u = p1 + R12*p2;
    B3.u = p1 + R12*p2 + R12*R23*p3;
    B4.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4;  % NO R34!
    B5.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4 + R12*R23*R34*R45*p5;
    B6.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4 + R12*R23*R34*R45*p5 + R12*R23*R34*R45*p6; % NO R56!
    %B7.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4 + R12*R23*R34*R45*p5 + R12*R23*R34*R45*p6 + R12*R23*R34*R45*R56*R67*p7;

    % Update rotations
    B1.quat = [1;0;0;0]; 
    B2.quat = qt( jnt1_axis, jointAngle(1)*pi/180 ); 
    B3.quat = qtmultiply( B2.quat, qt( jnt2_axis, jointAngle(2)*pi/180 ) ); 
    B4.quat = qtmultiply( B3.quat, qt( jnt3_axis, jointAngle(3)*pi/180 ) ); 
    B5.quat = qtmultiply( B4.quat, qt( jnt4_axis, jointAngle(4)*pi/180 ) ); 
    B6.quat = qtmultiply( B5.quat, qt( jnt5_axis, jointAngle(5)*pi/180 ) ); 
    %B7.quat = qtmultiply( B6.quat, qt( jnt6_axis, jointAngle(6)*pi/180 ) ); 
    
    %% Group bodies, and turn off self collision with the arm
    %P = Body_plane([0;0;0],[0;0;1]); 
    b = mesh_cube(); b.visible = false; b.u = [0;0;0.5]; b.dynamic = false; 
    
    bodies = [B1, B2, B3, B4, B5, B6, b];
    for b=1:length(bodies), bodies(b).doesNotCollideWith = 1:7; end
    
    %% Initialize simulator
    sim = Simulator();
    sim.H_dynamics = @mLCPdynamics; 
    sim = sim_addBody( sim, bodies );
    sim.userFunction = @powerball_controller; 
    %sim.drawJoints = true; 
    sim.MAX_STEP = 320; 
    
    %% Add joints
    sim = sim_addJoint(sim,1,2,jnt1_pos,jnt1_axis,'revolute');
    sim = sim_addJoint(sim,2,3,jnt2_pos,jnt2_axis,'revolute');
    sim = sim_addJoint(sim,3,4,jnt3_pos,jnt3_axis,'revolute');
    sim = sim_addJoint(sim,4,5,jnt4_pos,jnt4_axis,'revolute');
    sim = sim_addJoint(sim,5,6,jnt5_pos,jnt5_axis,'revolute');
    
    %% Run simulation 
    sim.MAX_STEP = 330; 
    sim = sim_run( sim ); 

end

